from typing import List, Optional
import numpy as np
import sys
import math
from copy import copy
from queue import PriorityQueue


EARTH_RADIUS = 6371000
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
METERS_PER_DEGREE_LAT = 111139.0


def latLongToMeters(old_lat, old_long, current_lat, current_long, 
		heading):
	delta_lat = (current_lat - old_lat) * DEG_TO_RAD
	delta_long = (current_long - old_long) * DEG_TO_RAD

	heading_rad = heading * DEG_TO_RAD
	dy = delta_lat * EARTH_RADIUS

	mean_lat = (current_lat + old_lat) / 2.0 * DEG_TO_RAD

	dx = delta_long * EARTH_RADIUS * np.cos(mean_lat)

	dx_local = dx * np.cos(heading_rad) + dy * np.sin(heading_rad)
	dy_local = -dx * np.sin(heading_rad) + dy * np.cos(heading_rad)
	return dx_local, dy_local


class TrackedObject:
	use: bool = False
	numpoints: int = 0
	xpos: float = 0.0
	ypos: float = 0.0
	latitude: float = 0.0
	longitude: float = 0.0
	min_long: float = 0.0
	max_long: float = 0.0
	min_lat: float = 0.0
	max_lat: float = 0.0

	tlx: float = sys.float_info.max
	tly: float = sys.float_info.max
	brx: float = -sys.float_info.max
	bry: float = -sys.float_info.max

	area: float = 0.0

	doppler_dir: float = 0.0
	status: int = 0  # 0=unconfirmed, 1=confirmed, 2=deleted
	classification: int = 0  #0: undefined, 1: stationary, 2: moving
	id: int = -1
	score: int = 0
	max_score: int = 0
	# arrays to store the last 10 values for mean
	heading = np.zeros(10, dtype=float)
	turning_rate = np.zeros(10, dtype=float)
	doppler_heading = np.zeros(10, dtype=float)
	vel = np.zeros(10, dtype=float)
	accell = np.zeros(10, dtype=float)
	xpos_mem = np.zeros(10, dtype=float)
	ypos_mem = np.zeros(10, dtype=float)
	history_index: int = 0
	history_min_10: bool = False

	# TODO: store turning rate to add that
	xpos_future = np.zeros(5, dtype=float) # list to keep future predicted positions
	ypos_future = np.zeros(5, dtype=float)
	tlx_future = np.zeros(5, dtype=float)
	brx_future = np.zeros(5, dtype=float)
	tly_future = np.zeros(5, dtype=float)
	bry_future = np.zeros(5, dtype=float)

	est_heading: float = 0.0
	est_heading_deg: float = 0.0
	est_turning_rate: float = 0.0  # rad/s pos clockwise not normal
	est_doppler_heading: float = 0.0
	est_vel: float = 0.0
	est_accell: float = 0.0

	est_xpos: float = 0.0 # not future position but mean current
	est_ypos: float = 0.0

	est_tlx: float = 0.0
	est_tly: float = 0.0
	est_brx: float = 0.0
	est_bry: float = 0.0

	time_birth = 0
	last_update = 0
	
	age: int = 0

	def __init__(self):
		pass

	def __repr__(self):
		return f"OBJ: {self.id}: ({round(self.xpos, 1)}, {round(self.ypos, 1)})"


class ClusterObject:
	cluster_id: int = -1
	matched_id: int = -1
	x_idx: int = 0
	y_idx: int = 0
	right_heavy: bool = False
	top_heavy: bool = False
	left_heavy: bool = False
	bottom_heavy: bool = False
	box_centre_x: float = 0.0
	box_centre_y: float = 0.0
	point_count: int = 0
	max_x: float = 0.0
	min_x: float = 0.0
	max_y: float = 0.0
	min_y: float = 0.0
	mass_center_x: float = 0.0
	mass_center_y: float = 0.0
	doppler_dir: float = 0.0
	check: bool = True

	def __init__(self):
		pass

class MergedClusterObject:
	id: int = 0
	mass_center_x: float = 0.0
	mass_center_y: float = 0.0
	max_x: float = 0.0
	min_x: float  = 0.0
	min_y: float = 0.0
	max_y: float = 0.0
	box_centre_x: float = 0.0
	box_centre_y: float = 0.0
	doppler_dir: float = 0.0  # deg true north
	point_count: int = 0
	box_count: int = 0
	remove: bool = False

	def __init__(self):
		pass


class HDTracker:
	def __init__(self):
		self.all_tracked_objects = []
		self.deleted_ids = PriorityQueue() # change to PQ 
		self.merged_clusters = []
		self.max_object_id = 0
		self.its_to_confirm = 1  # 3 iterations or seconds 
		# self.its_to_delete = 4  # 4 iterations or seconds 
		self.its_to_delete = 1  # 5 iterations or seconds 


	def df_to_obj(self, clusters, timestamp):
		self.merged_clusters = []
		for _, obj in clusters.iterrows():
			mc = MergedClusterObject()
			mc.mass_center_x = obj['mass_center_x']
			mc.mass_center_y = obj['mass_center_y']
			mc.max_x = obj['max_x']
			mc.min_x = obj['min_x']
			mc.max_y = obj['max_y']
			mc.min_y = obj['min_y']
			mc.doppler_dir = 0.0
			mc.point_count = int(obj['point_count'])
			mc.box_count = int(obj['box_count'])
			mc.id = int(obj['id'])
			mc.box_centre_x = obj['box_centre_x']
			mc.box_centre_y = obj['box_centre_y']

			self.merged_clusters.append(mc)


	def create_tracked_object(self, objects, curr_time):
		num_objs: int = 0
		for mc in self.merged_clusters:
			if mc.remove:
				continue

			obj = TrackedObject()
			obj.tlx = mc.min_x
			obj.tly = mc.max_y
			obj.brx = mc.max_x
			obj.bry = mc.min_y

			obj.numpoints = mc.point_count
			obj.score = 0

			if obj.numpoints == 0:
				continue

			obj.time_birth = curr_time

			obj.xpos = mc.mass_center_x  # TODO: evaluate if the mass center is better
			obj.ypos = mc.mass_center_y

			obj.area = abs((obj.brx - obj.tlx) * (obj.bry - obj.tly)) # TODO: abs area

			obj.doppler_dir = mc.doppler_dir

			# Increment objectIds and assign it to obj.id
			# TODO: add possibility to reuse deleted ids
			if not self.deleted_ids.empty():
				obj.id = self.deleted_ids.get()
			else:
				obj.id = self.max_object_id
				self.max_object_id += 1

			obj.use = True
			num_objs += 1
			objects.append(obj)

	def cost_function(self, obj1, obj2, max_dist=16.0**2, max_area_diff=0.20):  # lower is better
		# TODO: maybe weight area or dist
		# 16 meters is eq 16 mps or 30 knots or 55 kph
		center_distance = ((obj1.xpos - obj2.xpos)**2 + (obj1.ypos - obj2.ypos)**2)
		normalized_distance = center_distance / max_dist

		area_diff = np.abs(obj1.area - obj2.area)/obj1.area
		normalized_area_diff = area_diff / max_area_diff

		if normalized_area_diff > 1.0 or normalized_distance > 1.0:
			return float("inf")
		
		return normalized_area_diff + normalized_distance
	
	def remove_multiple_tracked_objects(self, indexes_to_remove):
		# Sort indexes in descending order
		indexes= sorted(indexes_to_remove, reverse=True)

		for index in indexes:
			# Check if index is within bounds
			if 0 <= index < len(self.all_tracked_objects):
				del self.all_tracked_objects[index]
			else:
				raise ValueError(f"Index {index} is out of bounds, skipping.")
		return True

		
	def match_objects(self, new_objects: List[TrackedObject]):
		# New method
		all_new_object_indexes = []
		match_dict = {} # key is all track index, and value is list of new_obj indexes
		for ni, new_obj in enumerate(new_objects):
			all_new_object_indexes.append(ni)
			lowest_cost = float("inf")
			lowest_cost_index = None
			for oi, obj in enumerate(self.all_tracked_objects):
				dt = (new_obj.time_birth - obj.last_update)/1.0e9 # in s
				if dt < 0.8: # then not new object
					continue
				cost = self.cost_function(new_obj, obj)
				if cost < lowest_cost:
					lowest_cost = cost
					lowest_cost_index = oi
			# end old objs
			if lowest_cost_index is not None and lowest_cost < 2.0:
				match_dict.setdefault(lowest_cost_index,[]).append((lowest_cost, ni))

		# no: new object
		# oo: old/all tracked object

		matches_map = [] # 0: old_obj_index, 1: new_object_id, should be merged with existing
		# all new objects should either be matched or not match and become old obj
		for key, val in match_dict.items():
			sorted_nids = sorted(val, key=lambda x: x[0])
			lowest_cost_no_index = sorted_nids[0][1]
			matches_map.append((key,lowest_cost_no_index))
			all_new_object_indexes.remove(lowest_cost_no_index)

		not_matched_indexes = all_new_object_indexes # to all tracked wo match

		# ========================
		# Handle matched objects
		for old_idx, new_idx in matches_map:
			new_obj = new_objects[new_idx]
			obj = self.all_tracked_objects[old_idx]
			obj.last_update = new_obj.time_birth
			obj.age = (obj.last_update - obj.time_birth)/1.0e9

			dx: float = new_obj.xpos - obj.xpos
			dy: float = new_obj.ypos - obj.ypos
			dist: float = np.sqrt(dx**2 + dy**2)
			print(f"Matched object: {obj.id}, new: {new_obj.id}. DIST: {round(dist, 3)} m")
			vel: float = dist / dt  # m/s TODO: must verify this becomes float seconds not something else
			# TODO: compensate for true heading maybe
			heading_rad_raw: float = np.arctan2(dy, dx)
			heading_rad: float = -heading_rad_raw + np.pi/2.0  # TODO: check this, Adjust so that 0 degrees is straight up
			if heading_rad < 0.0:
				heading_rad = np.pi*2-heading_rad

			turning_rate: float = heading_rad/dt

			# TODO: Updated this

			# Convert to a clockwise system (from a counterclockwise one)
			# and ensure the degrees are within the range [0, 360)
			#heading_deg = np.fmod(heading_deg + 360.0, 360.0)
			heading_rad = np.fmod(heading_rad, np.pi*2) # TODO: check this
			
			# Calculate the est vel and heading
			obj.vel[obj.history_index] = vel
			obj.heading[obj.history_index] = heading_rad # TODO: change all heading to rad
			obj.turning_rate[obj.history_index] = turning_rate
			obj.doppler_heading[obj.history_index] = new_obj.doppler_dir
			#obj.accell[obj.history_index] = (vel - obj.vel[(obj.history_index - 1) % 10]) / float(dt/1e9)  # m/s^2
			#obj.accell[obj.history_index] = (vel) / float(dt/1e9)  # m/s^2
			obj.accell[obj.history_index] = vel / dt  # m/s^2
			obj.xpos_mem[obj.history_index] = new_obj.xpos
			obj.ypos_mem[obj.history_index] = new_obj.ypos

			sum_head: float = 0.0
			sum_turning_rate: float = 0.0
			sum_doppler_head: float = 0.0
			sum_vel: float = 0.0
			sum_accell: float = 0.0
			sum_xpos: float = 0.0
			sum_ypos: float = 0.0

			
			if not obj.history_min_10:
				its: int = obj.history_index+1
				for i in range(its):
					sum_head += obj.heading[i] # TODO: fix heading avg not same as vel
					sum_turning_rate += obj.turning_rate[i] 
					sum_doppler_head += obj.doppler_heading[i]
					sum_vel += obj.vel[i]
					sum_accell += obj.accell[i]
					sum_xpos += obj.xpos_mem[i]
					sum_ypos += obj.ypos_mem[i]

				# maybe check lat and long vel check
				# TODO: change so that if less than 10 obs take those
				obj.est_vel = sum_vel / float(its)
				obj.est_heading = sum_head / float(its)
				obj.est_turning_rate = sum_turning_rate / float(its)
				obj.est_doppler_heading = sum_doppler_head / float(its)
				obj.est_accell = sum_accell / float(its)
				obj.est_xpos = sum_xpos / float(its)
				obj.est_ypos = sum_ypos / float(its)


				
				if obj.history_index == 9 and (obj.history_index + 1) % 10 == 0:
					obj.history_min_10 = True
			else:
				for i in range(10):
					sum_head += obj.heading[i]
					sum_turning_rate += obj.turning_rate[i] 
					sum_doppler_head += obj.doppler_heading[i]
					sum_vel += obj.vel[i]
					sum_accell += obj.accell[i]
				# maybe check lat and long vel check
				# TODO: change so that if less than 10 obs take those
				obj.est_vel = sum_vel / 10.0	
				obj.est_heading = sum_head / 10.0
				obj.est_turning_rate = sum_turning_rate / 10.0
				obj.est_doppler_heading = sum_doppler_head / 10.0
				obj.est_accell = sum_accell / 10.0
				obj.est_xpos = sum_xpos / 10.0
				obj.est_ypos = sum_ypos / 10.0

			obj.history_index = (obj.history_index + 1) % 10

			# TODO: check this
			#obj.est_xpos = obj.xpos + (obj.est_vel * np.cos(obj.est_heading * np.pi / 180.0) * dt)
			#obj.est_ypos = obj.ypos + (obj.est_vel * np.sin(obj.est_heading * np.pi / 180.0) * dt)
			# TODO: do same for est_xpos as vel, and add future positions
			# obj.est_xpos = new_obj.xpos + dx
			# obj.est_ypos = new_obj.ypos + dy
			obj.est_heading_deg = obj.est_heading * (180/np.pi)
			obj.xpos = new_obj.xpos
			obj.ypos = new_obj.ypos


			obj.tlx = new_obj.tlx
			obj.tly = new_obj.tly
			obj.brx = new_obj.brx
			obj.bry = new_obj.bry


			# update future positions
			# TODO: add bbox future
			for i in range(5):
				next_heading = obj.est_heading + obj.est_turning_rate * dt * (i + 1)
				cdx = (obj.est_vel * np.cos(-next_heading+np.pi/2.0) * dt)
				cdy = (obj.est_vel * np.sin(-next_heading+np.pi/2.0) * dt)
				obj.xpos_future[i] = obj.est_xpos + cdx
				obj.ypos_future[i] = obj.est_ypos + cdy

				obj.tlx_future[i] = obj.tlx + cdx
				obj.tly_future[i] = obj.tly + cdy
				obj.brx_future[i] = obj.brx + cdx
				obj.bry_future[i] = obj.bry + cdy

			
			obj.area = new_obj.area
			obj.num_points = new_obj.numpoints

			obj.score += 10  # TODO: set as parameter in ros

			# Update status for those with score above threshold
			# TODO: add velocity history
			obj.max_score = max(obj.score, obj.max_score-5)




		# ========================
		# Handle not matched
		for not_matched_idx in not_matched_indexes:
			new_obj = new_objects[not_matched_idx]
			new_obj.last_update = new_obj.time_birth
			new_obj.score += 10
			self.all_tracked_objects.append(copy(new_obj))

		to_delete = []

		for obj in self.all_tracked_objects:
			obj.score -= 5

			# TODO: question this
			if (obj.max_score - obj.score) > (5 + 5*self.its_to_delete) or (obj.score < 0):
				obj.status = 2  # deleted
				to_delete.append(obj.id) # add id instead of index
			elif obj.score > (5*self.its_to_confirm):  # TODO: set as parameter in ros
				obj.status = 1  # confirmed

			if obj.status == 1:
				# TODO: check that heading is not jumping
				#min_vel = 0.51444*4 	# 1 knot
				#min_vel = 4 			# 4 mps
				min_vel = 0.277778*10	# 10 kph
				if obj.est_vel > min_vel: # 1 knot TODO: set as parameter and sync with doppler
					obj.classification = 2  # moving
				else:
					obj.classification = 1  # stationary TODO: must verify that object is confirmed!
			else:
				obj.classification = 0

		to_delete.sort()
		indexes_to_del = []
		for delete_id in to_delete:
			for index, obj in enumerate(self.all_tracked_objects):
				if delete_id == obj.id:
					self.deleted_ids.put(delete_id) # TODO: should be push, i on top of stack
					# print("Deleting ID:", delete_id)
					indexes_to_del.append(index) # TODO: this wont work bc changing index

		#Deletion
		self.remove_multiple_tracked_objects(indexes_to_del)
		# END OF MATCH OBJECTS
		# ========================

	

	def estimate_object_position(self, lat, lon, td):
		x_diff: float = 0.0
		y_diff: float = 0.0
		last_lat = 0.0  # TODO: update!
		last_lon = 0.0
		heading = 0.0 # TODO: update

		x_diff, y_diff = latLongToMeters(last_lat, last_lon, lat, lon, heading)
		# TODO: this is not working probably

		tcos = 0.0
		tsin = 0.0

		for to in self.all_tracked_objects:
			if to.est_vel > 0:
				tcos = to.est_vel * np.cos(to.est_heading * np.pi / 180.0) * td
				tcos = to.est_vel * np.cos(to.est_heading) * td
				tsin = to.est_vel * np.sin(to.est_heading) * td

			to.est_xpos = to.xpos + tcos
			to.est_ypos = to.ypos + tsin
			to.est_tlx = to.tlx + tcos
			to.est_tly = to.tly + tsin
			to.est_brx = to.brx + tcos
			to.est_bry = to.bry + tsin

			# update with egomotion
			to.est_xpos -= x_diff
			# to.est_xpos += x_diff
			to.est_ypos -= y_diff


	def iteration(self):
		# 1. Get detections
		# 2. Get objects
		# 3. Match objects
		pass