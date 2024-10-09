from typing import List, Optional
import numpy as np
import sys
import math
from copy import copy
from queue import PriorityQueue


PI = np.pi
EARTH_RADIUS = 6371000
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI
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


# class TrackedObject:
# 	use: bool = False
# 	numpoints: int = 0
# 	xpos: float = 0.0
# 	ypos: float = 0.0
# 	x_centre: float = 0.0
# 	y_centre: float = 0.0
# 	latitude: float = 0.0
# 	longitude: float = 0.0
# 	min_long: float = 0.0
# 	max_long: float = 0.0
# 	min_lat: float = 0.0
# 	max_lat: float = 0.0

# 	tlx: float = sys.float_info.max
# 	tly: float = sys.float_info.max
# 	brx: float = -sys.float_info.max
# 	bry: float = -sys.float_info.max

# 	area: float = 0.0

# 	doppler_dir: float = 0.0
# 	status: int = 0  # 0=unconfirmed, 1=confirmed, 2=deleted
# 	classification: int = 0  #0: undefined, 1: stationary, 2: moving
# 	id: int = -1
# 	cluster_id: int = -1
# 	score: int = 0
# 	max_score: int = 0
# 	# arrays to store the last 10 values for mean
# 	heading: float = -1.0 # in rad 0 -> 2pi clockwise up is 0
# 	turning_rate: float = 0.0
# 	doppler_heading: float = 0.0
# 	vel: float = -1.0 # should only be pos
# 	accell: float = 0.0
# 	xpos_mem = np.zeros(10, dtype=float)
# 	ypos_mem = np.zeros(10, dtype=float)
# 	history_index: int = 0
# 	history_min_10: bool = False

# 	# TODO: store turning rate to add that
# 	has_future: bool = False
# 	xpos_future = np.zeros(5, dtype=float) # list to keep future predicted positions
# 	ypos_future = np.zeros(5, dtype=float)
# 	tlx_future = np.zeros(5, dtype=float)
# 	brx_future = np.zeros(5, dtype=float)
# 	tly_future = np.zeros(5, dtype=float)
# 	bry_future = np.zeros(5, dtype=float)
# 	x_centre_future = np.zeros(5, dtype=float)
# 	y_centre_future = np.zeros(5, dtype=float)

# 	est_heading: float = 0.0
# 	est_heading_deg: float = 0.0
# 	est_turning_rate: float = 0.0  # rad/s pos clockwise not normal
# 	est_doppler_heading: float = 0.0
# 	est_vel: float = 0.0
# 	est_accell: float = 0.0

# 	est_xpos: float = 0.0 # not future position but mean current
# 	est_ypos: float = 0.0

# 	est_tlx: float = 0.0
# 	est_tly: float = 0.0
# 	est_brx: float = 0.0
# 	est_bry: float = 0.0

# 	time_birth = 0
# 	last_update = 0
	
# 	age: int = 0

# 	def __init__(self):
# 		pass

# 	def __repr__(self):
# 		return f"OBJ: {self.id}: ({round(self.xpos, 1)}, {round(self.ypos, 1)}),S:{self.score}"


# class ClusterObject:
# 	cluster_id: int = -1
# 	matched_id: int = -1
# 	x_idx: int = 0
# 	y_idx: int = 0
# 	right_heavy: bool = False
# 	top_heavy: bool = False
# 	left_heavy: bool = False
# 	bottom_heavy: bool = False
# 	box_centre_x: float = 0.0
# 	box_centre_y: float = 0.0
# 	point_count: int = 0
# 	max_x: float = 0.0
# 	min_x: float = 0.0
# 	max_y: float = 0.0
# 	min_y: float = 0.0
# 	mass_center_x: float = 0.0
# 	mass_center_y: float = 0.0
# 	doppler_dir: float = 0.0
# 	check: bool = True

# 	def __init__(self):
# 		pass

# class MergedClusterObject:
# 	id: int = 0
# 	mass_center_x: float = 0.0
# 	mass_center_y: float = 0.0
# 	max_x: float = 0.0
# 	min_x: float  = 0.0
# 	min_y: float = 0.0
# 	max_y: float = 0.0
# 	box_centre_x: float = 0.0
# 	box_centre_y: float = 0.0
# 	doppler_dir: float = 0.0  # deg true north
# 	point_count: int = 0
# 	box_count: int = 0
# 	remove: bool = False

# 	def __init__(self):
# 		pass

class TrackedObject_old:
	def __init__(self):
		self.use = False
		self.numpoints = 0
		self.xpos = 0.0
		self.ypos = 0.0
		self.x_centre = 0.0
		self.y_centre = 0.0
		self.latitude = 0.0
		self.longitude = 0.0
		self.min_long = 0.0
		self.max_long = 0.0
		self.min_lat = 0.0
		self.max_lat = 0.0

		self.tlx = sys.float_info.max
		self.tly = sys.float_info.max
		self.brx = -sys.float_info.max
		self.bry = -sys.float_info.max

		self.area = 0.0

		self.doppler_dir = 0.0
		self.status = 0  # 0=unconfirmed, 1=confirmed, 2=deleted
		self.classification = 0  # 0: undefined, 1: stationary, 2: moving
		self.id = -1
		self.cluster_id = -1
		self.score = 0
		self.max_score = 0

		self.heading = -1.0  # in rad 0 -> 2pi clockwise up is 0
		self.turning_rate = 0.0
		self.doppler_heading = 0.0
		self.vel = -1.0  # should only be pos
		self.accell = 0.0
		self.xpos_mem = np.zeros(10, dtype=float)
		self.ypos_mem = np.zeros(10, dtype=float)
		self.history_index = 0
		self.history_min_10 = False

		self.has_future = False
		self.xpos_future = np.zeros(5, dtype=float)  # list to keep future predicted positions
		self.ypos_future = np.zeros(5, dtype=float)
		self.tlx_future = np.zeros(5, dtype=float)
		self.brx_future = np.zeros(5, dtype=float)
		self.tly_future = np.zeros(5, dtype=float)
		self.bry_future = np.zeros(5, dtype=float)
		self.x_centre_future = np.zeros(5, dtype=float)
		self.y_centre_future = np.zeros(5, dtype=float)

		self.est_heading = 0.0
		self.est_heading_deg = 0.0
		self.est_turning_rate = 0.0  # rad/s pos clockwise not normal
		self.est_doppler_heading = 0.0
		self.est_vel = 0.0
		self.est_accell = 0.0

		self.est_xpos = 0.0  # not future position but mean current
		self.est_ypos = 0.0

		self.est_tlx = 0.0
		self.est_tly = 0.0
		self.est_brx = 0.0
		self.est_bry = 0.0

		self.time_birth = 0
		self.last_update = 0
		
		self.age = 0

	def __repr__(self):
		return f"OBJ: {self.id}: ({round(self.xpos, 1)}, {round(self.ypos, 1)}),S:{self.score}"


class TrackedObject:
	def __init__(self, futures=5, mems=10):
		self.id = -1
		self.status = 0  # 0=unconfirmed, 1=confirmed, 2=deleted
		self.classification = 0  # 0: undefined, 1: stationary, 2: moving
		self.cluster_id = -1
		self.score = 0
		self.max_score = 0
		self.numpoints = 0

		self.time_birth = 0
		self.last_update = 0
		self.age = 0
		self.updated_this_it = False

		self.xpos = 0.0
		self.ypos = 0.0
		self.vel = 0.0
		self.heading = 0.0
		self.turning_rate = 0.0
		self.doppler_heading = 0.0
		self.accell = 0.0
		

		self.latitude = 0.0
		self.longitude = 0.0
		self.min_long = 0.0
		self.max_long = 0.0
		self.min_lat = 0.0
		self.max_lat = 0.0

		self.tlx = sys.float_info.max
		self.tly = sys.float_info.max
		self.brx = -sys.float_info.max
		self.bry = -sys.float_info.max

		self.area = 0.0

		# === MEM ===
		# arrays to store the last 'mems' values for mean
		self.history_index = 0
		self.history_min_counted = False
		self.history_first = True
		#self.heading_mem = np.zeros(mems, dtype=float)  # in rad 0 -> 2pi clockwise up is 0
		# self.xpos_mem = np.zeros(mems, dtype=float)
		# self.ypos_mem = np.zeros(mems, dtype=float)
		self.head_prev = -1.0
		self.turning_rate_mem = np.zeros(mems, dtype=float)
		# self.doppler_heading_mem = np.zeros(mems, dtype=float)
		#self.vel_mem = np.zeros(mems, dtype=float)
		self.vel_prev = -1.0
		self.accell_mem = np.zeros(mems, dtype=float)
		# self.tlx_mem = np.zeros(mems, dtype=float)
		# self.tly_mem = np.zeros(mems, dtype=float)
		# self.brx_mem = np.zeros(mems, dtype=float)
		# self.bry_mem = np.zeros(mems, dtype=float)

		# === FUTURE ===
		self.has_future = False
		self.xpos_future = np.zeros(futures, dtype=float)  # list to keep future predicted positions
		self.ypos_future = np.zeros(futures, dtype=float)
		self.tlx_future = np.zeros(futures, dtype=float)
		self.brx_future = np.zeros(futures, dtype=float)
		self.tly_future = np.zeros(futures, dtype=float)
		self.bry_future = np.zeros(futures, dtype=float)
		self.x_centre_future = np.zeros(futures, dtype=float)
		self.y_centre_future = np.zeros(futures, dtype=float)	

	def __repr__(self):
		return f"OBJ {self.id}: ({round(self.xpos, 1)}, {round(self.ypos, 1)}),S:{self.score}"


class ClusterObject:
	def __init__(self):
		self.cluster_id = -1
		self.matched_id = -1
		self.x_idx = 0
		self.y_idx = 0
		self.right_heavy = False
		self.top_heavy = False
		self.left_heavy = False
		self.bottom_heavy = False
		self.box_centre_x = 0.0
		self.box_centre_y = 0.0
		self.point_count = 0
		self.max_x = 0.0
		self.min_x = 0.0
		self.max_y = 0.0
		self.min_y = 0.0
		self.mass_center_x = 0.0
		self.mass_center_y = 0.0
		self.doppler_dir = 0.0
		self.check = True


class MergedClusterObject:
	def __init__(self):
		self.id = 0
		self.mass_center_x = 0.0
		self.mass_center_y = 0.0
		self.max_x = 0.0
		self.min_x = 0.0
		self.min_y = 0.0
		self.max_y = 0.0
		self.box_centre_x = 0.0
		self.box_centre_y = 0.0
		self.doppler_dir = 0.0  # deg true north
		self.point_count = 0
		self.box_count = 0
		self.remove = False



class HDTracker:
	def __init__(self, logger):
		self.logger = logger
		self.all_tracked_objects = []
		self.deleted_ids = PriorityQueue() # change to PQ 
		self.merged_clusters = []
		self.max_object_id = 0
		self.its_to_confirm = 1  # 3 iterations or seconds 
		# self.its_to_delete = 4  # 4 iterations or seconds 
		self.its_to_delete = 2  # 5 iterations or seconds 
		self.futures = 5
		self.mems = 10
		self.mems_f = 10.0


	def df_to_obj(self, clusters, timestamp):
		self.merged_clusters.clear() # TODO make sure this is deleted
		
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

			obj = TrackedObject(self.futures, self.mems)
			obj.tlx = mc.min_x
			obj.tly = mc.max_y
			obj.brx = mc.max_x
			obj.bry = mc.min_y
			obj.cluster_id = mc.id

			obj.numpoints = mc.point_count
			obj.score = 0

			if obj.numpoints == 0:
				continue

			obj.time_birth = curr_time

			obj.xpos = mc.mass_center_x
			obj.ypos = mc.mass_center_y

			obj.area = abs((obj.brx - obj.tlx) * (obj.bry - obj.tly))
			if obj.area < 6.0:
				continue

			obj.doppler_heading = mc.doppler_dir

			# Increment objectIds and assign it to obj.id
			# TODO: add possibility to reuse deleted ids
			if not self.deleted_ids.empty():
				obj.id = self.deleted_ids.get()
				self.logger.debug(f"Reusing ID: {obj.id}")
				# print(f"Reusing ID: {obj.id}")
			else:
				obj.id = self.max_object_id
				self.max_object_id += 1
			# # TODO: temporary removal of reuse
			# obj.id = self.max_object_id
			self.max_object_id += 1

			num_objs += 1
			objects.append(obj)

	def cost_function(self, obj1, obj2, max_dist=16.0**2, max_area_diff=0.75):  # lower is better 16**2 = 256
		# TODO: maybe weight area or dist
		# 16 meters is eq 16 mps or 30 knots or 55 kph
		if obj2.has_future:
			# match with future position
			center_distance = ((obj1.xpos - obj2.xpos_future[0])**2 + (obj1.ypos - obj2.ypos_future[0])**2)
		else:
			# if no future match with previuse pos
			center_distance = ((obj1.xpos - obj2.xpos)**2 + (obj1.ypos - obj2.ypos)**2)
		
		area_diff = np.abs(obj1.area - obj2.area)/((obj1.area+obj2.area)/2.0)

		normalized_distance = center_distance / max_dist

		normalized_area_diff = area_diff / max_area_diff
		
		dynamic_weight = 1 / (1 + normalized_distance**0.5)

		# Apply the dynamic weight to the area difference
		weighted_area_diff = normalized_area_diff * dynamic_weight

		if weighted_area_diff > 1.0 or normalized_distance > 1.0:
			return float("inf")
		
		return weighted_area_diff + normalized_distance

	def cost_function_old(self, obj1, obj2, max_dist=16.0**2, max_area_diff=0.60):  # lower is better 16**2 = 256
		# TODO: maybe weight area or dist
		# 16 meters is eq 16 mps or 30 knots or 55 kph
		center_distance = ((obj1.xpos - obj2.xpos)**2 + (obj1.ypos - obj2.ypos)**2)
		normalized_distance = center_distance / max_dist

		area_diff = np.abs(obj1.area - obj2.area)/((obj1.area+obj2.area)/2.0)
		normalized_area_diff = area_diff / max_area_diff

		if normalized_area_diff > 1.0 or normalized_distance > 1.0:
			return float("inf")
		
		return normalized_area_diff + normalized_distance
	
	def remove_multiple_tracked_objects(self, indexes_to_remove):
		# Sort indexes in descending order
		indexes = sorted(indexes_to_remove, reverse=True)

		for index in indexes:
			# Check if index is within bounds
			if 0 <= index < len(self.all_tracked_objects):
				del self.all_tracked_objects[index]
			else:
				raise ValueError(f"Index {index} is out of bounds, skipping.")
		return True
	
	def store_attribute_history(self, attribute, hist_ind, value):
		attribute[hist_ind] = value

	def get_attribute_mean(self, obj: TrackedObject, attribute):
		# ignore first!!!
		att_sum = 0.0
		att_mean = 0.0
		mems_f = float(self.mems)
		if not obj.history_min_counted:
			its: int = obj.history_index+1
			for i in range(its):
				att_sum += attribute[i] # TODO: fix heading avg not same as vel

			att_mean = att_sum / float(its)
			
			if obj.history_index == (self.mems-1) and (obj.history_index + 1) % self.mems == 0:
				obj.history_min_counted = True
		else:
			for i in range(self.mems):
				att_sum += attribute[i]
				
			# maybe check lat and long vel check
			# TODO: change so that if less than 10 obs take those
			att_mean = att_sum / mems_f
		
		return att_mean

		
	def match_objects(self, new_objects: List[TrackedObject], curr_time):
		# New method
		all_new_object_indexes = []
		general_dt = 1.0
		match_dict = {} # key is all track index, and value is list of new_obj indexes
		for ni, new_obj in enumerate(new_objects):
			all_new_object_indexes.append(ni)
			lowest_cost = float("inf")
			lowest_cost_index = None
			for oi, obj in enumerate(self.all_tracked_objects):
				dt = (new_obj.time_birth - obj.last_update)/1.0e9 # in s
				if dt < 0.8: # then not new object
					continue
				if not general_dt:
					general_dt = dt
				# TODO: maybe do both est pos and actual
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
		for to_all_idx, val in match_dict.items():
			sorted_nids = sorted(val, key=lambda x: x[0])
			lowest_cost_no_index = sorted_nids[0][1]
			matches_map.append((to_all_idx,lowest_cost_no_index))
			all_new_object_indexes.remove(lowest_cost_no_index)

		not_matched_indexes = all_new_object_indexes # to all tracked wo match

		# ========================
		# Handle matched objects
		for old_idx, new_idx in matches_map:
			new_obj = new_objects[new_idx]
			obj = self.all_tracked_objects[old_idx]
			obj.last_update = new_obj.time_birth
			obj.age = (obj.last_update - obj.time_birth)/1.0e9
			obj.last_update = curr_time
			obj.updated_this_it = True

			dx: float = new_obj.xpos - obj.xpos
			dy: float = new_obj.ypos - obj.ypos
			dist: float = np.sqrt(dx**2 + dy**2)

			# Always use actual heading and velocity
			# but calculate acell and turning rate from that
			self.logger.debug(f">> {obj.id} got CI: {new_obj.cluster_id}, DIST: {round(dist, 3)} m")
			vel: float = dist / dt  # m/s TODO: must verify this becomes float seconds not something else

			# TODO: compensate for true heading maybe
			heading_rad_raw: float = np.arctan2(dy, dx)
			heading_rad: float = -heading_rad_raw + PI/2.0  # TODO: check this, Adjust so that 0 degrees is straight up

			if heading_rad < 0.0:
				heading_rad = PI*2-heading_rad

			heading_rad = np.fmod(heading_rad, PI*2)

			# === end actual values ===

			# turning_rate = 0.0
			# if obj.heading >= 0.0:
			# 	heading_diff = heading_rad - obj.heading
			# 	heading_diff = (heading_diff + PI) % (2 * PI) - PI

			# 	turning_rate: float = heading_diff/dt
			
			# accell = 0.0
			# if obj.vel >= 0.0:
			# 	vel_diff = vel - obj.vel
			# 	accell = vel_diff / dt

			

			# TODO: Please take the history back!!
			
			# ======= store in history / mem
			mean_tr = 0.0
			mean_acell = 0.0
			if obj.history_first:
				obj.head_prev = heading_rad
				obj.vel_prev = vel
				obj.history_first = False
			else:
				# HEading tr
				heading_diff = heading_rad - obj.head_prev
				heading_diff = (heading_diff + PI) % (2 * PI) - PI
				turning_rate = heading_diff / dt
				self.store_attribute_history(obj.turning_rate_mem, obj.history_index, turning_rate)
				obj.head_prev = heading_rad
				mean_tr = self.get_attribute_mean(obj, obj.turning_rate_mem)

				# Vel acell
				vel_diff = vel - obj.vel_prev
				accell = vel_diff / dt
				self.store_attribute_history(obj.accell_mem, obj.history_index, accell)
				obj.vel_prev = vel
				mean_acell = self.get_attribute_mean(obj, obj.accell_mem)

				obj.history_index = (obj.history_index + 1) % self.mems
			
			obj.vel = vel
			obj.accell = mean_acell
			obj.heading = heading_rad
			obj.turning_rate = mean_tr

			#print("=====================================")
			# TODO: check this
			obj.xpos = new_obj.xpos
			obj.ypos = new_obj.ypos
			# obj.est_xpos = obj.xpos + (vel * np.cos(heading_rad) * dt)
			# obj.est_ypos = obj.ypos + (vel * np.sin(heading_rad) * dt)
			# TODO: do same for est_xpos as vel, and add future positions
			# obj.est_xpos = new_obj.xpos + dx
			# obj.est_ypos = new_obj.ypos + dy
			#obj.est_heading_deg = obj.est_heading * (180/PI)

			obj.area = new_obj.area 
			obj.num_points = new_obj.numpoints

			if obj.area < 100.0: # sure not mountain
				obj.has_future = True

			obj.tlx = new_obj.tlx
			obj.tly = new_obj.tly
			obj.brx = new_obj.brx
			obj.bry = new_obj.bry
			obj.x_centre = obj.tlx + (obj.brx-obj.tlx)/2.0
			obj.y_centre = obj.bry + (obj.tly-obj.bry)/2.0


			# update future positions
			# TODO: add bbox future
			# TODO: add next_vel also currently assuming constant speed
			# HARD LIMITS
			if obj.turning_rate > PI/4:
					obj.turning_rate = PI/4
			if obj.turning_rate < -PI/4:
				obj.turning_rate = -PI/4
			if obj.accell > 5.0: # abt 10 m/s^2 o-40kntos in 4.5 secs
				obj.accell = 5.0
			if obj.accell < -5.0:
				obj.accell = -5.0
			if vel > 21.0: # abt 40 knots
				vel = 21.0
			if vel < 0.0:
				vel = 0.0

			self.logger.debug(f"Vel: {round(vel,2)}, head: {round(obj.heading*180/PI, 2)}, TR: {round(obj.turning_rate*180/PI,4)}, acc: {round(obj.accell, 4)}")
			

			if obj.has_future:
				for i in range(5):
					next_heading = heading_rad + obj.turning_rate * dt * (i + 1)
					next_heading = np.fmod(next_heading, PI*2) # TO limit to 0 to 2pi
					next_vel = vel + obj.accell * dt * (i + 1)
					if next_vel < 0.0:
						next_vel = 0.0
					# next_heading = heading_rad # constant
					# next_vel = vel # constant

					cdx = (next_vel * np.cos(-next_heading+PI/2.0)) * dt * (i + 1)
					cdy = (next_vel * np.sin(-next_heading+PI/2.0)) * dt * (i + 1)
					obj.xpos_future[i] = obj.xpos + cdx
					obj.ypos_future[i] = obj.ypos + cdy

					obj.tlx_future[i] = obj.tlx + cdx
					obj.tly_future[i] = obj.tly + cdy
					obj.brx_future[i] = obj.brx + cdx
					obj.bry_future[i] = obj.bry + cdy

					obj.x_centre_future[i] = obj.tlx_future[i] + (obj.brx_future[i]-obj.tlx_future[i])/2.0
					obj.y_centre_future[i] = obj.bry_future[i] + (obj.tly_future[i]-obj.bry_future[i])/2.0

			
			
			obj.score += 15  # TODO: set as parameter in ros

			# Update status for those with score above threshold
			# TODO: add velocity history
			obj.max_score = max(obj.score, obj.max_score-5)




		# ========================
		# Handle not matched
		for not_matched_idx in not_matched_indexes:
			new_obj = new_objects[not_matched_idx]
			self.logger.debug(f"NEW TO: {new_obj.id}, with Cluster ID: {new_obj.cluster_id}")
			new_obj.last_update = new_obj.time_birth
			new_obj.age = (new_obj.last_update - new_obj.time_birth)/1.0e9
			new_obj.score += 10
			self.all_tracked_objects.append(copy(new_obj))

		to_delete = []

		for obj in self.all_tracked_objects:
			obj.score -= 5
			
			if not obj.updated_this_it:
				obj.age += general_dt
				obj.last_update = curr_time

			if obj.updated_this_it:
				obj.updated_this_it = False


			# TODO: question this
			if (obj.max_score - obj.score) > (5 + 5*self.its_to_delete) or (obj.score < 0):
				obj.status = 2  # deleted
				to_delete.append(obj.id) # add id instead of index
			elif obj.score > (5*self.its_to_confirm):  # TODO: set as parameter in ros
				# maybe change
				obj.status = 1  # confirmed
				# TODO:  maybe printout

			if obj.status == 1:
				# TODO: check that heading is not jumping
				min_vel = 0.51444*5 	# 5 knot
				#min_vel = 4 			# 4 mps
				#min_vel = 0.277778*10	# 10 kph
				if (obj.vel > min_vel) and obj.has_future and obj.area < 100.0: # 1 knot TODO: set as parameter and sync with doppler
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
					self.logger.debug(f"Deleting TO: {delete_id}")
					indexes_to_del.append(index) # TODO: this wont work bc changing index

		#Deletion
		self.remove_multiple_tracked_objects(indexes_to_del)
		# END OF MATCH OBJECTS
		# ========================

	

	# def estimate_object_position(self, lat, lon, td):
	def estimate_object_position(self):
		"""
			here we should update age 
			and change all attributes based on egomtion/gps
		"""


		# x_diff: float = 0.0
		# y_diff: float = 0.0
		# last_lat = 0.0  # TODO: update!
		# last_lon = 0.0
		# heading = 0.0 # TODO: update

		# x_diff, y_diff = latLongToMeters(last_lat, last_lon, lat, lon, heading)
		# # TODO: this is not working probably

		# tcos = 0.0
		# tsin = 0.0

		# for to in self.all_tracked_objects:
		# 	if to.est_vel > 0:
		# 		tcos = to.est_vel * np.cos(to.est_heading * PI / 180.0) * td
		# 		tcos = to.est_vel * np.cos(to.est_heading) * td
		# 		tsin = to.est_vel * np.sin(to.est_heading) * td

		# 	to.est_xpos = to.xpos + tcos
		# 	to.est_ypos = to.ypos + tsin
		# 	to.est_tlx = to.tlx + tcos
		# 	to.est_tly = to.tly + tsin
		# 	to.est_brx = to.brx + tcos
		# 	to.est_bry = to.bry + tsin

		# 	# update with egomotion
		# 	to.est_xpos -= x_diff
		# 	# to.est_xpos += x_diff
		# 	to.est_ypos -= y_diff
		# for to in self.all_tracked_objects:
		# 	if to.status == 1:
		# 		to.xpos = to.xpos_future[0]
		# 		to.ypos = to.ypos_future[0]
		# 		to.tlx = to.tlx_future[0]
		# 		to.tly = to.tly_future[0]
		# 		to.brx = to.brx_future[0]
		# 		to.bry = to.bry_future[0]
		# 		to.x_centre = to.tlx + (to.brx-to.tlx)/2.0
		# 		to.y_centre = to.bry + (to.tly-to.bry)/2.0


	def iteration(self):
		# 1. Get detections
		# 2. Get objects
		# 3. Match objects
		pass