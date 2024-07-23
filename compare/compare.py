import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.patches as patches
from clusterAlgos import DBSCANDF, GridScanDF
import pandas as pd
import os
import tqdm
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from rosbags.highlevel import AnyReader
from pathlib import Path
from tracker import HDTracker
np.random.seed(0)


def guess_msgtype(path: Path) -> str:
	"""Guess message type name from path."""
	name = path.relative_to(path.parents[2]).with_suffix('')
	name = name.relative_to(name.parts[0])
	if 'msg' not in name.parts:
		name = name.parent / 'msg' / name.name
	return str(name)

TOPIC = "/marv/sys/nav/RadarScanCartesian"
# MSG_TYPE = "radar_msgs/RadarScanCartesian"
# register_types(get_types_from_msg(RADARSPOKE_MSG, MSG_TYPE))

def load_msg_defs():
	msgs_dir = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/compare_2/msg_definitions"
	msg_dirs = os.listdir(msgs_dir)
	msg_defs = []
	for d in msg_dirs:
		sub_msgs = os.listdir(os.path.join(msgs_dir, d))
		for m in sub_msgs:
			if m.endswith(".msg"):
				msg_defs.append(Path(os.path.join(msgs_dir, d, m)))
	


	typestore = get_typestore(Stores.ROS2_FOXY)
	add_types = {}

	for pathstr in msg_defs:
		msgpath = Path(pathstr)
		msgdef = msgpath.read_text(encoding='utf-8')
		add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

	typestore.register(add_types)
	return typestore

def print_meta(reader):
	swe_tz = 3600000000000
	dur = str(round(reader.duration*10**(-9), 2))
	st = pd.to_datetime(reader.start_time+swe_tz, unit='ns').strftime('%H:%M:%S')
	et = pd.to_datetime(reader.end_time+swe_tz, unit='ns').strftime('%H:%M:%S')
	mc = str(reader.message_count)
	topics = sorted([f"{('('+str(v.msgcount)+')').rjust(8)} {k}" for k,v in reader.topics.items()])
	w = max([len(dur), len(et), len(mc), len(st)]+ [len(t)+3 for t in topics])
	print("╔"+"═"*(w+2)+"╗")
	print("║"+" "*((w-6)//2)+"LOG DATA"+" "*((w-5)//2)+"║")
	print("╠"+"═"*(w+2)+"╣")
	print("║"+"Duration: "+dur + " s"+" "*(w-len(dur)-10)+"║")
	print("║"+"Start time: "+st+" "*(w-len(st)-10)+"║")
	print("║"+"End time: "+et+" "*(w-len(et)-8)+"║")
	print("║"+"Message count: "+mc+" "*(w-len(mc)-13)+"║")
	print("║"+"Topics: "+" "*(w-6)+"║")
	for t in topics:
		print("║"+"   "+t+" "*(w-len(t)-1)+"║")
	print("╚"+"═"*(w+2)+"╝")



class Plotter:
	def __init__(self, r, gridsize, algos=[], out_dir=None):
		self.r = r
		self.gridsize = gridsize
		self.subplots = len(algos)
		self.algos = algos
		self.trackers = [HDTracker(), HDTracker()]
		if out_dir and not os.path.exists(out_dir):
			os.makedirs(out_dir)
		self.out_dir = out_dir

	def plot(self, points, t="0", timestamp=0):
		run_tracking = True
		fix, axs = plt.subplots(self.subplots, 2, figsize=(34, 16))
		# Convert ax to a list if it's not already one
		if self.subplots == 1:
			axs = [[axs[0], axs[1]]]

		x_ticks = np.linspace(-self.r, self.r, self.gridsize + 1)
		y_ticks = np.linspace(-self.r, self.r, self.gridsize + 1)

		for i, algo in enumerate(self.algos):
			# scans
			axs[i][0].set_xticks(np.round(x_ticks, decimals=1))
			axs[i][0].set_xticklabels(np.round(x_ticks, decimals=1), rotation=45, ha='right')  # Set x tick labels at 45-degree downward angle
			axs[i][0].set_yticks(np.round(y_ticks, decimals=1))
			axs[i][0].grid(True, linestyle='--', linewidth=0.5, color='gray')
			axs[i][0].set_title(algo.name)
			axs[i][0].set_xlabel('X axis')
			axs[i][0].set_ylabel('Y axis') 	
			axs[i][0].set_xlim(-self.r, self.r)
			axs[i][0].set_ylim(-self.r, self.r)

			# tracking
			axs[i][1].set_xticks(np.round(x_ticks, decimals=1))
			axs[i][1].set_xticklabels(np.round(x_ticks, decimals=1), rotation=45, ha='right')  # Set x tick labels at 45-degree downward angle
			axs[i][1].set_yticks(np.round(y_ticks, decimals=1))
			axs[i][1].set_xlabel('X axis')
			axs[i][1].set_ylabel('Y axis')
			axs[i][1].set_xlim(-self.r, self.r)
			axs[i][1].set_ylim(-self.r, self.r)

		# run clustering
		for i, algo in enumerate(self.algos):
			# Perform clustering
			# print(f'Running {algo.name}')
			start = time.time()
			clusters = algo.fit_predict(points)
			end = time.time()
			time_diff = str(round(end - start, 2))
			# print(f'{algo.shortname}: {round(end - start, 2)} s')

			axs[i][0].scatter(points['x'], points['y'], s=0.5, color='black')
			axs[i][1].scatter(points['x'], points['y'], s=0.5, color='black')

			axs[i][0].text(1, 1, f"{time_diff} s, {len(clusters)} labels", fontsize=9, transform=axs[i][0].transAxes, 
        				verticalalignment='top', horizontalalignment='right', color='black', ha='center',
					    va='center', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
			# Add the cluster labels to the dataframe
			for _, row in clusters.iterrows():
				axs[i][0].text(row['mass_center_x'], row['mass_center_y'], int(row['id']), fontsize=7, color='black', ha='center',
							va='center', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.1'))
				# Draw bounding box
				rect = patches.Rectangle((row['min_x'], row['min_y']), row['max_x'] - row['min_x'],
										 row['max_y'] - row['min_y'], linewidth=1, edgecolor='r', facecolor='none')
				axs[i][0].add_patch(rect)


			# run tracking
			if run_tracking:
				# convert df to class object
				new_objects = []
				# clustring done, now convert to obj
				self.trackers[i].df_to_obj(clusters, timestamp)

				# create tracked objects
				self.trackers[i].create_tracked_object(new_objects, timestamp)

				# upadate object positions
				# here

				# then match objects
				self.trackers[i].match_objects(new_objects)

				# Plot tracked objects
				for obj in self.trackers[i].all_tracked_objects:
					# TODO: print heading arrow and fufutre track
					# if obj.status == 1:
					if True:
						# axs[i][1].scatter(obj.xpos, obj.ypos, color='red', linewidth=1)
						# axs[i][1].text(obj.xpos, obj.ypos, obj.id, fontsize=7, color='black', ha='center',
						# 		va='center', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.1'))


						axs[i][1].text(obj.xpos, obj.ypos, str(round(obj.age)), fontsize=7, color='black', ha='center',
								va='center', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.1'))
						
						edge_color = ["red", "blue", "green"][obj.classification] # red=unclass, blue=stat, green=moving, 

						rect = patches.Rectangle((obj.tlx, obj.bry), obj.brx - obj.tlx,
											obj.tly - obj.bry, linewidth=1, edgecolor=edge_color, facecolor='none')
						axs[i][1].add_patch(rect)

						dx = obj.est_vel * np.cos(-obj.est_heading+np.pi/2.0) * 20
						dy =  obj.est_vel * np.sin(-obj.est_heading+np.pi/2.0) * 20

						# Plotting the arrow
						axs[i][1].arrow(obj.xpos, obj.ypos, dx, dy, head_width=1, head_length=1, fc='black', ec='black')


						#axs[i][1].plot(obj.xpos_future, obj.ypos_future, color="cyan")

						for bi in range(5):
							rect = patches.Rectangle((obj.tlx_future[bi], obj.bry_future[bi]), obj.brx_future[bi] - obj.tlx_future[bi],
											obj.tly_future[bi] - obj.bry_future[bi], linewidth=1, edgecolor="grey", facecolor='none')
							axs[i][1].add_patch(rect)


		plt.tight_layout()

		if not self.out_dir:
			plt.show()
		else:
			file_path = os.path.join(self.out_dir, f'{t}.png')
			plt.savefig(file_path)
			print(f"Saved: {t}.png")
			plt.close()


def iterate_times(logpath):
	# grid
	r = 50
	# gridsize = 27  # Grid size (number of divisions along x and y axes)
	gridsize = 69   # Grid size (number of divisions along x and y axes)
	c = 0.1  # center box size ratio for setting heaviness
	min_points = 40  # min points per cluster, change based on range
	d = 0.4  # meters, is max separation between clusters, i think this should be range independent
	
	# db
	# eps = 2
	eps = 0.2 # radius im meters for neighbouring points
	min_samples = 40
	N = 15000 # used to be 30000

	# df = df[(df['x'] >= -9.3) & (df['x'] < 1.9) & (df['y'] >= 20.4) & (df['y'] < 35.2)]  #
	# df = df.sample(n=1000)
	#df.reset_index(drop=True, inplace=True)
	# print(df)
	basename = os.path.basename(logpath)
	out_fold = '/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/compare/out'
	out_dir = os.path.join(out_fold, basename)
	print("storing logs in:", out_dir)
	ts = load_msg_defs()
	# algos = [GridScanDF(gridsize, r, c, d, min_points), DBSCANDF(eps=eps, min_samples=min_samples)]
	algos = [DBSCANDF(eps=eps, min_samples=min_samples)]
	# algos = [GridScanDF(gridsize, r, c, d, min_points)]
	plotter = Plotter(r, gridsize, algos=algos, out_dir=out_dir)
	t = 0
	prev_r = 50
	frame_start = 68
	frame_end = 90
	frame_counter = 0
	print("running reader")
	# 10_14_25 to 10_14_42
	# TODO: future track from heading and vel for those with positive vel

	with AnyReader([Path(logpath)], default_typestore=ts) as reader:
		#print_meta(reader)
		connections = [x for x in reader.connections if x.topic == TOPIC]
		# for connection, timestamp, rawdata in tqdm.tqdm(reader.messages(connections=connections)):
		for connection, timestamp, rawdata in reader.messages(connections=connections):
			if frame_counter < frame_start:
				frame_counter += 1
				continue
			if frame_counter > frame_end:
				break
			frame_counter += 1
			msg = reader.deserialize(rawdata, connection.msgtype)
			rng = msg.detection_range
			
			

			if rng != prev_r:
				print("range changed to:", rng)
				r = rng
				plotter.r = r
				algos[0] = GridScanDF(gridsize, r, c, d, min_points)
				prev_r = rng

			df = pd.DataFrame({"time": t, "x": msg.x_indices, "y": msg.y_indices})

			t_string = (pd.to_datetime(timestamp, unit='ns') + pd.Timedelta(hours=1)).strftime('%H_%M_%S')
			# print(t_string)
			print("points in df:", len(df))
			if len(df) > N:
				sdf = df.sample(n=N)
			else:
				sdf = df
			plotter.plot(sdf, t=t_string, timestamp=timestamp)
			t += 1


def single(logpath):
	r = 50
	# gridsize = 27  # Grid size (number of divisions along x and y axes)
	gridsize = 69  # Grid size (number of divisions along x and y axes)
	c = 0.1  # center box size ratio for setting heaviness
	min_points = 40  # min points per cluster, change based on range
	d = 0.4  # meters, is max separation between clusters, i think this should be range independent
	print("D:", d, "m")
	df = pd.read_csv(logpath)
	df = df[(df['time'] >= 34) & (df['time'] < 35)]  #
	print("Number of points:", len(df))
	# df = df[(df['x'] >= -9.3) & (df['x'] < 1.9) & (df['y'] >= 20.4) & (df['y'] < 35.2)]  #
	df = df.sample(n=30000)
	df.reset_index(drop=True, inplace=True)
	# print(df)

	eps = 2
	min_samples = 40
	algos = [GridScanDF(gridsize, r, c, d, min_points), DBSCANDF(eps=eps, min_samples=min_samples)]
	# algos = [GridScanDF(gridsize, r, c, d, min_points)]
	plotter = Plotter(r, gridsize, algos)

	plotter.plot(df)

if __name__ == '__main__':
	# single('/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/rosbag2_2024_02_16-14_54_56.csv')
	# LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/5-2/rosbag2_2024_02_16-14_54_56"
	# LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/march/rosbag2_2024_03_05-13_55_47"


	LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/may/rosbag2_2024_05_24-13_01_02"
	iterate_times(LOGPATH)