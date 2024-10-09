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
		self.tracker = HDTracker()
		if out_dir and not os.path.exists(out_dir):
			os.makedirs(out_dir)
		self.out_dir = out_dir

	def plot(self, points, t="0"):
		fix, axs = plt.subplots(1, self.subplots, figsize=(20, 8))
		# Convert ax to a list if it's not already one
		if not isinstance(axs, np.ndarray):
			axs = [axs]

		x_ticks = np.linspace(-self.r, self.r, self.gridsize + 1)
		y_ticks = np.linspace(-self.r, self.r, self.gridsize + 1)

		for i, algo in enumerate(self.algos):
			axs[i].set_xticks(np.round(x_ticks, decimals=1))
			axs[i].set_xticklabels(np.round(x_ticks, decimals=1), rotation=45, ha='right')  # Set x tick labels at 45-degree downward angle
			axs[i].set_yticks(np.round(y_ticks, decimals=1))
			axs[i].grid(True, linestyle='--', linewidth=0.5, color='gray')
			axs[i].set_title(algo.name)
			axs[i].set_xlabel('X axis')
			axs[i].set_ylabel('Y axis') 	
			axs[i].set_xlim(-self.r, self.r)
			axs[i].set_ylim(-self.r, self.r)

		# run clustering
		for i, algo in enumerate(self.algos):
			# Perform clustering
			# print(f'Running {algo.name}')
			start = time.time()
			clusters = algo.fit_predict(points)
			end = time.time()
			time_diff = str(round(end - start, 2))
			# print(f'{algo.shortname}: {round(end - start, 2)} s')

			axs[i].scatter(points['x'], points['y'], s=0.5, color='black')

			axs[i].text(1, 1, f"{time_diff} s, {len(clusters)} labels", fontsize=9, transform=axs[i].transAxes, 
        				verticalalignment='top', horizontalalignment='right', color='black', ha='center',
					    va='center', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
			# Add the cluster labels to the dataframe
			for _, row in clusters.iterrows():
				axs[i].text(row['mass_center_x'], row['mass_center_y'], int(row['id']), fontsize=7, color='black', ha='center',
							va='center', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.1'))
				# Draw bounding box
				rect = patches.Rectangle((row['min_x'], row['min_y']), row['max_x'] - row['min_x'],
										 row['max_y'] - row['min_y'], linewidth=1, edgecolor='r', facecolor='none')
				axs[i].add_patch(rect)

		plt.tight_layout()

		if not self.out_dir:
			plt.show()
		else:
			file_path = os.path.join(self.out_dir, f'{t}.png')
			plt.savefig(file_path)
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
	eps = 2
	min_samples = 40
	N = 30000

	# df = df[(df['x'] >= -9.3) & (df['x'] < 1.9) & (df['y'] >= 20.4) & (df['y'] < 35.2)]  #
	# df = df.sample(n=1000)
	#df.reset_index(drop=True, inplace=True)
	# print(df)
	basename = os.path.basename(logpath)
	out_fold = '/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/compare/out'
	out_dir = os.path.join(out_fold, basename)
	print("storing logs in:", out_dir)
	ts = load_msg_defs()
	algos = [GridScanDF(gridsize, r, c, d, min_points), DBSCANDF(eps=eps, min_samples=min_samples)]
	# algos = [GridScanDF(gridsize, r, c, d, min_points)]
	plotter = Plotter(r, gridsize, algos=algos, out_dir=out_dir)
	t = 0
	prev_r = 50
	print("running reader")
	with AnyReader([Path(logpath)], default_typestore=ts) as reader:
		print_meta(reader)
		connections = [x for x in reader.connections if x.topic == TOPIC]
		for connection, timestamp, rawdata in tqdm.tqdm(reader.messages(connections=connections)):
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
			plotter.plot(sdf, t=t_string)
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
	# df = df.sample(n=1000)
	df.reset_index(drop=True, inplace=True)
	# print(df)

	eps = 2
	min_samples = 40
	algos = [GridScanDF(gridsize, r, c, d, min_points), DBSCANDF(eps=eps, min_samples=min_samples)]
	# algos = [GridScanDF(gridsize, r, c, d, min_points)]
	plotter = Plotter(r, gridsize, algos)

	plotter.plot(df)

if __name__ == '__main__':
	single('/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/rosbag2_2024_02_16-14_54_56.csv')
	# LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/5-2/rosbag2_2024_02_16-14_54_56"
	# LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/march/rosbag2_2024_03_05-13_55_47"
	# LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/may/rosbag2_2024_05_24-13_01_02"
	# iterate_times(LOGPATH)