from hdbscan import HDBSCAN
from sklearn.cluster import DBSCAN
import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.highlevel import AnyReader
from pathlib import Path
import tqdm
import os


RADARSPOKE_MSG = """
uint32 detection_range
uint32 detections
float32[] x_indices
float32[] y_indices
float32[] ranges
uint8[] intensities
bool doppler_active
uint8[] doppler_returns
	"""

TOPIC = "/radar_driver/RadarScanCartesian"
MSG_TYPE = "radar_msgs/RadarScanCartesian"
register_types(get_types_from_msg(RADARSPOKE_MSG, MSG_TYPE))

out_folder = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/out"

LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/5-2/rosbag2_2024_02_20-13_11_18"
# LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/5-2/rosbag2_2024_02_16-14_54_56"

log_folder = os.path.basename(LOGPATH)
new_dir = os.path.join(out_folder, log_folder)
os.makedirs(new_dir, exist_ok=True)
out_folder = new_dir

N = 10000
ti = 0

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

#dbscan = HDBSCAN(min_cluster_size=40)#30
eps = 0.9 # approx 1 m at 50 m
min_samples = 16 # need to scale these with distance
dbscan = DBSCAN(eps=eps, min_samples=min_samples)
with AnyReader([Path(LOGPATH)]) as reader:
	print_meta(reader)
	connections = [x for x in reader.connections if x.topic == TOPIC]
	for connection, timestamp, rawdata in tqdm.tqdm(reader.messages(connections=connections)):
		msg = reader.deserialize(rawdata, connection.msgtype)
		rng = msg.detection_range

		if not (30<ti<42):
			ti += 1
			continue
		swe_tz = 3600000000000
		t_str = pd.to_datetime(timestamp+swe_tz, unit='ns').strftime('%H:%M:%S')
		data = {"time": "", "x": [], "y": []}
		for x, y, intens in zip(msg.x_indices, msg.y_indices, msg.intensities):
			data["time"] = t_str
			data["x"].append(x)
			data["y"].append(y)
			
		df = pd.DataFrame(data)
		ti += 1
		#df["dist"] = np.sqrt(df["x"]**2 + df["y"]**2)
		#print("step:", ti)
		# for each timestep
		sdf = df.sample(n=N)

		# Perform HDBSCAN clustering
		clusters = dbscan.fit_predict(sdf[['x', 'y']])

		# Create a larger figure
		fig, ax = plt.subplots(figsize=(10, 8))
		ax.set_xlim(-rng, rng)
		ax.set_ylim(-rng, rng)

		# Plot the clusters
		scatter = ax.scatter(sdf['x'], sdf['y'], c=clusters, cmap='turbo', s=0.5)
		ax.set_xlabel('x')
		ax.set_ylabel('y')
		ax.set_title(f'DBSCAN t: {t_str}')

		# Create legend labels and handles
		unique_labels = np.unique(clusters)
		legend_labels = ['{}'.format(label) for label in unique_labels]

		# Create legend handles
		legend_handles = [plt.Line2D([0], [0], marker='o', color=scatter.cmap(scatter.norm(label)), markersize=10) for label in unique_labels]

		# Add legend
		ax.legend(legend_handles, legend_labels, title='Labels', loc='upper right')

		# Save the plot in the specified folder
		file_path = os.path.join(out_folder, f't_{ti}.png')
		plt.savefig(file_path)
		plt.close()