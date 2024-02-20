from hdbscan import HDBSCAN
import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.highlevel import AnyReader
from pathlib import Path
import tqdm



RADARSPOKE_MSG = """
uint32[32] time
float32[32] bearing
uint16[32] spoke_index
uint16[32] range
float32[32] latitude
float32[32] longitude
uint8[16384] spoke
uint8[32] doppler_mode
uint8[16384] doppler_spoke
"""
TOPIC = "/marv/nav/radar/spoke"
MSG_TYPE = "marv_msgs/msg/RadarSpoke"
register_types(get_types_from_msg(RADARSPOKE_MSG, MSG_TYPE))

out_folder = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/cv2_images/old_log"


#df = pd.DataFrame(columns=["time", 'x', 'y'])

LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/logs_ockero_08_05_23/rosbag2_2023_05_08-15_50_22/"
N = 10000
ti = 0
dbscan = HDBSCAN(min_cluster_size=20)#30
prev_rng = None
rot_c = 0
frame_i = 0
detections = 0


ri = 750 * np.array(range(0, 512))
sdf = pd.DataFrame(columns=["time", 'x', 'y'])
with AnyReader([Path(LOGPATH)]) as reader:
	connections = [x for x in reader.connections if x.topic == TOPIC]
	for connection, timestamp, rawdata in tqdm.tqdm(reader.messages(connections=connections)):
		msg = reader.deserialize(rawdata, connection.msgtype)
		
		#df = df.drop(index=df.index)
		# if not (32<ti//64<34):
		# 	ti += 1
		# 	continue




		# ============ OLD ============

		rng = int(msg.range[0])

		rot_c += 1
		if not prev_rng or prev_rng != rng: 
			#print(rng)
			prev_rng = rng
			# ax.set_xlim(-rng,rng)
			# ax.set_ylim(-rng,rng)
			ri = rng * np.array(range(1, 513))/512
		
		detections += np.count_nonzero(msg.spoke)
		
		#detections = len(msg.x_indices)
		angles = np.array(msg.bearing)
		ang_rad = np.deg2rad(angles)	

		t = -(ang_rad - np.pi/2)

		# Reshape the array into a matrix with 32 rows and 512 columns
		mat = msg.spoke.reshape(32, 512)
		above_value_indices = np.where(mat > 0)  # first is which spoke, second is which index
		
		si = above_value_indices[0]
		ii = above_value_indices[1]

		xs = np.cos(t[si]) * ri[ii]
		ys = np.sin(t[si]) * ri[ii]
		#iss = mat[(si, ii)]


		# ============ OLD ============
		
		
		data = {"x": xs, "y": ys}
		# for x, y, intens in zip(msg.x_indices, msg.y_indices, msg.intensities):
		# 	data["time"] = t_str
		# 	data["x"].append(x)
		# 	data["y"].append(y)
		sdf = pd.concat([pd.DataFrame(data), sdf], axis=1)
		#sdf = pd.DataFrame(data)
		ti += 1
		#print("step:", ti)
		# for each timestep
		#sdf = df.sample(n=N)

		# Perform HDBSCAN clustering
		# clusters = dbscan.fit_predict(sdf[['x', 'y']])
		if rot_c == 64:
			t_str = pd.to_datetime(timestamp, unit='ns').strftime('%H:%M:%S')
			# Create a larger figure
			fig, ax = plt.subplots(figsize=(10, 8))

			# Plot the clusters
			# scatter = ax.scatter(sdf['x'], sdf['y'], c=clusters, cmap='turbo', s=0.5)
			scatter = ax.scatter(sdf['x'], sdf['y'], s=0.5)
			ax.set_xlabel('x')
			ax.set_ylabel('y')

			ax.set_xlim(-rng,rng)
			ax.set_ylim(-rng,rng)
			ax.set_title(f'HDBSCAN t: {t_str}')

			# Create legend labels and handles
			# unique_labels = np.unique(clusters)
			# legend_labels = ['{}'.format(label) for label in unique_labels]

			# # Create legend handles
			# legend_handles = [plt.Line2D([0], [0], marker='o', color=scatter.cmap(scatter.norm(label)), markersize=10) for label in unique_labels]

			# # Add legend
			# ax.legend(legend_handles, legend_labels, title='Labels', loc='upper right')

			# Save the plot in the specified folder
			file_path = os.path.join(out_folder, f't_{ti}.png')
			plt.savefig(file_path)
			plt.close()
			detections = 0
			frame_i += 1
			rot_c = 0
			sdf = pd.DataFrame(columns=["time", 'x', 'y'])

