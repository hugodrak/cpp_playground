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



out_folder = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/out"

LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/5-2/rosbag2_2024_02_20-13_11_18"
# LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/5-2/rosbag2_2024_02_16-14_54_56"

log_folder = os.path.basename(LOGPATH)
new_dir = os.path.join(out_folder, log_folder)
os.makedirs(new_dir, exist_ok=True)
out_folder = new_dir


dets = pd.read_csv("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/rosbag2_2024_02_20-13_11_18.csv")
objects = pd.read_csv("objects.csv")

dets['time'] = dets['time'].astype(int)
objects['time'] = objects['time'].astype(int)

end_time = dets["time"].iloc[-1]
rng = 50
for i in tqdm.tqdm(range(0, end_time, 1)):
	det = dets[dets['time'] == i]
	# Create a larger figure
	fig, ax = plt.subplots(figsize=(10, 8))
	ax.set_xlim(-rng, rng)
	ax.set_ylim(-rng, rng)

	# Plot the clusters
	scatter = ax.scatter(det['x'], det['y'], s=0.5)
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	#ax.set_title(f'DBSCAN t: {t_str}')

	objs = objects[objects['time'] == i]
	for j, obj in objs.iterrows():
		# print(obj)
		tlx = obj['tlx']
		tly = obj['tly']
		brx = obj['brx']
		bry = obj['bry']
		ax.plot([tlx, brx], [tly, tly], color='red')
		ax.plot([brx, brx], [tly, bry], color='red')
		ax.plot([brx, tlx], [bry, bry], color='red')
		ax.plot([tlx, tlx], [bry, tly], color='red')
		xpos = obj['xpos']
		ypos = obj['ypos']
		# str(obj['id'])
		# , color='black', fontsize=12., ha='center', va='center'
		#ax.text(xpos, ypos, '1')
		ax.text(xpos, ypos, str(int(obj['id'])), color='blue', fontsize=7., ha='center', va='center', bbox=dict(facecolor='red', alpha=0.5, edgecolor='black', boxstyle='round,pad=0.2') )

	# Save the plot in the specified folder
	file_path = os.path.join(out_folder, f'match_{i}.png')
	plt.savefig(file_path)
	plt.close()