import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path
import tqdm
import os



#cluster_out = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/compare/out/grid/clusters"
obj_out = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/compare/out/grid/objects"



dets = pd.read_csv("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/log_3.csv")
objects = pd.read_csv("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/tracker_grid/objects.csv")
clusters = pd.read_csv("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/tracker_grid/clusters.csv")

dets['time'] = dets['time'].astype(int)
objects['time'] = objects['time'].astype(int)
clusters['time'] = clusters['time'].astype(int)

start_time = dets["time"].iloc[0]
end_time = dets["time"].iloc[-1]
rng = 50
x_ticks = np.linspace(-rng, rng, 27 + 1)
y_ticks = np.linspace(-rng, rng, 27 + 1)
for i in tqdm.tqdm(range(start_time, end_time+1, 1)):
	det = dets[dets['time'] == i]
	# print("dets", len(det))
	# Create a larger figure
	fig, axs = plt.subplots(1,2,figsize=(20, 8))

	# Plot the clusters
	scatter = axs[0].scatter(det['x'], det['y'], s=0.5)
	axs[0].set_title("Objects")

	axs[0].set_xticks(np.round(x_ticks, decimals=1))
	axs[0].set_xticklabels(np.round(x_ticks, decimals=1), rotation=45, ha='right')  # Set x tick labels at 45-degree downward angle
	axs[0].set_yticks(np.round(y_ticks, decimals=1))
	axs[0].grid(True, linestyle='--', linewidth=0.5, color='gray')
	axs[0].set_xlabel('X axis')
	axs[0].set_ylabel('Y axis')
	axs[0].set_xlim(-rng, rng)
	axs[0].set_ylim(-rng, rng)


	scatter = axs[1].scatter(det['x'], det['y'], s=0.5)
	axs[1].set_title("Clusters")

	axs[1].set_xticks(np.round(x_ticks, decimals=1))
	axs[1].set_xticklabels(np.round(x_ticks, decimals=1), rotation=45, ha='right')  # Set x tick labels at 45-degree downward angle
	axs[1].set_yticks(np.round(y_ticks, decimals=1))
	axs[1].grid(True, linestyle='--', linewidth=0.5, color='gray')
	axs[1].set_xlabel('X axis')
	axs[1].set_ylabel('Y axis')
	axs[1].set_xlim(-rng, rng)
	axs[1].set_ylim(-rng, rng)
	#ax.set_title(f'DBSCAN t: {t_str}')

	objs = objects[objects['time'] == i]
	clusts = clusters[clusters['time'] == i]
	# print("obs:", len(objs))
	for j, obj in objs.iterrows():
		# print(obj)
		tlx = obj['tlx']
		tly = obj['tly']
		brx = obj['brx']
		bry = obj['bry']
		axs[0].plot([tlx, brx], [tly, tly], color='red')
		axs[0].plot([brx, brx], [tly, bry], color='red')
		axs[0].plot([brx, tlx], [bry, bry], color='red')
		axs[0].plot([tlx, tlx], [bry, tly], color='red')
		xpos = obj['xpos']
		ypos = obj['ypos']
		# print(xpos, ypos)
		# str(obj['id'])
		# , color='black', fontsize=12., ha='center', va='center'
		#ax.text(xpos, ypos, '1')
		axs[0].text(xpos, ypos, str(int(obj['id'])), color='black', fontsize=7., ha='center', va='center', bbox=dict(facecolor='white', alpha=0.5, edgecolor='black', boxstyle='round,pad=0.2') )

		if obj["classification"] == 2:
			# Input: angle in degrees from your 'world' system
			angle_deg = obj['est_heading']  # This is the 'world' angle

			# Convert 'world' angle back to 'mathematical' angle
			angle_deg_math = (360 - angle_deg + 90) % 360

			# Convert angle to radians for trigonometric functions
			angle_radians = np.deg2rad(angle_deg_math)

			# Calculate change in x and y based on speed and angle
			speed = obj['est_velocity']  # Your speed value
			dx = speed * np.cos(angle_radians) * 10
			dy = speed * np.sin(angle_radians) * 10

			# Plotting the arrow
			axs[0].arrow(xpos, ypos, dx, dy, head_width=1, head_length=1, fc='black', ec='black')

	
	for k, clust in clusts.iterrows():
		# print(obj)
		tlx = clust['tlx']
		tly = clust['tly']
		brx = clust['brx']
		bry = clust['bry']
		axs[1].plot([tlx, brx], [tly, tly], color='green')
		axs[1].plot([brx, brx], [tly, bry], color='green')
		axs[1].plot([brx, tlx], [bry, bry], color='green')
		axs[1].plot([tlx, tlx], [bry, tly], color='green')

		xpos = clust['xpos']
		ypos = clust['ypos']
		# print(xpos, ypos)
		# str(clust['id'])
		# , color='black', fontsize=12., ha='center', va='center'
		#ax.text(xpos, ypos, '1')
		axs[1].text(xpos, ypos, str(int(clust['id'])), color='white', fontsize=7., ha='center', va='center', bbox=dict(facecolor='green', alpha=0.5, edgecolor='black', boxstyle='round,pad=0.2') )

	# Save the plot in the specified folder
	file_path = os.path.join(obj_out, f'match_{i}.png')
	plt.savefig(file_path)
	plt.close()