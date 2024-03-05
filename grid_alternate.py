import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.patches as patches

# Generate synthetic data
np.random.seed(0)

# df = pd.read_csv('/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/rosbag2_2024_02_20-13_11_18.csv')
# df = df.sample(n=1000)
# df.reset_index(drop=True, inplace=True)

df = pd.read_csv('/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/rosbag2_2024_02_16-14_54_56.csv')
df = df[(df['time'] >= 34) & (df['time'] < 35)] #
#df = df[(df['x'] >= -9.3) & (df['x'] < 1.9) & (df['y'] >= 20.4) & (df['y'] < 35.2)] #
# df = df.sample(n=1000)
df.reset_index(drop=True, inplace=True)

print(df)


# TODO: add min points per cluster

# df = df[(df['x'] > -5.6) & (df['x'] < 1.9) & (df['y'] > 1.9) & (df['y'] < 16.7)]

# Parameters
r = 50  # Range for x and y
#D = 10  # Distance threshold for merging clusters
n = 27  # Grid size (number of divisions along x and y axes)
c = 0.1 # center box size ratio for setting heaviness
min_points = 40 # min points per cluster, change based on range
#D = x_size / (1.5) # must be div by max 2, rn, 75% of box size
#D = 0.3 # meters, is max separation between clusters
D = 0.4 # meters, is max separation between clusters, i think this should be range independent
print("D:", D, "m")


# TODO: do n*n for clsuters! so that updated clusters have updated com and redo clustering so not one shot
# todo: add doppler data when doing bounding boxes
# Check n is odd
if n % 2 != 1:
	raise ValueError('n must be an odd number')


plt.figure(figsize=(10, 10))
# Determine grid cell size
x_min, x_max, y_min, y_max = -r, r, -r, r
x_size = (x_max - x_min) / n
y_size = (y_max - y_min) / n


# Assign points to grid cells
df['x_idx'] = ((df['x'] - x_min) / x_size).astype(int)
df['y_idx'] = ((df['y'] - y_min) / y_size).astype(int)

# Create initial clusters based on grid cells
initial_clusters = df.groupby(['x_idx', 'y_idx']).apply(lambda group: group.index.tolist()).tolist()

# Initialize a new DataFrame for clusters
clusters_df = pd.DataFrame(columns=['cluster_id', 'mass_center_x', 'mass_center_y'])

# Assign cluster IDs and calculate centers of mass
for cluster_id, cluster_indices in enumerate(initial_clusters):
	# print("cluster id:", cluster_id, "contains", len(cluster_indices), "points")
	cluster_points = df.loc[cluster_indices]

	x_idx = cluster_points['x_idx'].iloc[0]
	y_idx = cluster_points['y_idx'].iloc[0]
	mass_center_x = cluster_points['x'].mean()
	mass_center_y = cluster_points['y'].mean()
	point_count = len(cluster_points)

	box_centre_x = x_min + (x_idx + 0.5) * x_size
	box_centre_y = y_min + (y_idx + 0.5) * y_size

	max_x = cluster_points['x'].max()
	min_x = cluster_points['x'].min()
	max_y = cluster_points['y'].max()
	min_y = cluster_points['y'].min()



	# add heavines bool
	right_hyeavy = (mass_center_x > box_centre_x+ x_size*c)
	left_heavy = (mass_center_x < box_centre_x - x_size*c)
	top_heavy = (mass_center_y > box_centre_y + y_size*c)
	bottom_heavy = (mass_center_y < box_centre_y - y_size*c)

	
	new_clust = pd.DataFrame({'cluster_id': [cluster_id], 'matched_id': [None], 'x_idx': [x_idx], 'y_idx': [y_idx], 
						   'mass_center_x': [mass_center_x], 'mass_center_y': [mass_center_y], 'max_x': [max_x], 'min_x': [min_x], 
						   'max_y': [max_y], 'min_y': [min_y],
						   'right_heavy': [right_hyeavy], 'top_heavy': [top_heavy], 'left_heavy': [left_heavy], 'bottom_heavy': [bottom_heavy],
						   'box_centre_x': [box_centre_x], 'box_centre_y': [box_centre_y], 'point_count': [point_count]})

	clusters_df = pd.concat([clusters_df, new_clust], ignore_index=True)
	#clusters_df = clusters_df.append({'cluster_id': cluster_id, 'center_x': center_x, 'center_y': center_y}, ignore_index=True)
	df.loc[cluster_indices, 'cluster_id'] = cluster_id
	 
clusters_df['x_idx'] = clusters_df['x_idx'].astype(int)
clusters_df['y_idx'] = clusters_df['y_idx'].astype(int)

df['cluster_id'] = df['cluster_id'].astype(int)




# fig, axs = plt.subplots(1,2, figsize=(12, 12))
x_ticks = np.linspace(x_min, x_max, n + 1)
y_ticks = np.linspace(y_min, y_max, n + 1)

plt.xticks(x_ticks)
plt.yticks(y_ticks)
plt.grid(True, linestyle='--', linewidth=0.5, color='gray')
# Plot 1: Grid-based Clustering without Merged Clusters
# plt.scatter(df['x'], df['y'], c=df['cluster_id'], cmap='turbo', s=0.5)
plt.scatter(df['x'], df['y'], color='black', s=0.5)
# for _, row in clusters_df.iterrows():
#     plt.scatter(row['mass_center_x'], row['mass_center_y'], c='red', marker='x')  # Marking the centers
#     #plt.text(row['box_centre_x'], row['box_centre_y'], row['cluster_id'], fontsize=9, color='black', ha='center', va='center')

plt.title('Grid-based Clustering without Merged Clusters')
plt.xlabel('X axis')
plt.ylabel('Y axis')
# plt.set_xlim(-10, 10)
# plt.set_ylim(-2, 18)
plt.xlim(-r, r)
plt.ylim(-r, r)


def merge_clusters(clusters_df, current_id, D, matchid, notcheck=[]):
	current_box = clusters_df[clusters_df['cluster_id'] == current_id].iloc[0]
	notcheck.append(current_id)
	# check all directions
	
	# Check up
	if not current_box['bottom_heavy']:
		up = clusters_df[(clusters_df['x_idx'] == current_box['x_idx']) & (clusters_df['y_idx'] == current_box['y_idx'] + 1)]
		if not up.empty:
			up = up.iloc[0] # can there be multiple in same box?
			if (up.cluster_id not in notcheck):
				if up['matched_id'] is None: # No double matching
					if abs(current_box['max_y'] - up['min_y']) < D: # Check if ups lowest point is D from current highest point
						if not up['top_heavy']:
							# recursion into next
							merge_clusters(clusters_df, up['cluster_id'], D, matchid, notcheck)

							# set the up box matched id to current box id
							clusters_df.loc[clusters_df['cluster_id'] == up['cluster_id'], 'matched_id'] = matchid
	
	# Check down
	if not current_box['top_heavy']:
		down = clusters_df[(clusters_df['x_idx'] == current_box['x_idx']) & (clusters_df['y_idx'] == current_box['y_idx'] - 1)]
		if not down.empty:
			down = down.iloc[0] # can there be multiple in same box?
			if (down.cluster_id not in notcheck):
				if down['matched_id'] is None: # No double matching
					if abs(current_box['min_y'] - down['max_y']) < D: # Check if ups lowest point is D from current highest point
						if not down['bottom_heavy']:
							# recursion into next
							merge_clusters(clusters_df, down['cluster_id'], D, matchid, notcheck)

							# set the up box matched id to current box id
							clusters_df.loc[clusters_df['cluster_id'] == down['cluster_id'], 'matched_id'] = matchid
	
	# Check left
	if not current_box['right_heavy']:
		left = clusters_df[(clusters_df['x_idx'] == current_box['x_idx'] - 1) & (clusters_df['y_idx'] == current_box['y_idx'])]
		if not left.empty:
			left = left.iloc[0] # can there be multiple in same box?
			if (left.cluster_id not in notcheck):
				if left['matched_id'] is None: # No double matching
					if abs(current_box['min_x'] - left['max_x']) < D: # Check if ups lowest point is D from current highest point
						if not left['left_heavy']:
							# recursion into next
							merge_clusters(clusters_df, left['cluster_id'], D, matchid, notcheck)

							# set the up box matched id to current box id
							clusters_df.loc[clusters_df['cluster_id'] == left['cluster_id'], 'matched_id'] = matchid
	
	# Check right
	if not current_box['left_heavy']:
		right = clusters_df[(clusters_df['x_idx'] == current_box['x_idx'] + 1) & (clusters_df['y_idx'] == current_box['y_idx'])]
		if not right.empty:
			right = right.iloc[0] # can there be multiple in same box?
			if (right.cluster_id not in notcheck):
				if right['matched_id'] is None: # No double matching
					if abs(current_box['max_x'] - right['min_x']) < D: # Check if ups lowest point is D from current highest point
						if not right['right_heavy']:
							# recursion into next
							merge_clusters(clusters_df, right['cluster_id'], D, matchid, notcheck)

							# set the up box matched id to current box id
							clusters_df.loc[clusters_df['cluster_id'] == right['cluster_id'], 'matched_id'] = matchid
	return clusters_df


		




# Merge clusters based on centers of mass closer than D apart
for index in clusters_df.index:
	current_box = clusters_df.loc[index]
	current_id = current_box.cluster_id
	# Merge clusters based on centers of mass closer than D apart
	if current_box['matched_id'] is None:
		clusters_df = merge_clusters(clusters_df, current_id, D, current_id)

	if current_box['matched_id'] is None:
		clusters_df.loc[clusters_df['cluster_id'] == current_id, 'matched_id'] = current_id

print(clusters_df)


unique_merged_ids = clusters_df['matched_id'].unique()


# Plot 2: Grid-based Clustering with Merged Clusters
mc_df = pd.DataFrame(columns=['id', 'mass_center_x', 'mass_center_y', 'max_x', 'min_x', 'max_y', 'min_y', 'box_centre_x', 'box_centre_y', 'point_count', 'box_count'])
new_id = 0
for i, id in enumerate(unique_merged_ids):
	new_clusters = clusters_df[clusters_df['matched_id'] == id]
	box_count = len(new_clusters)
	total_mass = new_clusters['point_count'].sum()

	if total_mass < min_points:
		continue

	centre_x = (new_clusters['mass_center_x'] * new_clusters['point_count']).sum() / total_mass
	centre_y = (new_clusters['mass_center_y'] * new_clusters['point_count']).sum() / total_mass
	max_x = new_clusters['max_x'].max()
	min_x = new_clusters['min_x'].min()
	max_y = new_clusters['max_y'].max()
	min_y = new_clusters['min_y'].min()
	box_centre_x = (max_x + min_x) / 2
	box_centre_y = (max_y + min_y) / 2

	new_clust = pd.DataFrame({'id': [new_id], 'mass_center_x': [centre_x], 'mass_center_y': [centre_y], 'max_x': [max_x], 'min_x': [min_x], 
						   'max_y': [max_y], 'min_y': [min_y], 'box_centre_x': [box_centre_x], 'box_centre_y': [box_centre_y], 'point_count': [total_mass], 'box_count': [box_count]})
	mc_df = pd.concat([mc_df, new_clust], ignore_index=True)
	new_id += 1


# mc_df = mc_df[mc_df['box_count'] > 1]


# Plot 2: Grid-based Clustering with Merged Clusters
# axs[1].scatter(df['x'], df['y'], c=df['matched_id'], cmap='viridis', alpha=0.6, edgecolors='k')
# for _, row in clusters_df.iterrows():
#     axs[1].scatter(row['mass_center_x'], row['mass_center_y'], c='red', marker='x')  # Marking the centers
# axs[1].set_title('Grid-based Clustering with Merged Clusters')
# axs[1].set_xlabel('X axis')
# axs[1].set_ylabel('Y axis')
# axs[1].set_xticks(x_ticks)
# axs[1].set_yticks(y_ticks)
# axs[1].grid(True, linestyle='--', linewidth=0.5, color='gray')
# axs[1].set_xlim(-r/2, r/2)
# axs[1].set_ylim(-r/2, r/2)

#axs[1].scatter(df['x'], df['y'], c=df['matched_id'], cmap='viridis', alpha=0.6, edgecolors='k')

# colors = cm.viridis(np.linspace(0, 1, len(clusters_df['matched_id'].unique())))



# axs[1].scatter(clusters_df['mass_center_x'], clusters_df['mass_center_y'], c=clusters_df['matched_id'], cmap='turbo', alpha=0.6, edgecolors='k')  # Marking the centers
# for _, row in clusters_df.iterrows():
# 	axs[1].text(row['box_centre_x'], row['box_centre_y'], row['matched_id'], fontsize=9, color='black', ha='center', va='center')

#     # axs[1].text(row['box_centre_x'], row['box_centre_y'], row['matched_id'], fontsize=12, color='black')

# axs[1].set_title('Grid-based Clustering with Merged Clusters')
# axs[1].set_xlabel('X axis')
# axs[1].set_ylabel('Y axis')
# axs[1].set_xticks(x_ticks)
# axs[1].set_yticks(y_ticks)
# axs[1].grid(True, linestyle='--', linewidth=0.5, color='gray')
# # axs[1].set_xlim(-10, 10)
# # axs[1].set_ylim(-2, 18)
# axs[1].set_xlim(-r, r)
# axs[1].set_ylim(-r, r)



#plt.scatter(mc_df['mass_center_x'], mc_df['mass_center_y'], c=mc_df['id'], cmap='turbo', alpha=0.6, edgecolors='k')  # Marking the centers

for _, row in mc_df.iterrows():
	# axs[2].text(row['box_centre_x'], row['box_centre_y'], row['id'], fontsize=9, color='black', ha='center', va='center')
	# Draw bounding box
	print("cluster id:", row['id'], "contains", row['point_count'], "points")

	plt.text(row['mass_center_x'], row['mass_center_y'], row['id'], fontsize=9, color='black', ha='center', va='center', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))  # Marking the centers
	rect = patches.Rectangle((row['min_x'], row['min_y']), row['max_x'] - row['min_x'], row['max_y'] - row['min_y'], linewidth=1, edgecolor='r', facecolor='none')
	plt.gca().add_patch(rect)


# axs[2].set_title('Grid-based Clustering with Merged Clusters')
# axs[2].set_xlabel('X axis')
# axs[2].set_ylabel('Y axis')
# axs[2].set_xticks(x_ticks)
# axs[2].set_yticks(y_ticks)
# axs[2].grid(True, linestyle='--', linewidth=0.5, color='gray')
# # axs[1].set_xlim(-10, 10)
# # axs[1].set_ylim(-2, 18)
# axs[2].set_xlim(-r, r)
# axs[2].set_ylim(-r, r)



plt.tight_layout()
plt.show()