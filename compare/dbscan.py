import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.patches as patches

from sklearn.cluster import DBSCAN
# Generate synthetic data
np.random.seed(0)
r = 50
# df = pd.read_csv('/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/rosbag2_2024_02_20-13_11_18.csv')
# df = df.sample(n=1000)
# df.reset_index(drop=True, inplace=True)
plt.figure(figsize=(10, 10))

df = pd.read_csv('/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/rosbag2_2024_02_16-14_54_56.csv')
df = df[(df['time'] >= 34) & (df['time'] < 35)] #
#df = df.sample(n=10000)
df.reset_index(drop=True, inplace=True)

print(df)
eps = 2
min_samples = 40
gridsize = 27  # Grid size (number of divisions along x and y axes)

plt.title(f'Dbscan Clustering eps={eps} min_samples={min_samples}')
plt.xlabel('X axis')
plt.ylabel('Y axis')
plt.xticks(np.linspace(-r, r, gridsize))
plt.yticks(np.linspace(-r, r, gridsize))
plt.xlim(-r, r)
plt.ylim(-r, r)
plt.grid(True, linestyle='--', linewidth=0.5, color='gray')


# Extract the features from the dataframe
# features = df[['x', 'y']].values

# Perform DBSCAN clustering
dbscan = DBSCAN(eps=eps, min_samples=min_samples)
labels = dbscan.fit_predict(df[['x', 'y']])

# Add the cluster labels to the dataframe
df['cluster'] = labels

# Plot the clusters
# fig, ax = plt.subplots()
for cluster_label in np.unique(labels):
	cluster_points = df[df['cluster'] == cluster_label]

	x_center = cluster_points['x'].mean()
	y_center = cluster_points['y'].mean()
	x_min, x_max = cluster_points['x'].min(), cluster_points['x'].max()
	y_min, y_max = cluster_points['y'].min(), cluster_points['y'].max()

	if cluster_label == -1: # Skip the noise
		plt.scatter(cluster_points['x'], cluster_points['y'], s=0.5, color='black')
		continue

	plt.scatter(cluster_points['x'], cluster_points['y'], s=0.5, color='blue')
	plt.text(x_center, y_center, cluster_label, fontsize=9, color='black', ha='center', va='center', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))  # Marking the centers
	rect = patches.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, linewidth=1, edgecolor='r', facecolor='none')
	plt.gca().add_patch(rect)

plt.show()