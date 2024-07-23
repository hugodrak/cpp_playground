import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN

"""
Created by Hugo Drakeskär 2024
"""

class GridScanDF:
	def __init__(self, n, r, c, d, min_points=1):
		self.name = "Grid based clustering with merged clusters"
		self.shortname = "GridScan"
		# Check n is odd
		if n % 2 != 1:
			raise ValueError('grid_size must be an odd number')
		self.n = n # number of divisions along x and y axes grid_size
		self.r = r # max and min along x and y for grids symmetrically
		self.c = c # center box size ratio for setting heaviness
		self.d = d # meters, is max separation between clusters, i think this should be range independent
		self.mp = min_points # minimum number of points in a cluster
		self.edge_limit = 0.5 # the limit of where points can be from center to set eg points_left to TRUE

		# Determine grid cell size
		self.x_min = -self.r
		self.x_max = self.r
		self.y_min = -self.r
		self.y_max = self.r
		self.x_size = (self.x_max - self.x_min) / self.n
		self.y_size = (self.y_max - self.y_min) / self.n
		self.notcheck = []

	def merge_clusters(self, clusters_df, current_id, D, matchid):
		current_box = clusters_df[clusters_df['cluster_id'] == current_id].iloc[0]
		self.notcheck.append(current_id)
		# check all directions
		
		# Check up
		if not current_box['bottom_heavy']:
			up = clusters_df[(clusters_df['x_idx'] == current_box['x_idx']) & (clusters_df['y_idx'] == current_box['y_idx'] + 1)]
			if not up.empty:
				up = up.iloc[0] # can there be multiple in same box?
				if (up.cluster_id not in self.notcheck):
					if up['matched_id'] is None: # No double matching
						if abs(current_box['max_y'] - up['min_y']) < D: # Check if ups lowest point is D from current highest point
							if not up['top_heavy']:
								# recursion into next
								self.merge_clusters(clusters_df, up['cluster_id'], D, matchid)

								# set the up box matched id to current box id
								clusters_df.loc[clusters_df['cluster_id'] == up['cluster_id'], 'matched_id'] = matchid
		
		# Check down
		if not current_box['top_heavy']:
			down = clusters_df[(clusters_df['x_idx'] == current_box['x_idx']) & (clusters_df['y_idx'] == current_box['y_idx'] - 1)]
			if not down.empty:
				down = down.iloc[0] # can there be multiple in same box?
				if (down.cluster_id not in self.notcheck):
					if down['matched_id'] is None: # No double matching
						if abs(current_box['min_y'] - down['max_y']) < D: # Check if ups lowest point is D from current highest point
							if not down['bottom_heavy']:
								# recursion into next
								self.merge_clusters(clusters_df, down['cluster_id'], D, matchid)

								# set the up box matched id to current box id
								clusters_df.loc[clusters_df['cluster_id'] == down['cluster_id'], 'matched_id'] = matchid
		
		# Check left
		if not current_box['right_heavy']:
			left = clusters_df[(clusters_df['x_idx'] == current_box['x_idx'] - 1) & (clusters_df['y_idx'] == current_box['y_idx'])]
			if not left.empty:
				left = left.iloc[0] # can there be multiple in same box?
				if (left.cluster_id not in self.notcheck):
					if left['matched_id'] is None: # No double matching
						if abs(current_box['min_x'] - left['max_x']) < D: # Check if ups lowest point is D from current highest point
							if not left['left_heavy']:
								# recursion into next
								self.merge_clusters(clusters_df, left['cluster_id'], D, matchid)

								# set the up box matched id to current box id
								clusters_df.loc[clusters_df['cluster_id'] == left['cluster_id'], 'matched_id'] = matchid
		
		# Check right
		if not current_box['left_heavy']:
			right = clusters_df[(clusters_df['x_idx'] == current_box['x_idx'] + 1) & (clusters_df['y_idx'] == current_box['y_idx'])]
			if not right.empty:
				right = right.iloc[0] # can there be multiple in same box?
				if (right.cluster_id not in self.notcheck):
					if right['matched_id'] is None: # No double matching
						if abs(current_box['max_x'] - right['min_x']) < D: # Check if ups lowest point is D from current highest point
							if not right['right_heavy']:
								# recursion into next
								self.merge_clusters(clusters_df, right['cluster_id'], D, matchid)

								# set the up box matched id to current box id
								clusters_df.loc[clusters_df['cluster_id'] == right['cluster_id'], 'matched_id'] = matchid
		return clusters_df

	def merge_clusters_new(self, clusters_df, current_id, D, matchid):
		current_box = clusters_df[clusters_df['cluster_id'] == current_id].iloc[0]
		self.notcheck.append(current_id)
		# check all directions
		keys = ["up", "down", "left", "right"]
		dir_lut = {"up": [0, 1], "down": [0, -1], "left": [-1, 0], "right": [1, 0]}
		other_heavy_lut = {"up": "bottom_heavy", "down": "top_heavy", "left": "right_heavy", "right": "left_heavy"}
		self_heavy_lut = {"up": "top_heavy", "down": "bottom_heavy", "left": "left_heavy", "right": "right_heavy"}
		coordinate_lut = {"up": {'self': 'max_y', 'neighbour': 'min_y'}, "down": {'self': 'min_y', 'neighbour': 'max_y'}, 
					"left": {'self': 'max_x', 'neighbour': 'min_x'}, "right": {'self': 'min_x', 'neighbour': 'max_x'}}
		# for c++ do bits in like 0010 for heaviness
		for key in keys:
			if not current_box[self_heavy_lut[key]]:
				neighbour = clusters_df[(clusters_df['x_idx'] == current_box['x_idx'] + dir_lut[key][0]) & (clusters_df['y_idx'] == current_box['y_idx'] + dir_lut[key][1])]
				if not neighbour.empty:
					neighbour = neighbour.iloc[0]
					if (neighbour.cluster_id not in self.notcheck):
						if neighbour['matched_id'] is None:
							if abs(current_box[coordinate_lut[key]['self']] - neighbour[coordinate_lut[key]['neighbour']]) < D:
								if not neighbour[other_heavy_lut[key]]:
									clusters_df = self.merge_clusters(clusters_df, neighbour['cluster_id'], D, matchid)
									clusters_df.loc[clusters_df['cluster_id'] == neighbour['cluster_id'], 'matched_id'] = matchid

		return clusters_df

	def fit_predict(self, points): # should return a list of labels
		df = points.copy()
		
		# Assign points to grid cells
		df['x_idx'] = ((df['x'] - self.x_min) / self.x_size).astype(int)
		df['y_idx'] = ((df['y'] - self.y_min) / self.y_size).astype(int)

		# Create initial clusters based on grid cells
		initial_clusters = df.groupby(['x_idx', 'y_idx']).apply(lambda group: group.index.tolist()).tolist()

		# Initialize a new DataFrame for clusters
		dtypes = {
			'cluster_id': 'int32',
			'matched_id': 'int32',
			'x_idx': 'int32',
			'y_idx': 'int32',
			'right_heavy': 'bool',
			'top_heavy': 'bool',
			'left_heavy': 'bool',
			'bottom_heavy': 'bool',
			'box_centre_x': 'float64',
			'box_centre_y': 'float64',
			'point_count': 'int32',
			'max_x': 'float64',
			'min_x': 'float64',
			'max_y': 'float64',
			'min_y': 'float64',
			'mass_center_x': 'float64',
			'mass_center_y': 'float64',
		}

		clusters_df = pd.DataFrame({col: pd.Series(dtype=typ) for col, typ in dtypes.items()})
		
		# Assign cluster IDs and calculate centers of mass
		for cluster_id, cluster_indices in enumerate(initial_clusters):
			# print("cluster id:", cluster_id, "contains", len(cluster_indices), "points")
			cluster_points = df.loc[cluster_indices]

			x_idx = cluster_points['x_idx'].iloc[0]
			y_idx = cluster_points['y_idx'].iloc[0]
			mass_center_x = cluster_points['x'].mean()
			mass_center_y = cluster_points['y'].mean()
			point_count = len(cluster_points)

			box_centre_x = self.x_min + (x_idx + 0.5) * self.x_size
			box_centre_y = self.y_min + (y_idx + 0.5) * self.y_size

			max_x = cluster_points['x'].max()
			min_x = cluster_points['x'].min()
			max_y = cluster_points['y'].max()
			min_y = cluster_points['y'].min()


			# do calcuations to see if there exists points in the borders
			points_right = (cluster_points['x'] > (box_centre_x + self.x_size/2*self.edge_limit)).any()
			points_left = (cluster_points['x'] < (box_centre_x - self.x_size/2*self.edge_limit)).any()
			points_top = (cluster_points['y'] > (box_centre_y + self.y_size/2*self.edge_limit)).any()
			points_bottom = (cluster_points['y'] < (box_centre_y - self.y_size/2*self.edge_limit)).any()

			# add heavines bool
			right_heavy = (mass_center_x > box_centre_x+ self.x_size*self.c) and not points_left # we add a center box to the heaviness
			left_heavy = (mass_center_x < box_centre_x - self.x_size*self.c) and not points_right
			top_heavy = (mass_center_y > box_centre_y + self.y_size*self.c) and not points_bottom
			bottom_heavy = (mass_center_y < box_centre_y - self.y_size*self.c) and not points_top

			new_clust = pd.DataFrame({'cluster_id': [cluster_id], 'matched_id': [None], 'x_idx': [x_idx], 'y_idx': [y_idx], 
								'mass_center_x': [mass_center_x], 'mass_center_y': [mass_center_y], 'max_x': [max_x], 'min_x': [min_x], 
								'max_y': [max_y], 'min_y': [min_y],
								'right_heavy': [right_heavy], 'top_heavy': [top_heavy], 'left_heavy': [left_heavy], 'bottom_heavy': [bottom_heavy],
								'box_centre_x': [box_centre_x], 'box_centre_y': [box_centre_y], 'point_count': [point_count]})

			if clusters_df.empty:
				clusters_df = new_clust
			else:
				clusters_df = pd.concat([clusters_df, new_clust], ignore_index=True)
			df.loc[cluster_indices, 'cluster_id'] = cluster_id


		# Merge clusters based on centers of mass closer than D apart
		self.notcheck = []
		for index in clusters_df.index:
			current_box = clusters_df.loc[index]
			current_id = current_box.cluster_id
			# Merge clusters based on centers of mass closer than D apart
			if current_box['matched_id'] is None:
				clusters_df = self.merge_clusters(clusters_df, current_id, self.d, current_id)

			if current_box['matched_id'] is None:
				clusters_df.loc[clusters_df['cluster_id'] == current_id, 'matched_id'] = current_id

		unique_merged_ids = clusters_df['matched_id'].unique()

		# Plot 2: Grid-based Clustering with Merged Clusters
		dtypes = {
			'id': 'int32',
			'mass_center_x': 'float64',
			'mass_center_y': 'float64',
			'max_x': 'float64',
			'min_x': 'float64',
			'max_y': 'float64',
			'min_y': 'float64',
			'box_centre_x': 'float64',
			'box_centre_y': 'float64',
			'point_count': 'int32',
			'box_count': 'int32'
		}

		mc_df = pd.DataFrame({col: pd.Series(dtype=typ) for col, typ in dtypes.items()})

		# mc_df = pd.DataFrame(columns=['id', 'mass_center_x', 'mass_center_y', 'max_x', 'min_x', 'max_y', 'min_y', 'box_centre_x', 'box_centre_y', 'point_count', 'box_count'])
		new_id = 0
		for i, id in enumerate(unique_merged_ids):
			new_clusters = clusters_df[clusters_df['matched_id'] == id]
			box_count = len(new_clusters)
			total_mass = new_clusters['point_count'].sum()

			if total_mass < self.mp:
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
								'max_y': [max_y], 'min_y': [min_y], 'box_centre_x': [box_centre_x], 'box_centre_y': [box_centre_y], 
								'point_count': [total_mass], 'box_count': [box_count]})

			if mc_df.empty:
				mc_df = new_clust
			else:
				mc_df = pd.concat([mc_df, new_clust], ignore_index=True)
					
			new_id += 1

		return mc_df
	

class DBSCANDF:
	def __init__(self, eps, min_samples):
		self.name = f"DBSCAN with eps={eps} and min_samples={min_samples}"
		self.shortname = "DBSCAN"
		self.eps = eps
		self.min_samples = min_samples

		self.dbscan = DBSCAN(eps=self.eps, min_samples=self.min_samples)

	def fit_predict(self, points):
		df = points.copy()
		# Perform DBSCAN clustering
		labels = self.dbscan.fit_predict(df[['x', 'y']])

		# add labels to poitns
		df['cluster'] = labels

		dtypes = {
			'id': 'int32',
			'mass_center_x': 'float64',
			'mass_center_y': 'float64',
			'max_x': 'float64',
			'min_x': 'float64',
			'max_y': 'float64',
			'min_y': 'float64',
			'box_centre_x': 'float64',
			'box_centre_y': 'float64',
			'point_count': 'int32',
			'box_count': 'int32'
		}

		clusters = pd.DataFrame({col: pd.Series(dtype=typ) for col, typ in dtypes.items()})

		# clusters = pd.DataFrame(columns=['id', 'mass_center_x', 'mass_center_y', 'max_x', 'min_x', 
		# 						   'max_y', 'min_y', 'box_centre_x', 'box_centre_y', 'point_count', 'box_count'])

		for id in np.unique(labels):
			# skip the noise
			if id == -1:
				continue
				
			
			cluster_points = df[df['cluster'] == id]
			point_count = len(cluster_points)
			centre_x = cluster_points['x'].mean()
			centre_y = cluster_points['y'].mean()
			max_x = cluster_points['x'].max()
			min_x = cluster_points['x'].min()
			max_y = cluster_points['y'].max()
			min_y = cluster_points['y'].min()
			box_centre_x = (max_x + min_x) / 2
			box_centre_y = (max_y + min_y) / 2

			new_clust = pd.DataFrame({'id': [id], 'mass_center_x': [centre_x], 'mass_center_y': [centre_y], 'max_x': [max_x], 'min_x': [min_x], 
								'max_y': [max_y], 'min_y': [min_y], 'box_centre_x': [box_centre_x], 
								'box_centre_y': [box_centre_y], 'point_count': [point_count], 'box_count': [1]})

			if clusters.empty:
				clusters = new_clust
			else:
				clusters = pd.concat([clusters, new_clust], ignore_index=True)
						
		return clusters
