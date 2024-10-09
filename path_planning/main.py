import json
import cv2
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from time import time
from matplotlib.colors import LinearSegmentedColormap


from a_star import a_star


class ChartLoader:
	Ltype_lut = {"COALNE": {"id": 1, "desc": "Coastline"}, "LNDRGN": {"id":2, "desc": "Land Region"}, 
			 "BOYCAR": {"id":3, "desc": "Boy"}}
	
	def __init__(self, chart_path):
		self.chart_path = chart_path
		self.df_chart = pd.DataFrame(columns=["Ltype", "id", "lat", "lon", "text", "point"])
		self.chart = self.load_chart()
		self.calculate_bounds()
		  
	def load_chart(self):
		with open(self.chart_path) as file:
			chart = json.load(file)
			num_layers = len(chart["layers"])
			print(f"Loaded chart with {num_layers} layers")

			tdf = pd.DataFrame(columns=["Ltype", "id", "lat", "lon", "text", "point"])
			for layer in chart["layers"]:
				key = layer["layer_name"]
				if key == "COALNE":
					coastlines = layer["features"]
					for feature in coastlines:
						id = feature["id"]
						geometry = feature["geometry"]
						lat = []
						lon = []
						
						idss = []
						
						coordinates = geometry["coordinates"]
						for coord in coordinates:
							lon.append(coord[0])
							lat.append(coord[1])
							idss.append(id)
						df = pd.DataFrame({"Ltype": self.Ltype_lut[key]["id"], "id": id, "lat": lat, "lon": lon, "text": None, "point":False})
						if tdf.empty:
							tdf = df
						else:
							tdf = pd.concat([tdf, df], ignore_index=True)
				elif key == "LNDRGN":
					coastlines = layer["features"]
					for feature in coastlines:
						id = feature["id"]
						geometry = feature["geometry"]
						lat = []
						lon = []
						idss = []
						
						coordinates = geometry["coordinates"]

						objname = feature["properties"]["OBJNAM"]
						
						lon.append(coordinates[0])
						lat.append(coordinates[1])
						idss.append(id)

						df = pd.DataFrame({"Ltype":self. Ltype_lut[key]["id"], "id": id, "lat": lat, "lon": lon, "text": objname, "point":True})
						if tdf.empty:
							tdf = df
						else:
							tdf = pd.concat([tdf, df], ignore_index=True)
				elif key == "BOYCAR":
					coastlines = layer["features"]
					for feature in coastlines:
						id = feature["id"]
						geometry = feature["geometry"]
						lat = []
						lon = []
						idss = []
						
						coordinates = geometry["coordinates"]

						objname = feature["properties"]["OBJNAM"]
						
						lon.append(coordinates[0])
						lat.append(coordinates[1])
						idss.append(id)

						df = pd.DataFrame({"Ltype": self.Ltype_lut[key]["id"], "id": id, "lat": lat, "lon": lon, "text": None, "point":True})
						#print(df)
						if tdf.empty:
							tdf = df
						else:
							tdf = pd.concat([tdf, df], ignore_index=True)
		self.df_chart = tdf

	def calculate_bounds(self):
		#print(self.df_chart)
		self.lat_min = self.df_chart["lat"].min()
		self.lat_max = self.df_chart["lat"].max()
		self.lat_mid = (self.lat_max-self.lat_min)/2
		self.lat_diff = np.abs(self.lat_max-self.lat_min)

		self.lon_min = self.df_chart["lon"].min()
		self.lon_max = self.df_chart["lon"].max()
		self.lon_diff = np.abs(self.lon_max-self.lon_min)
		self.aspect_ratio = self.lon_diff / self.lat_diff

		self.avg_lat = self.df_chart["lat"].mean()

		# Calculate aspect ratio adjustment for the longitude
		self.scale_factor = np.cos(np.radians(self.avg_lat))

		# Adjust longitude range based on the average latitude's cosine
		self.adjusted_lon_range = (self.lon_max - self.lon_min) * self.scale_factor


		print("Latitude min: ", self.lat_min)
		print("Latitude max: ", self.lat_max)
		print("Longitude min: ", self.lon_min)
		print("Longitude max: ", self.lon_max)
		print("Aspect ratio: ", self.aspect_ratio)


class ChartPlotter:
	def __init__(self, chart) -> None:
		self.chart = chart
		self.height = 2000
		self.width = int(self.height * self.chart.aspect_ratio)
		self.coords_to_xy()
		

	def coords_to_xy(self):
		#lons, lats = self.chart.df_chart["lon"], self.chart.df_chart["lat"]
		#xs = int((lons - self.chart.lon_min) * self.chart.scale_factor / (self.chart.lon_max - self.chart.lon_min) * (self.width - 1))
		#ys = self.height - int((lats - self.chart.lat_min) / (self.chart.lat_max - self.chart.lat_min) * (self.height - 1))
		# add these scaled to df

		def calculate_x_value(df):
			return int((df["lon"] - self.chart.lon_min) * self.chart.scale_factor / (self.chart.lon_max - self.chart.lon_min) * (self.width - 1))
		def calculate_y_value(df):
			return self.height - int((df["lat"] - self.chart.lat_min) / (self.chart.lat_max - self.chart.lat_min) * (self.height - 1))
		# Apply the function to create a new column 'E'
		self.chart.df_chart['x'] = self.chart.df_chart.apply(calculate_x_value, axis=1)
		self.chart.df_chart['y'] = self.chart.df_chart.apply(calculate_y_value, axis=1)
		#self.chart.df_chart["x"] = xs
		#self.chart.df_chart["y"] = ys
		

	def plot_cv(self):
		# Determine output image size
		

		# Create a blank image #RGB to BGR 
		background_color = (255, 241, 210) #d2f1ff
		land_color = (203, 242,254) #fef2cb
		image = np.full((self.height, self.width, 3), background_color, dtype=np.uint8)

		#image = np.zeros((height, width, 3), dtype=np.uint8)

		# Define colors for each type
		colors = {"COALNE": (0, 0, 0), "LNDRGN": (255, 0, 0), "BOYCAR": (0, 0, 0)}

		for lid in self.chart.df_chart["Ltype"].unique():
			desc = ""
			l_key = ""

			for k,v in self.chart.Ltype_lut.items():
				if v["id"] == lid:
					desc = v["desc"]
					l_key = k
			color = colors[l_key]
			layer = self.chart.df_chart[self.chart.df_chart["Ltype"] == lid]
			for id in layer["id"].unique():
				layer_obj = layer[layer["id"] == id]
				text = layer_obj["text"].values[0]
				is_point = layer_obj["point"].values[0]

				scaled_coords = []
				for coord in list(zip(layer_obj["x"], layer_obj["y"])):
					x, y = coord
					scaled_coords.append([x, y])
				scaled_coords = np.array([scaled_coords], np.int32)
				
				match layer_obj["Ltype"].values[0]:
					case 1: # COALNE
						#points = np.array([[100, 50], [200, 300], [700, 200], [500, 100]], np.int32)
						points = scaled_coords.reshape((-1, 1, 2))  # Reshape to required shape for fillPoly

						# Define the fill color in BGR format
						#fill_color = (0, 255, 0)  # green

						# Fill the polygon
						cv2.fillPoly(image, [points], land_color)
						cv2.polylines(image, [scaled_coords], isClosed=False, color=color, thickness=1)
					# case 2: # LNDRGN
					# 	bbox_props = dict(boxstyle="square,pad=0.3", fc="none", ec="black", lw=0.5)
					# 	ax.text(float(layer_obj["lon"].iloc[0]), float(layer_obj["lat"].iloc[0]), text, fontsize=6, ha='center', va='center', color='black', bbox=bbox_props)
					case 3: # BOYCAR
						cv2.circle(image, tuple(scaled_coords[0][0]), 5, color, -1)
						#ax.scatter(layer_obj["lon"], layer_obj["lat"], color=colors[l_key]) # ro
		


		# Save the image
		output_path = "chart.png"
		cv2.imwrite(output_path, image)

		# Optionally show the image
		cv2.imshow('Chart', image)
		


class GridManager:
	EARTH_RADIUS = 6371000  # Radius of Earth in meters
	#DEGREES_PER_CELL = 0.00008486082824  # Approximate degrees per 5 meters
	DEGREES_PER_CELL = 0.00001697216565*50  # Approximate degrees per 50 meters
	def __init__(self, chart) -> None:
		self.chart = chart
		self.grid = None
		self.create_grid()

	def degrees_to_cells(self, degrees):
			return int(degrees / self.DEGREES_PER_CELL)
	
	def mark_polygon_on_grid(self, polygons):
			for polygon in polygons:
				# Convert polygon coordinates to grid indices
				pts = np.array([
					(self.degrees_to_cells(lon - self.chart.lon_min), self.degrees_to_cells(self.chart.lat_max - lat)) 
					for lon, lat in polygon
				], np.int32)
				#print(pts)
				# Reshape points for fillPoly
				pts = pts.reshape((-1, 1, 2))

				# Fill the polygon area with 1
				cv2.fillPoly(self.grid, [pts], 1)

	def create_grid(self):
		# Calculate grid dimensions
		lon_cells = self.degrees_to_cells(self.chart.lon_max - self.chart.lon_min)
		lat_cells = self.degrees_to_cells(self.chart.lat_max - self.chart.lat_min)

		# Create the grid matrix initialized to 0
		self.grid = np.zeros((lat_cells, lon_cells), dtype=np.uint8)

		for lid in self.chart.df_chart["Ltype"].unique():
			layer = self.chart.df_chart[self.chart.df_chart["Ltype"] == lid]
			for id in layer["id"].unique():
				layer_obj = layer[layer["id"] == id]
				geometry = list(zip(layer_obj["lon"], layer_obj["lat"]))
				#print(geometry)
				self.mark_polygon_on_grid([geometry])

		#self.grid[-40:, :40] = 1

	def show_cv_grid(self):
		print("Grid size:", self.grid.shape)
		# # Encode image as a JPEG 
		# _, buffer = cv2.imencode('.jpg', grid*255)

		# # Convert to byte array
		# image_bytes = buffer.tobytes()

		# # Display the image
		# display(Image(data=image_bytes))
		cv2.imshow('Grid', self.grid * 255)  # Scale by 255 to see the polygons clearly in the image display
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		

class PathPlanner:
	def __init__(self, gm):
		self.gm = gm
		self.grid = gm.grid
		self.path = None
		self.plan_path()


		# Set up the color map (0water: blue, 1obstacle: black, 2path: yellow, 3start: green, 4end: red)
		colors = [(0, (1.0, 1.0, 1.0)),  #  white
          		(0.25, (0, 0, 0)), # black
				(0.5, (1, 1, 0)), # yellow
				(0.75, (0, 1, 0)),  # green
				(1, (1, 0, 0)),] # red

		# Create a new colormap from the list of colors
		# LinearSegmentedColormap takes a name, a list of colors, and N (the number of discrete colors)
		cmap_name = 'custom_cmap'
		custom_cmap = LinearSegmentedColormap.from_list(cmap_name, colors, N=5)
		self.cmap = custom_cmap
		# print(self.cmap)
		#self.cmap
		#self.cmap
		self.norm = plt.Normalize(0.5, 4.5)

	def plan_path(self):
		# Initialize the path with the starting point
		self.start = (60, 40) # lat, lon
		self.end = (400, 300)
		ts = time()
		path = a_star(self.grid, self.start, self.end)
		print("Path planned in ", time()-ts, " seconds")
		if path is None:
			print("No path found")
		else:
			print("Path found with length", len(path))
			print("path first 10: ", path[:10])
		self.path = path

	def calculate_path_grid(self, path):
		if path is None:
			return None
		# Convert path to a grid for visualization
		path_grid = np.zeros_like(self.grid)
		for point in path:
			path_grid[point[0]][point[1]] = 2  # 2 will represent the path

		# Update the path_grid with obstacles and start/end points
		for i in range(len(self.grid)):
			for j in range(len(self.grid[i])):
				if self.grid[i][j] == 1:
					path_grid[i][j] = 1  # 1 represents obstacles
		path_grid[self.start[0]][self.start[1]] = 3  # 3 represents the start point
		path_grid[self.end[0]][self.end[1]] = 4      # 4 represents the end point
		return path_grid

	
	def plot_path_raw(self, path):
		# Convert the path to a grid for visualization
		path_grid = self.calculate_path_grid(path)
		print(path_grid.shape)
		# Create the plot
		fig, ax = plt.subplots()
		# ax.imshow(path_grid, cmap=self.cmap)
		ax.imshow(path_grid, cmap=self.cmap, norm=self.norm)

		# Add gridlines and set ticks
		#ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=0.1)
		#ax.set_xticks(np.arange(-.5, len(self.grid[0]), 1))
		#ax.set_yticks(np.arange(-.5, len(self.grid), 1))

		# Remove tick labels
		ax.set_xticklabels([])
		ax.set_yticklabels([])

		# Show the plot
		plt.show()
	
	def plot_path_cv(self, path):
		path_grid = self.calculate_path_grid(path)
		height, width = path_grid.shape[0:2]
		# Create a blank image #RGB to BGR
		background_color = (255, 241, 210) #d2f1ff
		land_color = (203, 242,254) #fef2cb
		image = np.full((height, width, 3), background_color, dtype=np.uint8)

		# Define colors for each type
		colors = {1: (203, 242,254), 2: (255, 0, 0), 3: (0, 0, 0), 4: (0, 0, 0)}  # BGR format

		for i in range(height):
			for j in range(width):
				pgv = path_grid[i][j]
				if pgv != 0:
					color = colors[pgv]
					image[i][j] = color

		cv2.imshow('Path', image)

		

	def plot_path_spline(self, path):
		# Assuming the rest of the A* code and path calculation is above...
		path_grid = self.calculate_path_grid(path)
		# Convert the A* path into a series of points
		x = [point[1] for point in path]  # Note: swapping x and y because of grid representation
		y = [point[0] for point in path]

		# Generate parameter t based on path indices
		t = np.arange(len(x))

		# Create a cubic spline interpolation of the path
		cs_x = CubicSpline(t, x, bc_type='natural')
		cs_y = CubicSpline(t, y, bc_type='natural')

		# Generate fine-grained parameter values for a smooth curve
		t_fine = np.linspace(0, t[-1], 300)

		# Evaluate the spline at the fine-grained parameter values
		smooth_x = cs_x(t_fine)
		smooth_y = cs_y(t_fine)

		# Now, let's add this to the visualization code from before
		# Assuming path_grid and other plotting code is already defined...

		fig, ax = plt.subplots()
		ax.imshow(path_grid, cmap=self.cmap, norm=self.norm)

		# Plot the smooth path
		ax.plot(smooth_x, smooth_y, 'm-', linewidth=0.1)  # 'm-' sets the color to magenta

		# Add gridlines and set ticks (same as before)
		ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=0.1)
		ax.set_xticks(np.arange(-.5, len(self.grid[0]), 1))
		ax.set_yticks(np.arange(-.5, len(self.grid), 1))

		# Remove tick labels
		ax.set_xticklabels([])
		ax.set_yticklabels([])

		plt.show()

		

# Load the chart data
# chart_path = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/path_planning/out/SE3DI7L8.000.json"
chart_path = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/path_planning/out/SE2BHS0W.000.json"

chart = ChartLoader(chart_path)
cp = ChartPlotter(chart)
# cp.plot_cv()

gm = GridManager(chart)
# gm.show_cv_grid()
# if cv2.getWindowProperty('Chart', cv2.WND_PROP_VISIBLE) == 0:
# 	print("CV2 window is closed")
# else:
# 	print("CV2 window is open")


pp = PathPlanner(gm)
pp.plot_path_cv(pp.path)

cv2.waitKey(0)
cv2.destroyAllWindows()


# # Determine output image size
# height = 1080
# width = int(height * chart.aspect_ratio)

# # Create a blank image
# image = np.zeros((height, width, 3), dtype=np.uint8)

# # Define colors for each type
# colors = {"COALNE": (0, 165, 255), "LNDRGN": (255, 0, 0), "BOYCAR": (0, 0, 0)}  # BGR format

# # Scale and draw each point
# for layer in chart["layers"]:
# 	key = layer["layer_name"]
# 	color = colors[key]
# 	for feature in layer["features"]:
# 		geometry = feature["geometry"]
# 		scaled_coords = []
# 		for coord in geometry["coordinates"]:
# 			# Scale coordinates to fit the image dimensions
# 			x = int((coord[0] - lon_min) / lon_range * (width - 1))
# 			y = height - int((coord[1] - lat_min) / lat_range * (height - 1))  # Flip y for image coordinates
# 			scaled_coords.append([x, y])
# 		scaled_coords = np.array([scaled_coords], np.int32)
# 		if len(scaled_coords[0]) == 1:  # Check if it's a point
# 			cv2.circle(image, tuple(scaled_coords[0][0]), 5, color, -1)
# 		else:
# 			cv2.polylines(image, [scaled_coords], isClosed=False, color=color, thickness=2)

# Save the image
# output_path = "chart.png"
# cv2.imwrite(output_path, image)

# # Optionally show the image
# cv2.imshow('Chart', image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
