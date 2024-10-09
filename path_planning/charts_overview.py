import json
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

summary = json.load(open("path_planning/summary.json", "r"))
fig, ax = plt.subplots(figsize=(15, 15))
# fig, ax = plt.subplots(size=(15, 15))
bbox_props = dict(boxstyle="square,pad=0.3", fc="none", ec="black", lw=0.5)

for key, chart in summary.items():
	name = key
	# print(name)
	# lat is y
	# lon is x
	lower_left = (chart["lon_bounds"][0], chart["lat_bounds"][0])
	#print(lower_left)
	width = chart["lon_bounds"][1] - chart["lon_bounds"][0]
	height = chart["lat_bounds"][1] - chart["lat_bounds"][0]
	lat_mid = (chart["lat_bounds"][1] + chart["lat_bounds"][0]) / 2
	lon_mid = (chart["lon_bounds"][1] + chart["lon_bounds"][0]) / 2
	#print(width, height)

	# Define the rectangle: (x, y) - lower left corner, width, height
	rectangle = Rectangle(lower_left, width, height, edgecolor='red', fill=False)
	ax.text(lon_mid, lat_mid, name, fontsize=6, ha='center', va='center', color='black', bbox=bbox_props)
	# Add the rectangle to the axes
	ax.add_patch(rectangle)

	# Set the limits of the plot
	

	# Show the plot
ax.set_xlim(11, 12.5)
ax.set_ylim(56.5,59)
plt.show()