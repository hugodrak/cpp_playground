import json
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

chart = json.load(open("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/path_planning/out/SE3DI7L8.000.json"))

coastlines = []

# 1: coastline
# 2: LNDARE
tdf = pd.DataFrame( columns=["type", "id", "lat", "lon"])

for layer in chart["layers"]:
	if layer["layer_name"] == "COALNE":
		coastlines = layer["features"]
		break



for feature in coastlines:
	lat = []
	lon = []
	id = feature["id"]
	idss = []
	geometry = feature["geometry"]
	coordinates = geometry["coordinates"]
	for coord in coordinates:
		lon.append(coord[0])
		lat.append(coord[1])
		idss.append(id)

	df = pd.DataFrame({"id": id, "lat": lat, "lon": lon}, columns=["id", "lat", "lon"])
	tdf = pd.concat([tdf, df], ignore_index=True)
print(tdf)
i = 1



w=1/math.cos(math.radians(57.9)) # lat center is 60
#plt_area=[0,w,59.5,60.5] #square area

a=np.zeros(shape=(300,300))

fig = plt.figure()
ax = fig.add_subplot(111)

#ax.imshow(a)

plt.grid(False)
#ax.axis(plt_area)
fig   = plt.gcf()
fig.set_size_inches(8,8)
ax.set_aspect(w)
fig.subplots_adjust(left=0.1, right=0.9, bottom=0.1, top=0.9)




ax.set_xlim(11.4, 11.6)
ax.set_ylim(57.8, 58.0)
for id in tdf["id"].unique():
	ax.plot(tdf[tdf["id"] == id]["lon"], tdf[tdf["id"] == id]["lat"], label="Coastline")
	i += 1
plt.show()
