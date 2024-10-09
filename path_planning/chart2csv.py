import json
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

chart = json.load(open("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/path_planning/out/SE3DI7L8.000.json"))
#chart = json.load(open("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/path_planning/out/SE3DI7LA.000.json"))
# TODO: chrashes for big charts. baybe do somthing smart with nodes so that nly obstacles gets noded?
#or say that 1x1 m nodes is waht we want


coastlines = []

# 1: coastline
# 2: LNDARE
Ltype_lut = {"COALNE": {"id": 1, "desc": "Coastline"}, "LNDRGN": {"id":2, "desc": "Land Region"}, 
			 "BOYCAR": {"id":3, "desc": "Boy"}}

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
			df = pd.DataFrame({"Ltype": Ltype_lut[key]["id"], "id": id, "lat": lat, "lon": lon, "text": None, "point":False})
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

			df = pd.DataFrame({"Ltype": Ltype_lut[key]["id"], "id": id, "lat": lat, "lon": lon, "text": objname, "point":True})
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

			df = pd.DataFrame({"Ltype": Ltype_lut[key]["id"], "id": id, "lat": lat, "lon": lon, "text": None, "point":True})
			#print(df)
			if tdf.empty:
				tdf = df
			else:
				tdf = pd.concat([tdf, df], ignore_index=True)

i = 1

lat_min = tdf["lat"].min()
lat_max = tdf["lat"].max()
lat_mid = (lat_max-lat_min)/2
lat_diff = np.abs(lat_max-lat_min)

lon_min = tdf["lon"].min()
lon_max = tdf["lon"].max()
lon_diff = np.abs(lon_max-lon_min)

w=1/math.cos(math.radians(lat_mid)) # lat center is 60
#plt_area=[0,w,59.5,60.5] #square area

fig = plt.figure()
ax = fig.add_subplot(111)

#ax.imshow(a)

plt.grid(False)
#ax.axis(plt_area)
fig   = plt.gcf()
fig.set_size_inches(6,8)
ax.set_aspect(w)
fig.subplots_adjust(left=0, right=1, bottom=0.05, top=1)


lat_pad=0.05
lon_pad=0.1

ax.set_xlim(lon_min-lon_diff*lon_pad, lon_max+lon_diff*lon_pad)
ax.set_ylim(lat_min-lat_diff*lat_pad, lat_max+lat_diff*lat_pad)
colors = {"COALNE": "orange", "LNDRGN": "blue", "BOYCAR": "black"}
for lid in tdf["Ltype"].unique():
	desc = ""
	l_key = ""
	for k,v in Ltype_lut.items():
		if v["id"] == lid:
			desc = v["desc"]
			l_key = k
	layer = tdf[tdf["Ltype"] == lid]
	for id in layer["id"].unique():
		layer_obj = layer[layer["id"] == id]
		text = layer_obj["text"].values[0]
		is_point = layer_obj["point"].values[0]
		
		match layer_obj["Ltype"].values[0]:
			case 1:
				ax.plot(layer_obj["lon"], layer_obj["lat"], color=colors[l_key])
			case 2:
				bbox_props = dict(boxstyle="square,pad=0.3", fc="none", ec="black", lw=0.5)
				ax.text(float(layer_obj["lon"].iloc[0]), float(layer_obj["lat"].iloc[0]), text, fontsize=6, ha='center', va='center', color='black', bbox=bbox_props)
			case 3:
				ax.scatter(layer_obj["lon"], layer_obj["lat"], color=colors[l_key]) # ro
		

	# todo: add text
plt.show()
