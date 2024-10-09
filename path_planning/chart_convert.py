from osgeo import gdal
import os
import json
# from pprint import pprint
# import numpy as np
#gdal.UseExceptions()


def s57_to_csv(s57_path, out_folder):
	# Specify the path to your S-57 file
	bn = os.path.basename(s57_path)
	# Open the dataset
	dataset = gdal.OpenEx(s57_path, gdal.OF_VECTOR)
	enc_lut = json.load(open("path_planning/enc_lut.json"))
	na_enc_codes = open("path_planning/na_enc_codes.txt", "w")

	if dataset is None:
		print('Unable to open the file')
	else:
		
		print('File opened successfully')
		json_data = {"chart_file": s57_path, "layers": []}

		#all_types = []
		#all_layers = {}
		top_lat_min, top_lat_max = 1000.0, -1000.0
		top_lon_min, top_lon_max = 1000.0, -1000.0

		# Loop through the layers in the S-57 fi
		for i in range(dataset.GetLayerCount()):
			layer_Data = {}
			layer = dataset.GetLayerByIndex(i)
			layer_name = layer.GetName()

			#calculated variables
			found_layer_coords = False
			layer_lat_min, layer_lat_max = 1000.0, -1000.0
			layer_lon_min, layer_lon_max = 1000.0, -1000.0
		
			#print('Layer:', layer_name)
			# all_layers[layer_name] = {"area": False, "point":False, "line":False, "desc": ""}
			# Add the layer name to the JSON data
			layer_Data["layer_name"] = layer.GetName()
			layer_Data["feature_count"] = layer.GetFeatureCount()
			if layer_name not in enc_lut.keys():
				na_enc_codes.write(layer_name + "\n")
				print("Layername:", layer_name, "not in enc_lut. Skipping")
				continue
			layer_Data["layer_desc"] = enc_lut[layer_name]["desc"]
			layer_Data["layer_isArea"] = enc_lut[layer_name]["area"]
			layer_Data["layer_isPoint"] = enc_lut[layer_name]["point"]
			layer_Data["layer_isLine"] = enc_lut[layer_name]["line"]
			layer_Data["features"] = []
			# use lookup to get the layer name
			#print('Layer:', layer.GetName())
			#print('Feature count:', layer.GetFeatureCount())

			# Fetch and print each feature
			for i, feature in enumerate(layer):
				found_feature_coords = False
				lat_min, lat_max = 1000.0, -1000.0
				lon_min, lon_max = 1000.0, -1000.0
				#print("Feature", i)
				data = feature.ExportToJson(as_object=True)  # Or use .ExportToString() if you prefer WKT
				feature_data = {"id": data["id"], "type": data["type"], 
					"properties": data["properties"], "geometry": None}
				#print("keys:", data.keys())

				if data["geometry"]:
					feature_data["geometry"] = data["geometry"]
					geom = data["geometry"]["coordinates"]
					# Logic to find the bbox of the chart file
					if not found_feature_coords:
						found_feature_coords = True
						if not found_layer_coords:
							found_layer_coords = True
					if type(geom[0]) == float: # point
						if geom[0] < lon_min:
							lon_min = geom[0]
						if geom[0] > lon_max:
							lon_max = geom[0]

						if geom[1] < lat_min:
							lat_min = geom[1]
						if geom[1] > lat_max:
							lat_max = geom[1]
					else:
						for g1 in geom:
							if type(g1[0]) == float: # point
								if g1[0] < lon_min:
									lon_min = g1[0]
								if g1[0] > lon_max:
									lon_max = g1[0]

								if g1[1] < lat_min:
									lat_min = g1[1]
								if g1[1] > lat_max:
									lat_max = g1[1]
							else:
								for g2 in g1:
									if g2[0] < lon_min:
										lon_min = g2[0]
									if g2[0] > lon_max:
										lon_max = g2[0]

									if g2[1] < lat_min:
										lat_min = g2[1]
									if g2[1] > lat_max:
										lat_max = g2[1]

				if found_feature_coords:
					feature_data["lat_bounds"] = [lat_min, lat_max]
					feature_data["lon_bounds"] = [lon_min, lon_max]
				else:
					feature_data["lat_bounds"] = None
					feature_data["lon_bounds"] = None


				if lon_min < layer_lon_min:
					layer_lon_min = lon_min
				if lon_max > layer_lon_max:
					layer_lon_max = lon_max
				if lat_min < layer_lat_min:
					layer_lat_min = lat_min
				if lat_max > layer_lat_max:
					layer_lat_max = lat_max



				# if data["geometry"]["type"] not in all_types:
				# 	all_types.append(data["geometry"]["type"])
				# if data["geometry"]["type"] == "Polygon":
				# 	pprint(data["geometry"]["coordinates"][0][:2])
				# else:
				# 	print(data["geometry"]["type"])
				# 	print(data["geometry"]["coordinates"])
				#break
				
					
				#print("======================")
				layer_Data["features"].append(feature_data)
			
			if found_layer_coords:
				layer_Data["lat_bounds"] = [layer_lat_min, layer_lat_max]
				layer_Data["lon_bounds"] = [layer_lon_min, layer_lon_max]
			else:
				layer_Data["lat_bounds"] = None
				layer_Data["lon_bounds"] = None

			if layer_lon_min < top_lon_min:
				top_lon_min = layer_lon_min
			if layer_lon_max > top_lon_max:
				top_lon_max = layer_lon_max
			if layer_lat_min < top_lat_min:
				top_lat_min = layer_lat_min
			if layer_lat_max > top_lat_max:
				top_lat_max = layer_lat_max

			json_data["layers"].append(layer_Data)
		
		json_data["lat_bounds"] = [top_lat_min, top_lat_max]
		json_data["lon_bounds"] = [top_lon_min, top_lon_max]

		#print(all_types)
		# pprint(all_layers)

		with open(os.path.join(out_folder, bn+".json"), "w") as js_out:
			js_out.write(json.dumps(json_data, indent=4))

	na_enc_codes.close()

def all_s57(chart_folder):
	#s57_folder = 'path_planning/chart_files/'
	out_folder = "path_planning/out/"

	chart_files = os.listdir(chart_folder)
	print(chart_files)
	for chart in chart_files:
		if chart.split(".")[-1] == "000" and chart != ".DS_Store":
			print(chart)
			s57_to_csv(os.path.join(chart_folder, chart), out_folder)
		#break
	#print(chart_files)

all_s57("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/path_planning/chart_files")
#s57_to_csv("path_planning/chart_files/SE3DI7L8.000", "path_planning/out/")

