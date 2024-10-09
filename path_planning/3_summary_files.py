import os
import json

out_folder = "path_planning/out/"
sum_f = open("path_planning/summary.json", "w")
chart_files = os.listdir(out_folder)
print(chart_files)
charts = {}
for chart in chart_files:
	if not chart.endswith(".json"):
		continue
	with open(os.path.join(out_folder, chart)) as f:
		data = json.load(f)
		name = data["chart_file"]
		lat_bounds = data["lat_bounds"]
		lon_bounds = data["lon_bounds"]
		#"chart_file": "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/path_planning/chart_files/SE2BHS0W.000",
		name_short = os.path.basename(name)
		info = {"name": name_short, "lat_bounds": lat_bounds, "lon_bounds": lon_bounds}
		charts[name_short] = info
sum_f.write(json.dumps(charts, indent=4))
	