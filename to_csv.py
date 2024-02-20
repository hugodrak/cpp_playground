from hdbscan import HDBSCAN
from sklearn.cluster import DBSCAN
import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.highlevel import AnyReader
from pathlib import Path
import tqdm
import os


RADARSPOKE_MSG = """
uint32 detection_range
uint32 detections
float32[] x_indices
float32[] y_indices
float32[] ranges
uint8[] intensities
bool doppler_active
uint8[] doppler_returns
	"""

TOPIC = "/radar_driver/RadarScanCartesian"
MSG_TYPE = "radar_msgs/RadarScanCartesian"
register_types(get_types_from_msg(RADARSPOKE_MSG, MSG_TYPE))

out_folder = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/out"

LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/5-2/rosbag2_2024_02_20-13_11_18"
# LOGPATH = "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/5-2/rosbag2_2024_02_16-14_54_56"

log_folder = os.path.basename(LOGPATH)
new_dir = os.path.join(out_folder, log_folder)
os.makedirs(new_dir, exist_ok=True)
out_folder = new_dir

N = 10000
ti = 0

def print_meta(reader):
	swe_tz = 3600000000000
	dur = str(round(reader.duration*10**(-9), 2))
	st = pd.to_datetime(reader.start_time+swe_tz, unit='ns').strftime('%H:%M:%S')
	et = pd.to_datetime(reader.end_time+swe_tz, unit='ns').strftime('%H:%M:%S')
	mc = str(reader.message_count)
	topics = sorted([f"{('('+str(v.msgcount)+')').rjust(8)} {k}" for k,v in reader.topics.items()])
	w = max([len(dur), len(et), len(mc), len(st)]+ [len(t)+3 for t in topics])
	print("╔"+"═"*(w+2)+"╗")
	print("║"+" "*((w-6)//2)+"LOG DATA"+" "*((w-5)//2)+"║")
	print("╠"+"═"*(w+2)+"╣")
	print("║"+"Duration: "+dur + " s"+" "*(w-len(dur)-10)+"║")
	print("║"+"Start time: "+st+" "*(w-len(st)-10)+"║")
	print("║"+"End time: "+et+" "*(w-len(et)-8)+"║")
	print("║"+"Message count: "+mc+" "*(w-len(mc)-13)+"║")
	print("║"+"Topics: "+" "*(w-6)+"║")
	for t in topics:
		print("║"+"   "+t+" "*(w-len(t)-1)+"║")
	print("╚"+"═"*(w+2)+"╝")

bdf = pd.DataFrame(columns=["time", "x", "y", "intens"])

with AnyReader([Path(LOGPATH)]) as reader:
	print_meta(reader)
	connections = [x for x in reader.connections if x.topic == TOPIC]
	for connection, timestamp, rawdata in tqdm.tqdm(reader.messages(connections=connections)):
		msg = reader.deserialize(rawdata, connection.msgtype)
		rng = msg.detection_range

		# if not (20<ti<50):
		# 	ti += 1
		# 	continue
		
		data = {"time": "", "x": [], "y": [], "intens": []}
		for x, y, intens in zip(msg.x_indices, msg.y_indices, msg.intensities):
			data["time"] = ti
			data["x"].append(x)
			data["y"].append(y)
			data["intens"].append(intens)

		df = pd.DataFrame(data)
		df["intens"] /= 255.0
		ti += 1

		sdf = df.sample(n=N)

		bdf = pd.concat([bdf, sdf])

	bdf.to_csv(f"logs/{log_folder}.csv", index=False)