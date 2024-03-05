from rosbags.typesys import get_types_from_msg, register_types
from rosbags.highlevel import AnyReader
from pathlib import Path
import pandas as pd

class LogReader:
	def __init__(self, path):
		self.path = path


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
		self.reader = AnyReader([Path(self.path)])
		self.print_meta(self.reader)
		self.connections = [x for x in self.reader.connections if x.topic == TOPIC]
		self.iterator = self.reader.messages(connections=self.connections)

	def print_meta(self, reader):
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

	def __next__(self):
		connection, timestamp, rawdata = self.iterator.__next__()
		msg = self.reader.deserialize(rawdata, connection.msgtype)
		return timestamp, msg.x_indices, msg.y_indices

	

if __name__ == "__main__":
	log = LogReader("/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/5-2/rosbag2_2024_02_16-14_54_56")
	for i in range(2):
		print(next(log))
		