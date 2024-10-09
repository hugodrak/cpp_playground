from rosbags.typesys import get_types_from_msg, register_types
from rosbags.highlevel import AnyReader
from pathlib import Path
import pandas as pd
from pprint import pprint

class LogReader:
	def __init__(self, path):
		self.path = path


		OBJ_MSG = """
uint32 time
uint16 object_id
uint8 status
uint8 classification
uint32 score
float32 velocity
float32 heading
float32 latitude
float32 longitude
float32[4] bbox
"""

		TOPIC = "/marv/sys/nav/tracked_object"
		MSG_TYPE = "tracker_msgs/TrackedObject"
		register_types(get_types_from_msg(OBJ_MSG, MSG_TYPE))
		self.reader = AnyReader([Path(self.path)])
		self.reader.open()
		self.print_meta(self.reader)
		self.connections = [x for x in self.reader.connections if x.topic == TOPIC]
		self.iterator = self.reader.messages(connections=self.connections)

	def print_meta(self, reader):
		swe_tz = 3600000000000
		dur = str(round(reader.duration*10**(-9), 2))
		st = pd.to_datetime(reader.start_time+swe_tz, unit='ns').strftime('%H:%M:%S')
		et = pd.to_datetime(reader.end_time+swe_tz, unit='ns').strftime('%H:%M:%S')
		mc = str(reader.message_count)
		topics = sorted([f"{('('+str(v.msgcount)+')').rjust(8)} {k}" for k,v in reader.topics.items()], key=lambda x: x.split()[1])
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
		if connection.msgtype == 'tracker_msgs/msg/TrackedObject':
			tobj = {"id": msg.object_id, "status": msg.status, "classification": msg.classification, 
		   "score": msg.score, "velocity": msg.velocity, "heading": msg.heading, "latitude": msg.latitude, 
		   "longitude": msg.longitude}
			return timestamp, tobj

	

if __name__ == "__main__":
	p= "/Users/hugodrak/Documents/chalmers/1_kandarb_EENX16/CASE/cpp_playground/logs/april/rosbag2_2024_04_12-11_07_24"
	log = LogReader(p)
	for i in range(10):
		ts, tobj = next(log)
		pprint(tobj)



	log.reader.close()
		