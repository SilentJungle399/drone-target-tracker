import json, sys
from pymavlink import mavutil

config = json.load(open("settings.json"))["rpi" if sys.platform == "linux" else "sim"]

def main():
	master = mavutil.mavlink_connection(**config)
	master.wait_heartbeat()
	print("received heartbeat")
	master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
	print("sent request")
	while True:
		msg = master.recv_match(type='ATTITUDE', blocking=True)
		pitch = msg.pitch
		roll = msg.roll
		print('pitch: {0}, roll: {1}'.format(pitch, roll))


if __name__ == '__main__':
	main()
