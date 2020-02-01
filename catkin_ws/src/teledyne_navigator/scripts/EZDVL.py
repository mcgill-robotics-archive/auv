#!/usr/bin/env python

import rospy
import time
import bitstring as bitstring
import serial as serial
from teledyne_navigator.msg import Ensemble
from teledyne_navigator import serializer

def _get_ensemble(rawPacket):
	# Read all ensemble data.
	packet = bitstring.BitStream("0x{:02X}".format(0x7D))
	characters = rawPacket
	for char in characters:
		packet.append("0x{:02X}".format(ord(char)))

		return deserialize(packet, frame_id)


if __name__ == "__main__":
	rospy.init_node("teledyne_navigator")  # Starts ROS node for this

	port = rospy.get_param("~port")  # hardware settings for DVL
	baudrate = rospy.get_param("~baudrate")
	frame_id = rospy.get_param("~frame")
	timeout = rospy.get_param("~timeout")

	pub = rospy.Publisher("~ensemble", Ensemble, queue_size=1)  # settings for publishing ensemble to ROS

	conn = serial.Serial(
	port=port, baudrate=baudrate, timeout=timeout, write_timeout=0.3)
	#conn= serial.Serial(port= "/dev/dvl",baudrate=115200, write_timeout=0.3)
	uninitialized = True

	while uninitialized:
		conn.send_break(duration = 0.3)
		time.sleep(0.5)
		print("Sent break")
		reply = conn.read_all()
		print(reply)
		if reply == "":
			print("No reply for DVL, waiting")
			time.sleep(1)
		else:
			print("got a reply from DVL, starting data read")
			uninitialized = False

	running = True

	print("Sending params")

	#conn.write("PD5/n")
	#time.sleep(0.1)
	#conn.read_all()
	#conn.write("EX1111/n")
	#time.sleep(0.1)
	#conn.read_all()
	conn.write("CS/n")
	#time.sleep(0.1)
	#conn.read_all()

	print("Starting read/publish loop")
        #conn.flush()
	print(running)
	time.sleep(1)
        print(conn.in_waiting)
	print(conn.read_all())

	while running:
                print(conn.in_waiting)
	        thisByte = conn.read()
	        print(thisByte)
	        if thisByte == chr(0x7D):
                    rawPacket = conn.read(87)
	            print("raw packet is: ")
		    print(rawPacket)
		    Ensemble = _get_ensemble(rawPacket)
		    pub.publish(Ensemble)
		    if rospy.is_shutdown():
		        conn.close()
		        break
