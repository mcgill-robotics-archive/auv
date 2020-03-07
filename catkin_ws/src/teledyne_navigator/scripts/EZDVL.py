#!/usr/bin/env python

import rospy
import time
import bitstring as bitstring
import serial as serial
from teledyne_navigator.msg import Ensemble
from teledyne_navigator.serializer import deserialize
import NumPy

def _get_ensemble(rawPacket):
        #Read all ensemble data.
        packet = bitstring.BitStream("0x{:02X}".format(0x7D))
        characters = rawPacket
        for char in characters:
                packet.append("0x{:02X}".format(ord(char)))
	#print("Final length of the packet is {}".format(len(packet)))
	test = deserialize(packet, frame_id)

	return test


if __name__ == "__main__":
        mountingAngle = 45
        rospy.init_node("teledyne_navigator")  # Starts ROS node for this

        port = rospy.get_param("~port")  # hardware settings for DVL
        baudrate = rospy.get_param("~baudrate")
        frame_id = rospy.get_param("~frame")
        timeout = rospy.get_param("~timeout")

        pub = rospy.Publisher("~ensemble", Ensemble, queue_size=1)  # settings for publishing ensemble to ROS

        conn = serial.Serial(
        port=port, baudrate=baudrate)
        #conn= serial.Serial(port= "/dev/dvl",baudrate=115200, write_timeout=0.3)
        uninitialized = True

        while uninitialized:
                conn.send_break(duration = 0.3)
                time.sleep(0.5)
                #print("Sent break")
                reply = conn.read_all()
                #print(reply)
                if reply == "":
                        print("No reply for DVL, waiting")
                        time.sleep(1)
                else:
                        print("got a reply from DVL, starting data read")
                        uninitialized = False

        running = True

        #print("Sending params")

        conn.write("PD5\n")
        time.sleep(0.1)
        conn.read_all()
        conn.write("EX1111\n")
        time.sleep(0.1)
        conn.read_all()
        conn.write("CS\n")
        time.sleep(0.1)
        conn.read_all()

        #print("Starting read/publish loop")
        #conn.flush()
        #print(running)
        time.sleep(1)
        #print(conn.in_waiting)
        #print(conn.read_all())

        while running:
            if (conn.in_waiting >= 88):
	        #print(conn.in_waiting)
                thisByte = conn.read()
                #print(thisByte)
                
                if thisByte == chr(0x7D):
                    rawPacket = conn.read(87)
                    #print("raw packet is: ")
                    #print(rawPacket)
                    
                    Ensemble = _get_ensemble(rawPacket)
                    print("============ BEFORE ROTATION ==============")
                    print(Ensemble.pitch)
                    print(Ensemble.roll)
                    print(Ensemble.heading)
                    print(Ensemble.bottom_translation.x)
                    print(Ensemble.bottom_translation.y)
                    print(Ensemble.bottom_translation.z)
                    print(Ensemble.reference_translation.x)
                    print(Ensemble.reference_translation.y)
                    print(Ensemble.reference_translation.z)

                    #rotate headeing by adjustment amount
                    Ensemble.heading = Ensemble.heading + mountingAngle #rotate this by 45deg

                    # Math for rotation around y axis for bottom translations
                    x1 = Ensemble.bottom_translation.x
                    x2 = Ensemble.bottom_translation.y
                    x3 = Ensemble.bottom_translation.z

                    Ensemble.bottom_translation.x = cos(mountingAngle)*x1 - sin(mountingAngle)*x3
                    Ensemble.bottom_translation.y = x2
                    Ensemble.bottom_translation.z = sin(mountingAngle)*x1 + cos(x3)

                    #Math for rotation around y axis for reference translation
                    x1 = Ensemble.reference_translation.x
                    x2 = Ensemble.reference_translation.y
                    x3 = Ensemble.reference_translation.z

                    Ensemble.reference_translation.x = cos(mountingAngle)*x1 - sin(mountingAngle)*x3
                    Ensemble.reference_translation.y = x2
                    Ensemble.reference_translation.z = sin(mountingAngle)*x1 + cos(x3)

                    #TODO: Clean these multiplications

                    print("============ AFTER ROTATION ==============")
                    print(Ensemble.pitch)
                    print(Ensemble.roll)
                    print(Ensemble.heading)
                    print(Ensemble.bottom_translation.x)
                    print(Ensemble.bottom_translation.y)
                    print(Ensemble.bottom_translation.z)
                    print(Ensemble.reference_translation.x)
                    print(Ensemble.reference_translation.y)
                    print(Ensemble.reference_translation.z)
                    pub.publish(Ensemble)
                else:
                    conn.flush()
                    #print('Something is wrong; Flushed the buffer!')
                    
	    if rospy.is_shutdown():
                conn.close()
                break
