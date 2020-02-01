#!/usr/bin/env python

import rospy
import time
import bitstring as bitstring
import serial as serial
from Ensemble.msg import Ensemble
from serializer import deserialize


class EZ_DVL(object):
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
            port=port, baudrate=baudrate, timeout=timeout)

        uninitialized = True

        while uninitialized:
            conn.sendBreak(0.3)
            conn.write("PD5/n")
            conn.write("EX1111/n")
            conn.write("CS/n")
            if conn.read() == "":
                time.sleep(0.1)
            else:
                uninitialized = False

        running = True

        while running:
            if conn.read() == chr(0x7D):
                rawPacket = conn.read(88)
                Ensemble = _get_ensemble(rawPacket)
                pub.publish(Ensemble)
            if rospy.is_shutdown():
                conn.close()
                break
