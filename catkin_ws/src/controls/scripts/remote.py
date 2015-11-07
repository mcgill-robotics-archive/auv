#!/usr/bin/env python
from math import pi
import rospy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64
from std_msgs.msg import String
from auv_msgs.msg import SetPosition
from auv_msgs.msg import SetVelocity


def setVelocity_callback(data):
    global surgeSpeed, swaySpeed, depth, roll, desired_pitch, desired_yaw, isSettingPosition
    surgeSpeed = data.surgeSpeed
    swaySpeed = data.swaySpeed
    depth = data.depth
    roll = data.roll
    desired_pitch = data.pitch
    desired_yaw = data.yaw

    isSettingPosition = 0




def rosInit():
    rospy.init_node('remote', anonymous=True)

    global wrenchPublisher
    wrenchPublisher = rospy.Publisher("remote/wrench", Wrench, queue_size=100)

if __name__ == '__main__':
    rosInit()

    fx = 0.0
    fy = 0.0
    fz = 0.0

    tx = 0.0
    ty = 0.0
    tz = 0.0


    dt = 0.1
    r = rospy.Rate(1/dt)


    rospy.Subscriber("remote/channels", SetPosition, setPosition_callback)


    while not rospy.is_shutdown():

        wrenchMsg = Wrench()

        wrenchMsg.force.x = fx;
        wrenchMsg.force.y = fy;
        wrenchMsg.force.z = fz;
        wrenchMsg.torque.x = tx;
        wrenchMsg.torque.y = ty;
        wrenchMsg.torque.z = tz;

        wrenchPublisher.publish(wrenchMsg)

        r.sleep()
