#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

gyroX = []
gyroY = []
gyroZ = []
    

def gyroCallback(data):
    global gyroX, gyroY, gyroZ
    gyroX.append(data.angular_velocity.x)
    gyroY.append(data.angular_velocity.y)
    gyroZ.append(data.angular_velocity.z)

    

    xOffset = sum(gyroX)/len(gyroX)
    yOffset = sum(gyroY)/len(gyroY)
    zOffset = sum(gyroZ)/len(gyroZ)

    print("x offset: {}\ny offset: {}\nz offset: {}\n"\
        .format(xOffset,yOffset,zOffset))
    
    

def init():
    rospy.init_node('sensor_calibration')
    rospy.Subscriber('imu_data', Imu, gyroCallback)
    global pub
    pub = rospy.Publisher('gyro_offsets', String)
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
