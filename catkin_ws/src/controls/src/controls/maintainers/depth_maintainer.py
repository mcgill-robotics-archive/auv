#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

from controls.utils import AsyncServoController, PID, trans_gains


class DepthMaintainer(AsyncServoController):
    def __init__(self, setpoint=None):
        pid = PID(*trans_gains['heave'])

        pub = rospy.Publisher('controls/superimposer/heave',
                              Float64,
                              queue_size=1)
        self.error = None

        if setpoint is None:
            setpoint = rospy.wait_for_message('state_estimation/depth',
                                              Float64,
                                              timeout=5.0).data

        super(DepthMaintainer, self).__init__(pid,
                                              pub,
                                              'state_estimation/depth',
                                              Float64,
                                              setpoint)

    def get_error(self, msg):
        self.error = self.setpoint - msg.data
        return self.error

    def setpoint_cb(self, msg):
        if (msg is None):
            return

        # Do not update with a negative depth
        if (msg.data < 0):
            rospy.logdebug('Received negative depth')
            self.stop()
            return

        if (self.active is False):
            self.start()

        self.set_setpoint(msg.data)


if __name__ == "__main__":
    rospy.init_node('depth_maintainer')
    maintainer = DepthMaintainer()

    sub = rospy.Subscriber('controls/depth', Float64, maintainer.setpoint_cb)

    rospy.loginfo('Depth maintainer node started.')
    rospy.spin()
