#!/usr/bin/env python
import rospy
from initialize_notask import InitializerNoTask

bla = {}
bla["drift_check"] = 3
bla["countdown"] = 10
rospy.init_node("INITIALIZER")
ini = InitializerNoTask(bla)
ini.start()
rospy.spin()

