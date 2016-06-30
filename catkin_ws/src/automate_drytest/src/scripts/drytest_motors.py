#!/usr/bin/env python

import rospy
from auv_msgs.msg import MotorCommands

motor_topic = '/electrical_interface/motor'
thrusters = ['port_surge', 'starboard_surge', 'bow_sway', 'stern_sway', 
             'port_bow_heave', 'starboard_bow_heave', 'port_stern_heave', 
             'starboard_stern_heave'] 


#depth_sensor_topic = 'depth'
#def drytest_depth_sensor():
   # rospy.init_node('drytest_depth_sensor')
   # pub_depth = rospy.Publisher('depth')



def create_command(thr):
    """ Create the command based on the thruster input.
        Args:
            thr : Thruster name
    """
    cmd = MotorCommands()
    cmd.port_surge            = 100 if thr == thrusters[0] else 0
    cmd.starboard_surge       = 100 if thr == thrusters[1] else 0
    cmd.bow_sway              = 100 if thr == thrusters[2] else 0
    cmd.stern_sway            = 100 if thr == thrusters[3] else 0
    cmd.port_bow_heave        = 100 if thr == thrusters[4] else 0
    cmd.starboard_bow_heave   = 100 if thr == thrusters[5] else 0
    cmd.port_stern_heave      = 100 if thr == thrusters[6] else 0
    cmd.starboard_stern_heave = 100 if thr == thrusters[7] else 0
    return cmd


def drytest_motors():
    """ Handle publisher node to test each AUV thruster sequentially.
    """
    pub = rospy.Publisher(motor_topic, MotorCommands, queue_size=3)
    rospy.init_node('drytest_automater')
    rate = rospy.Rate(20) # Hz
 
    while not rospy.is_shutdown():
        # Test each thruster 1 by 1
        for thr in thrusters:
            print('The {} thruster is the next to be tested'.format(thr))
            confirm = raw_input('Press enter to proceed')
            if confirm == '':
                # 60 itterations at 20 Hz -> ~3 seconds per motor
                for i in range(60):
                    cmd = create_command(thr)
                    pub.publish(cmd)
                    rate.sleep()
            
            # If it's the last thruster then exit the drytest
            if thr == thrusters[-1]: return None

   # try: 
   #     drytest_depth_sensor()
   # except rospy.RosInterruptException:
   #     pass



def start_drytest():
    """ Validate with user whether to begin drytest, and run procedures.
    """  
    Print('Drytest of the robot is about to begin')
    print('Use ctrl+c to halt the Drytest at any time')
    command = srt(raw_input('Begin Drytest? Y/N ')).upper()

    if command != 'Y' and command != '':
        print('Exiting. Maybe Later')
        return None

  #  print('Use ctrl+c to halt motors at any time.')
  # proceed = str(raw_input('Begin Drytest? Y/n ')).upper()
    
  #  if proceed != 'Y' and proceed != '':
  #      print('Exiting')
  #      return None
   
     # Call procedure(s) for drytest
    try:
        drytest_motors()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    start_drytest()

