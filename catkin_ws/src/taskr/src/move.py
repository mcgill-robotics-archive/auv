#!/usr/bin/env python

import rospy
from math import fabs
from std_msgs.msg import Float64, Bool


class Move(object):
    '''
    Move Action.
    '''

    def __init__(self, data):
        '''
        Constructor for the Move Object.

        Args:
            data: dict of movement commands (distance, heading, depth)
        '''

        # PUBLISHERS/SUBSCRIBERS ===============================================
        # Handle Surge Functions -----------------------------------------------
        self.surge_pub = rospy.Publisher('controls/superimposer/surge',
                                         Float64,
                                         queue_size=1)

        # Handle Depth Functions -----------------------------------------------
        depth_setpoint_topic = rospy.get_param('taskr/depth_setpoint_topic',
                                               default='depth_pid/setpoint')
        self.depth_setpoint_pub = rospy.Publisher(depth_setpoint_topic,
                                                  Float64,
                                                  queue_size=1)
        depth_enable_topic = rospy.get_param('taskr/depth_enable_topic',
                                             default='depth_pid/pid_enable')
        self.depth_enable_pub = rospy.Publisher(depth_enable_topic,
                                                Bool,
                                                queue_size=1)
        self.depth_sub = rospy.Subscriber('state_estimation/depth',
                                          Float64,
                                          self.depth_cb)
        # Note: Used only to send a zero cmd on preempt!
        self.heave_pub = rospy.Publisher('controls/superimposer/heave',
                                         Float64,
                                         queue_size=1)

        # Handle Yaw Functions -------------------------------------------------
        yaw_setpoint_topic = rospy.get_param('taskr/yaw_setpoint_topic',
                                             default='yaw_pid/setpoint')
        self.yaw_setpoint_pub = rospy.Publisher(yaw_setpoint_topic,
                                                Float64,
                                                queue_size=1)
        yaw_enable_topic = rospy.get_param('taskr/yaw_enable_topic',
                                            default='yaw_pid/pid_enable')
        self.yaw_enable_pub = rospy.Publisher(yaw_enable_topic,
                                              Bool,
                                              queue_size=1)
        self.yaw_sub = rospy.Subscriber('state_estimation/yaw',
                                        Float64,
                                        self.yaw_cb)
        # Note: Used only to send a zero cmd on preempt!
        self.yaw_pub = rospy.Publisher('controls/superimposer/yaw',
                                       Float64,
                                       queue_size=1)

        # Gather input from the loaded YAML task file ==========================
        self.distance = data['distance'] if 'distance' in data else 0
        self.depth = data['depth'] if 'depth' in data else None
        self.yaw = data['heading'] if 'heading' in data else None

        # PARAMETERS ===========================================================
        # Velocity Params ------------------------------------------------------
        self.velocity = rospy.get_param('taskr/move/velocity',
                                        default=1)
        self.vel_coeff = rospy.get_param('taskr/move/vel_coeff',
                                         default=1)
        self.vel_cmd_rate = rospy.get_param('taskr/move/vel_cmd_rate',
                                            default=10)

        # Depth Params ---------------------------------------------------------
        self.depth_thresh = rospy.get_param('taskr/move/depth_thresh',
                                            default=0.1)
        self.depth_counts = rospy.get_param('taskr/move/depth_counts',
                                            default=30)
        self.depth_delay = rospy.get_param('taskr/move/depth_delay_s',
                                           default=0.1)

        # Yaw Params -----------------------------------------------------------
        self.yaw_thresh = rospy.get_param('taskr/move/yaw_thresh',
                                          default=0.1)
        self.yaw_counts = rospy.get_param('taskr/move/yaw_counts',
                                          default=30)
        self.yaw_delay = rospy.get_param('taskr/move/yaw_delay_s',
                                         default=0.1)

        # General Params -------------------------------------------------------
        self.post_delay = rospy.get_param('taskr/move/post_delay_s',
                                          default=2)

        # Set Default Behaviour ================================================
        self.yaw_err = None
        self.depth_err = None
        self.preempted = False

        rospy.loginfo("Move initialized with:\n" +
                      "Depth: {}\n".format(self.depth) +
                      "Heading: {}\n".format(self.yaw) +
                      "Distance: {}\n...".format(self.distance));

    def start(self, server, feedback_msg):
        '''
        Do the move action.

        Args:
            server: Action server provided by taskr
            feedback_msg: Action server feedback_msg provided by taskr
        '''

        start = rospy.Time.now()    # Used to track time since initialization
        rospy.loginfo('Starting [Move]...')

        rate = rospy.Rate(self.vel_cmd_rate)                 # Rate in Hz for controls
        time = (fabs(self.distance) / self.velocity) # Est time to travel dist
        surge = self.velocity * self.vel_coeff       # Metric --> Thrust cmd

        # If the distance is negative, we want to go backwards.
        if self.distance < 0:
            surge *= -1         # TODO: check if valid for both directions

        # Set the setpoints before waiting
        # and enable the PIDs (if not already enabled)
        #
        # (This way, depth and yaw will converge concurrently!)
        if self.depth:
            self.depth_setpoint_pub.publish(self.depth)
            self.depth_enable_pub.publish(True)

        if self.yaw:
            self.yaw_setpoint_pub.publish(self.yaw)
            self.yaw_enable_pub.publish(True)

        # First, wait for a stable depth
        if self.depth:
            rospy.loginfo('Going to depth...')
            self.wait_for_depth()
            rospy.loginfo('Depth reached')

        # Next, wait for a stable heading
        if self.yaw:
            rospy.loginfo('Going to heading...')
            self.wait_for_yaw()
            rospy.loginfo('Heading reached')

        # Finally, send surge commands
        '''
        This loop iterates once per frequency tick for the total number
        of frequency ticks it takes to reach the required `time`

        Therefore it should run `rate * time` times
        Ex: 5 seconds @ 10 Hz (cmd/s) --> 50 iterations
        '''
        counts = int(self.vel_cmd_rate * time)
        for i in range(0, counts):
            if self.preempted:
                return

            rospy.loginfo('Surging {}%'.format(float(i) / float(counts)))

            self.surge_pub.publish(surge) # The const thrust cmd est. above
            rate.sleep()

        # Sleep is needed to allow robot to stop before other actions are done.
        rospy.sleep(self.post_delay)

        elapsed = (rospy.Time.now() - start).to_sec()
        rospy.loginfo('Finished [Move] in {}s'.format(elapsed))

    def stop(self):
        '''
        In the event of a mission failure, preempt action.
        '''
        self.preempted = True

        # Disable the controlled PIDs
        self.yaw_enable_pub.publish(False)
        self.depth_enable_pub.publish(False)

        # Send zero commands to all controlled axes
        self.surge_pub.publish(0)
        self.heave_pub.publish(0)
        self.yaw_pub.publish(0)

    def wait_for_depth(self):
        '''
        Sample at a constant rate until depth is within threshold for _ ticks.
        '''
        stable_counts = 0
        while stable_counts < self.depth_counts:
            if self.preempted:
                return

            if self.depth_err is None:
                pass
            elif fabs(self.depth_err) < self.depth_thresh:
                stable_counts += 1
                rospy.loginfo(
                    '{}/{} valid depth periods'.format(stable_counts,
                                                       self.depth_counts))
            else:
                stable_counts = 0
            rospy.sleep(self.depth_delay)

    def wait_for_yaw(self):
        '''
        Sample at a constant rate until yaw is within threshold for _ ticks.
        '''
        stable_counts = 0
        while stable_counts < self.yaw_counts:
            if self.preempted:
                return

            if self.yaw_err is None:
                pass
            elif fabs(self.yaw_err) < self.yaw_thresh:
                stable_counts += 1
                rospy.loginfo(
                    '{}/{} valid yaw periods'.format(stable_counts,
                                                     self.yaw_counts))
            else:
                stable_counts = 0
            rospy.sleep(self.yaw_delay)

    def depth_cb(self, msg):
        '''
        Callback for receiving depth message.
        '''
        # We don't care unless given a setpoint
        if self.depth:
            self.depth_err = (self.depth - msg.data)

    def yaw_cb(self, msg):
        '''
        Callback for receiving yaw message
        '''
        # We don't care unless given a setpoint
        if self.yaw:
            self.yaw_err = (self.yaw - msg.data)
