import rospy

import tf

from tf.transformations import euler_from_quaternion


def get_transform(origin_frame, target_frame):
    listener = tf.TransformListener()
    (trans, rot) = listener.lookupTransform(
        # FROM
        origin_frame,
        # TO
        target_frame,
        # NOW
        rospy.Time())
    return (trans, rot)


def get_yaw_and_depth():
    (trans, rot) = get_transform("/initial_horizon", "/robot")
    # Use static frame, RPY
    angles_estimated = list(euler_from_quaternion(rot, axes='sxyz'))
    return (angles_estimated[2], trans[2])
