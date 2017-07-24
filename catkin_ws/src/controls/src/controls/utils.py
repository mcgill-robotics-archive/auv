import math


def normalize_angle(angle):
    """
    normalizes an angle to the interval (-pi, pi]

    Args:
        angle (float): an angle in radians

    Returns:
        float: the angle normalized onto (-pi, pi]
    """
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        angle -= 2 * math.pi
    return angle
