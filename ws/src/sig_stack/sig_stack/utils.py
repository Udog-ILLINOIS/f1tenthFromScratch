import math

def yaw_from_quaternion( w, x, y, z):
    # Quaternion to yaw (rotation around Z axis). Result is in [-pi, pi].
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def quaternion_from_yaw(yaw):
    # Yaw (rotation around Z axis) to Quaternion
    w = math.cos(yaw/2)
    x = 0
    y = 0
    z = math.sin(yaw/2)
    return (w, x, y, z)