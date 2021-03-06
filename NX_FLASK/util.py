"""
This file is written for drone control (calculate rotation matrix, geo calculation etc.)
2021/08/23 seungwoo
"""
import math
import numpy as np

def gstreamer_pipeline(
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=60,
    flip_method=0,
    
    # capture_width=3264,
    # capture_height=2464,
    # display_width=3264,
    # display_height=2464,
    # framerate=21,
    # flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def get_intersections(cen0, r0, cen1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1
    x0, y0 = cen0
    x1, y1 = cen1
    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    # non intersecting
    if d > r0 + r1:
        return None
    # One circle within other
    if d < abs(r0 - r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(r0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        inter0 = (x2 + h * (y1 - y0) / d, y2 - h * (x1 - x0) / d)
        inter1 = (x2 - h * (y1 - y0) / d, y2 + h * (x1 - x0) / d)
        return (inter0, inter1)

def get_location_metres(original_location, dxdy):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dxdy[0] / earth_radius * 180 / math.pi
    dLon = dxdy[1] / (earth_radius * math.cos(math.pi * original_location[0] / 180)) * 180 / math.pi

    # New position in decimal degrees
    newlat = original_location[0] + dLat
    newlon = original_location[1] + dLon
    return [newlat, newlon]


def get_metres_location(original_location, other_location):
    """Convert distance from degrees longitude-latitude to meters.
        Takes the two points described by (Lat,Lon) in degrees
        and converts it into distances *dx(north distance)* and *dy(east distance)* in meters.
        returns (float, float)
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLatLon = [other_location[0] - original_location[0], other_location[1] - original_location[1]]
    dx = dLatLon[0] * earth_radius * math.pi / 180
    dy = dLatLon[1] * (earth_radius * math.cos(math.pi * original_location[0] / 180)) * math.pi / 180

    return [dx, dy]


def euler_from_quaternion(orientation):
    """
    :param orientation = client.simGetMultiRotorState().pose.orientation

    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    # Note that the return value is in PRY order NOT RPY
    return roll_x, pitch_y, yaw_z  # in radians


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def quaternion_rotation_matrix(orientation):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    x, y, z, w = orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val
    q0, q1, q2, q3 = w, x, y, z
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


def euler_rotation_matrix(roll, pitch, yaw):
    """
    Generate a full three-dimensional rotation matrix from euler angles

    Input
    :param roll: The roll angle (radians) - Rotation around the x-axis
    :param pitch: The pitch angle (radians) - Rotation around the y-axis
    :param yaw: The yaw angle (radians) - Rotation around the z-axis

    Output
    :return: A 3x3 element matix containing the rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.

    """
    # First row of the rotation matrix
    r00 = np.cos(yaw) * np.cos(pitch)
    r01 = np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll)
    r02 = np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)

    # Second row of the rotation matrix
    r10 = np.sin(yaw) * np.cos(pitch)
    r11 = np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll)
    r12 = np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)

    # Third row of the rotation matrix
    r20 = -np.sin(pitch)
    r21 = np.cos(pitch) * np.sin(roll)
    r22 = np.cos(pitch) * np.cos(roll)

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix

def euler_rotation_matrix_YRP(roll, pitch, yaw):
    """
    Generate a full three-dimensional rotation matrix from euler angles
    Original rotation matix order is YPR. But, because of the gimbal sensor, the order need to be changed to YRP

    Input
    :param roll: The roll angle (radians) - Rotation around the x-axis
    :param pitch: The pitch angle (radians) - Rotation around the y-axis
    :param yaw: The yaw angle (radians) - Rotation around the z-axis

    Output
    :return: A 3x3 element matix containing the rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.

    """
    # First row of the rotation matrix

    r00 = np.cos(yaw) * np.cos(pitch) - np.sin(pitch)*np.sin(roll)*np.sin(yaw)
    r01 = - np.sin(yaw) * np.cos(roll)
    r02 = np.sin(yaw) * np.sin(roll) * np.cos(pitch) + np.cos(yaw) * np.sin(pitch)

    # Second row of the rotation matrix
    r10 = np.sin(pitch) * np.sin(roll)*np.cos(yaw) + np.cos(pitch)*np.sin(yaw)
    r11 = np.cos(yaw) * np.cos(roll)
    r12 = np.sin(yaw) * np.sin(pitch) - np.cos(yaw) * np.sin(roll)* np.cos(pitch)

    # Third row of the rotation matrix
    r20 = -np.sin(pitch)*np.cos(roll)
    r21 = np.sin(roll)
    r22 = np.cos(pitch) * np.cos(roll)

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix
