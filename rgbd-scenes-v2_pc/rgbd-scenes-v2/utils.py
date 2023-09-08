import numpy as np
def quaternion_to_rotation_matrix(quat):
    a, b, c, d = quat
    return np.array([
        [1 - 2*(c**2 + d**2), 2*(b*c - a*d), 2*(a*c + b*d)],
        [2*(b*c + a*d), 1 - 2*(b**2 + d**2), 2*(c*d - a*b)],
        [2*(b*d - a*c), 2*(a*b + c*d), 1 - 2*(b**2 + c**2)]
    ])


def quaternion_rotation_matrix(Q):
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
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

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


def get_extrinsic_matrix(camera_poses):

    quat = camera_poses[:4]
    translation = camera_poses[4:]

    # Convert quaternion to rotation matrix
    R = quaternion_rotation_matrix(quat)

    mx = np.eye(4)
    mx[:3, :3] = R
    mx[:3, 3] = translation

    return mx

def get_estimated_intrinsic_matrix(w, h):
    fx = 525.0  # Focal length in pixels (typically around 525 for 640x480 images)
    fy = 525.0  # Focal length in pixels (typically around 525 for 640x480 images)

    cx = float(w/2)  # Principal point (image center) in pixels (half of 640)
    cy = float(h/2)  # Principal point (image center) in pixels (half of 480)
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    return K


from scipy.spatial import ConvexHull
def convex_hull(coordinates):
    # Convert the list of coordinates to a NumPy array
    points = np.array(coordinates)

    # Compute the convex hull
    hull = ConvexHull(points)

    # Extract the vertices of the convex hull
    convex_hull_points = points[hull.vertices]


    return convex_hull_points

