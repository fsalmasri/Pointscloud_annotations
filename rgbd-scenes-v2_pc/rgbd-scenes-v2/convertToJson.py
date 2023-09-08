import matplotlib.pyplot as plt
import numpy as np
from plyfile import PlyData, PlyElement
import open3d as o3d
from scipy.spatial.transform import Rotation as R

from utils import *

posfile = 'pc/01.pose'
lbl = 'pc/01.label'
pcloudfile = 'pc/01.ply'


pcd = o3d.io.read_point_cloud(pcloudfile)
point_cloud = np.asarray(pcd.points)
camera_poses = np.loadtxt(posfile)

#First frame's camera pose (which serves as the global origin)
first_frame_pose = camera_poses[0]
quat = first_frame_pose[:4]
translation = first_frame_pose[4:]


idx = 0
pose = poses[idx]
quat = pose[:4]
translation = pose[4:]

# print(quaternion_to_rotation_matrix(quat))
# print(quaternion_rotation_matrix(quat))

R = quaternion_rotation_matrix(quat)

T_global_origin = np.eye(4)
T_global_origin[:3, :3] = R
T_global_origin[:3, 3] = translation

projected_points = []

idx = 1
pose = poses[idx]
quat = pose[:4]
translation = pose[4:]
R = quaternion_to_rotation_matrix(quat)
T_frame = np.eye(4)
T_frame[:3, :3] = R
T_frame[:3, 3] = translation

T_relative = np.linalg.inv(T_global_origin) @ T_frame

fx = 525.0  # Focal length in pixels (typically around 525 for 640x480 images)
fy = 525.0  # Focal length in pixels (typically around 525 for 640x480 images)
cx = 320.0  # Principal point (image center) in pixels (half of 640)
cy = 240.0  # Principal point (image center) in pixels (half of 480)
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

for point in point_cloud:
    # Add homogeneous coordinate
    point_homogeneous = np.append(point, 1)

    # Transform point to global coordinates using T_relative
    global_point = np.dot(T_relative, point_homogeneous)

    # Project global coordinates to 2D image coordinates (as described earlier)
    uvw = np.dot(K, global_point[:3])
    uv = uvw[:2] / uvw[2]

    # Store or visualize the projected point
    projected_points.append(uv)

frame = np.zeros((480, 640), dtype=np.uint8)
for point in projected_points:
    x, y = map(int, point)
    if 0 <= x < 640 and 0 <= y < 480:
        frame[y, x] = 255  # Set pixel to white

plt.imshow(frame)
plt.show()
