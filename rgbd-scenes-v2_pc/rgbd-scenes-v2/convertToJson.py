import numpy as np
from plyfile import PlyData, PlyElement
import open3d as o3d

plydata = PlyData.read('pc/01.ply')

print(plydata)
# print(plydata)

pcd = o3d.io.read_point_cloud('pc/01.ply')
print(pcd)
print(np.array(pcd.points).shape)

lbl = 'pc/01.label'