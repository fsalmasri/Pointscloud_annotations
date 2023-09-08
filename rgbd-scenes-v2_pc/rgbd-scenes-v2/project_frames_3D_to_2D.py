import os

import matplotlib.pyplot as plt
import numpy as np
from plyfile import PlyData, PlyElement
import open3d as o3d
from scipy.spatial.transform import Rotation as R

from PIL import Image
from skimage.morphology import disk
from skimage.morphology import closing, remove_small_objects
from skimage.measure import label


from utils import *

posfile = 'pc/01.pose'
lblfile = 'pc/01.label'
pcloudfile = 'pc/01.ply'


pcd = o3d.io.read_point_cloud(pcloudfile)
point_cloud = np.asarray(pcd.points)
camera_poses = np.loadtxt(posfile)
lbls = np.loadtxt(lblfile)[1:]



# Construct the transformation matrix for the global origin using First frame's camera pose
# T_global_origin = get_extrinsic_matrix(camera_poses[0])

# Construct the estimated intrinsic matrix
K = get_estimated_intrinsic_matrix(640, 480)

print('points in the cloud: ', point_cloud.shape)
print('points label: ', lbls.shape)
print('frames pos: ', camera_poses.shape)

# Get the 3D points as a Nx4 array
points_3d = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))

def get_fname(i):
    return "0" * (7 - len(str(1 + i * 5))) + str(1 + i * 5) + "-"

flst = os.listdir('/home/feras/Desktop/rgbd-scenes-v2_imgs/rgbd-scenes-v2/imgs/scene_01')
flst = [f'{get_fname(i)}{f}' for i, f in enumerate(flst)]

objects = {1: 'bowl', 2: 'cap', 3: 'cereal_box', 4: 'coffee_mug', 5: 'coffee_table',
           6: 'office_chair', 7: 'soda_can', 8: 'sofa', 9: 'table', 10: 'background'}
objarr = []
obj = {}
for key in objects:
    obj["name"]= f'{objects[key]}: key'
    objarr.append(obj)


data = {}
for i, frame_pos in enumerate(camera_poses):
    T_frame = get_extrinsic_matrix(frame_pos)

    # T_relative = np.linalg.inv(T_global_origin) @ T_frame

    # Transform all 3D points to global coordinates using T_relative
    global_points = np.dot(T_frame, points_3d.T).T[:, :3]

    print(K.shape, global_points.shape)
    # Project all global coordinates to 2D image coordinates
    uvw = np.dot(K, global_points.T)
    uv = (uvw[:2] / uvw[2]).T

    frame_coords_idx = []
    im_frame = np.zeros((480, 640), dtype=np.uint8)
    for uv_idx, point in enumerate(uv):
        x, y = map(int, point)
        if 0 <= x < 640 and 0 <= y < 480:
            im_frame[y, x] = int(lbls[uv_idx])  # Set pixel to white
            frame_coords_idx.append(uv_idx)

    frame_coords = uv[frame_coords_idx]
    # frame_mask = lbls[frame_coords_idx]

    oimage = Image.open('/home/feras/Desktop/rgbd-scenes-v2_imgs/rgbd-scenes-v2/imgs/scene_01/00000-color.png')

    imgs = []
    imgs_lbls = []
    uniques  = np.unique(im_frame)
    frame_polygons = {}
    frame_polygons_arr = []
    for un in uniques[1:-1]:
        mask = np.zeros_like(im_frame).astype(bool)
        idx = np.where(im_frame == un)

        mask[idx] = True

        footprint = disk(6)
        mask = closing(mask, footprint)

        fmask = remove_small_objects(mask, 30)
        label_image = label(fmask)

        lbl_uniqe = np.unique(label_image)

        for lblidx in lbl_uniqe[1:]:
            tmp_im = np.zeros_like(label_image)
            tmp_im[label_image==lblidx] = 1
            imgs.append(tmp_im)
            imgs_lbls.append(un)

    object_polygon_arr = []
    for imidx, im in enumerate(imgs):
        # plt.imshow(im)
        # plt.show()
        coords = np.where(im == 1)
        coords = [[coords[1][i],coords[0][i]] for i, c in enumerate(coords[0])]

        result = convex_hull(coords)
        result = np.vstack((result, result[0]))

        xs, ys = zip(*result)

        object_polygon = {}
        object_polygon["x"] = str(list(xs))
        object_polygon["y"] = str(list(ys))
        object_polygon["XYZ"] = str(list([]))
        object_polygon["object"] = str(imgs_lbls[imidx])
        object_polygon_arr.append(object_polygon)

    frame_polygons['polygon'] = object_polygon_arr
    frame_polygons_arr.append(frame_polygons)

data["name"] = "scene_01"
data["date"] = ""
data["frames"] = frame_polygons_arr



data['objects'] = objarr
data['extrinsics'] = "textfile.txt"
data['conflictList'] = []

fileNum = "0" * (7 - len(str(1 + i * 5))) + str(1 + i * 5) + "-"
data['fileList'] = flst


import json
print(data)
with open('data.json', 'w') as f:
    json.dump(data, f)



        # plt.subplot(121)
        # plt.imshow(oimage)
        # plt.plot(xs, ys)
        # plt.subplot(122)
        # plt.imshow(im)
        # if imgs_lbls[imidx] in objects.keys():
        #     plt.title(f'{imgs_lbls[imidx]}: {objects[imgs_lbls[imidx]]}')
        # plt.show()




