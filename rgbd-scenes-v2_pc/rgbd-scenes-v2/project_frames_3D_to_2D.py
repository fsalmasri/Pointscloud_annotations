import os
import json
from os.path import join

import matplotlib.pyplot as plt
import numpy as np
from plyfile import PlyData, PlyElement
import open3d as o3d
from scipy.spatial.transform import Rotation as R

from PIL import Image


from utils import *

def projection(main_folder, fname, annot_files):

    print(fname)
    for an in annot_files:
        if 'label' in an: lblfile = an
        elif 'pose' in an: posfile = an
        elif 'ply' in an: pcloudfile = an
        else:
            exit('Error in annotation file names')


    pcd = o3d.io.read_point_cloud(pcloudfile)
    point_cloud = np.asarray(pcd.points)
    camera_poses = np.loadtxt(posfile)
    lbls = np.loadtxt(lblfile)[1:]


    # Construct the transformation matrix for the global origin using First frame's camera pose
    T_global_origin = get_extrinsic_matrix(camera_poses[0])

    # Construct the estimated intrinsic matrix
    K = get_estimated_intrinsic_matrix(640, 480)
    np.savetxt(join(main_folder, 'intrinsics.txt'), K)

    print('points in the cloud: ', point_cloud.shape)
    print('points label: ', lbls.shape)
    print('frames pos: ', camera_poses.shape)


    object_categories = {1: 'bowl', 2: 'cap', 3: 'cereal_box', 4: 'coffee_mug', 5: 'coffee_table',
               6: 'office_chair', 7: 'soda_can', 8: 'sofa', 9: 'table', 10: 'background'}

    flst = os.listdir(join(main_folder, 'image'))
    # flst = [f'{get_fname(i)}{f}' for i, f in enumerate(flst)]


    data = {}
    frame_polygons_arr = []
    mainObjects = []

    # Sample 20 frames within the scene
    nFrame_samples = 20
    index = np.random.choice(camera_poses.shape[0], nFrame_samples, replace=False)
    camera_poses = camera_poses[index]
    flst = np.array(flst)[index]

    extrinsics = []
    for i, frame_pos in enumerate(camera_poses):

        oimage = Image.open(join(main_folder, 'image', flst[i]))
        # Map 3D points to 2D.
        uv, uvw, extrinsic = project_3d_2d(point_cloud, frame_pos, T_global_origin, K)
        extrinsics.append(extrinsic[:3, :])

        # Construct mask, xy and xyz frames.
        im_frame, xyz_frame, xy_frame, test_frame = construct_frames(uv, point_cloud, lbls, oimage)


        imgs, imgs_lbls = clean_mask(im_frame, oimage)



        object_polygon_arr = []
        for imidx, im in enumerate(imgs):
            # plt.imshow(im)
            # plt.show()
            coords = np.where(im == 1)
            coords = [[coords[1][i],coords[0][i]] for i, c in enumerate(coords[0])]

            result = convex_hull(coords)
            result = np.vstack((result, result[0]))

            xs_idx, ys_idx = zip(*result)
            XY = xy_frame[ys_idx, xs_idx, :]
            if 0 in XY:
                # XY = interpolate_zeros(XY)
                XY = replace_zeroRow_previousRow(XY)
            xs, ys = XY[:, 0], XY[:, 1]

            # reconstructed_points =[]
            # for ux in XY:
            #     normalized_coordinates = np.dot(np.linalg.inv(K), np.hstack((ux, 1)))
            #     relative_matrix = np.linalg.inv(extrinsic)
            #     rotation_matrix = relative_matrix[:3, :3]
            #     translation_vector = relative_matrix[:3, 3]
            #
            #     camera_coordinates = np.dot(rotation_matrix, normalized_coordinates - translation_vector)
            #
            #     reconstructed_points.append(camera_coordinates)
            #
            # print(np.array(reconstructed_points).shape)
            # exit()

            XYZ = xyz_frame[ys_idx, xs_idx, :]
            if 0 in XYZ:
                XYZ = replace_zeroRow_previousRow(XYZ)

            cur_obj_name = object_categories[int(imgs_lbls[imidx])]

            matching_dicts = {}
            for cur_objDict in mainObjects:
                if cur_obj_name in cur_objDict['name']:
                    matching_dicts = cur_objDict

            if not matching_dicts:
                mainObjects.append({'name': f'{cur_obj_name} 1'})
            else:
                mainObjects.append({'name': f'{cur_obj_name} {int(matching_dicts["name"][-1])+1}'})


            object_polygon = {}
            object_polygon["x"] = xs.tolist()
            object_polygon["y"] = ys.tolist()
            object_polygon["XYZ"] = XYZ.tolist()
            object_polygon["object"] = len(mainObjects)-1 #int(imgs_lbls[imidx])
            object_polygon_arr.append(object_polygon)

        frame_polygons = {}
        frame_polygons['polygon'] = object_polygon_arr
        frame_polygons_arr.append(frame_polygons)

        # plt.imshow(oimage)
        # for f in object_polygon_arr:
        #     print(f)
        #     x = f['x']
        #     y = f['y']
        #     plt.plot(x, y)
        # plt.show()
        # exit()

    data["date"] = ""
    data["name"] = "scene_01"
    data["frames"] = frame_polygons_arr
    data['objects'] = mainObjects
    data['extrinsics'] = "extrinsics.txt"
    data['conflictList'] = []
    data['fileList'] = flst.tolist()

    extrinsics = np.asarray(extrinsics)
    extrinsics = extrinsics.reshape(extrinsics.shape[0], -1)
    np.savetxt(join(main_folder, 'extrinsics\extrinsics.txt'), extrinsics)

    with open(join(main_folder, f'{fname}.json'), 'w') as f:
        json.dump(data, f,  indent=4)

