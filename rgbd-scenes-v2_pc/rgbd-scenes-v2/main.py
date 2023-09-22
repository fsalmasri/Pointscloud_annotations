import os
from os.path import join
import shutil

from project_frames_3D_to_2D import projection

dataDir = r'C:\Users\fedai\Desktop\data\rgbd-scenes-v2\data'
pcDir = 'pc'
dirlst = os.listdir(dataDir)


def get_fname(i):
    return "0" * (7 - len(str(1 + i * 5))) + str(1 + i * 5) + "-"

def pre_rpocess_files(main_folder):

    if not os.path.exists(join(main_folder, 'image')):
        os.mkdir(join(main_folder, 'image'))
    if not os.path.exists(join(main_folder, 'depth')):
        os.mkdir(join(main_folder, 'depth'))
    if not os.path.exists(join(main_folder, 'extrinsics')):
        os.mkdir(join(main_folder, 'extrinsics'))

    flst = [f for f in os.listdir(main_folder) if 'png' in f]

    for i, f in enumerate(flst):
        if 'color' in f:
            shutil.move(join(main_folder, f), join(main_folder, 'image', f))  # f'{get_fname(i)}{f}
        elif 'depth' in f:
            shutil.move(join(main_folder, f), join(main_folder, 'depth', f))  # f'{get_fname(i)}{f}'



for subDir in dirlst:
    # pre_rpocess_files(join(dataDir, subDir))
    annot_files = [join(pcDir, s) for s in os.listdir(pcDir) if subDir[-2:] in s]
    projection(join(dataDir, subDir), subDir, annot_files)

    # exit()

