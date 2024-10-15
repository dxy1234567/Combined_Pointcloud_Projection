import os
import numpy as np

def read_calib(path_file):
    """
        读取KITTI的外参信息，读取左相机。
    """
    with open(path_file, 'r') as file:
        for line in file:
            if line.startswith("P0"):
                key, values = line.split(":")
                values = values.split()
                projections_list = [float(v) for v in values]   # 投影矩阵 
            elif line.startswith("Tr"):
                key, values = line.split(":")
                values = values.split()
                trans_list = [float(v) for v in values]         # 变换矩阵

    K = np.zeros((3, 3))
    K[0, :3] = projections_list[0:3]
    K[1, :3] = projections_list[4:7]
    K[2, :3] = projections_list[8:11]

    R = np.zeros((3, 3))
    R[0, :3] = trans_list[0:3]
    R[1, :3] = trans_list[4:7]
    R[2, :3] = trans_list[8:11]

    t = np.zeros((3, 1))
    t[0, 0] = trans_list[3]
    t[1, 0] = trans_list[7]
    t[2, 0] = trans_list[11]

    return K, R, t

path_file = "calib.txt"

read_calib(path_file)