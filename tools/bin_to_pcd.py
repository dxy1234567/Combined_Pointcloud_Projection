import numpy as np
import open3d as o3d

from others.KITTI.utils.functions import read_pcd_list, get_point_cloud
from others.KITTI.utils.util import print_progress


directory_pcd = "/data/KITTI/data_odometry_velodyne/dataset/sequences/00/velodyne"
path_output = "/root/data/output/pcd/"

pcd_list = read_pcd_list(directory_pcd)

N = len(pcd_list)
for i in range(N):
    path_bin = pcd_list[i]
    pcd = get_point_cloud(path_bin)
    o3d.io.write_point_cloud(path_output, pcd)

    print_progress(i, N)
    