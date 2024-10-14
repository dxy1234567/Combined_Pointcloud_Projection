import open3d as o3d
import os
import numpy as np
import sys
sys.path.append(".")

from others.KITTI.utils.functions import get_point_cloud, read_odom, odom_to_T_r, read_pcd_list, read_image_list, read_calibration_file


directory_pcd = "/home/cjs/data/KITTI/data_odometry_velodyne/dataset/sequences/00/velodyne"
directory_image = "/home/cjs/data/KITTI/data_odometry_gray/dataset/sequences/00/image_0"
path_odom = "/home/cjs/data/KITTI/data_odometry_poses/dataset/poses/00.txt"

path_output = "/home/cjs/data/output"

odom_lists = read_odom(path_odom)

pcd_lists = read_pcd_list(directory_pcd)

image_lists = read_image_list(directory_image)

N = min(len(pcd_lists), len(image_lists))

for i in range(5, N - 5):
    combined_pcd = o3d.geometry.PointCloud()
    filename = os.path.basename(pcd_lists[i])

    for j in range(i - 5, i + 6):
        
        pcd = get_point_cloud(pcd_lists[j])

        T, _, _ = odom_to_T_r(odom_lists, j)

        pcd.transform(T)

        combined_pcd += pcd

    path = os.path.join(path_output, filename)
    o3d.io.write_point_cloud(path, combined_pcd)







