import open3d as o3d
import os
import numpy as np
import sys
sys.path.append(".")

from utils.functions import read_odom, odom_to_T_r, read_pcd_list, read_image_list


directory_pcd = "/home/cjs/rosbag/2024-09-27/gml_2024-09-27-17-10-28/_hesai_pandar"
directory_image = "/home/cjs/rosbag/2024-09-27/gml_2024-09-27-17-10-28/_camera_infra1_image_rect_raw"
path_odom = "/home/cjs/rosbag/2024-09-27/gml_2024-09-27-17-10-28/odom_noggo_gml_2024-09-27-17-10-28.txt"

path_output = "/home/cjs/rosbag/2024-09-27/gml_2024-09-27-17-10-28/output"

odom_lists = read_odom(path_odom)

pcd_lists = read_pcd_list(directory_pcd)

image_lists = read_image_list(directory_image)

start_index = 100
N = min(len(pcd_lists), len(image_lists))

for i in range(start_index, N - 6):
    combined_pcd = o3d.geometry.PointCloud()
    filename = os.path.basename(pcd_lists[i])

    for j in range(i - 5, i + 6):
        
        pcd = o3d.io.read_point_cloud(pcd_lists[j])

        T, _, _ = odom_to_T_r(odom_lists, j)

        pcd.transform(T)

        combined_pcd += pcd

    path = os.path.join(path_output, filename)
    o3d.io.write_point_cloud(path, combined_pcd)







