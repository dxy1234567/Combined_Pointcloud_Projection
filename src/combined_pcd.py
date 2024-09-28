import open3d as o3d
import os
import numpy as np
from util.functions import extract_timestamp, read_odom, odom_to_T_r


directory_pcd = "/home/cjs/rosbag/20240926/2/_hesai_pandar"
directory_image = "/home/cjs/rosbag/20240926/2/_camera_infra1_image_rect_raw"
path_odom = "/home/cjs/rosbag/20240926/odom.txt"

path_output = "/home/cjs/rosbag/20240926/combined_pcd"

odom_lists = read_odom(path_odom)

pcd_lists = []
for file_name in sorted(os.listdir(directory_pcd), key=lambda f: extract_timestamp(f)):
    if file_name.endswith(".pcd"):
        file_path = os.path.join(directory_pcd, file_name)
        pcd_lists.append(file_path)

image_lists = []
for file_name in sorted(os.listdir(directory_image), key=lambda f: extract_timestamp(f)):
    if file_name.endswith(".png"):
        file_path = os.path.join(directory_image, file_name)
        image_lists.append(file_path)

N = min(len(pcd_lists), len(image_lists))

for i in range(10, N):
    combined_pcd = o3d.geometry.PointCloud()
    filename = os.path.basename(pcd_lists[i])

    for j in range(i - 5, i + 600):
        
        pcd = o3d.io.read_point_cloud(pcd_lists[j])

        T, _, _ = odom_to_T_r(odom_lists, j)

        pcd.transform(T)

        combined_pcd += pcd

    path = os.path.join(path_output, filename)
    o3d.io.write_point_cloud(path, combined_pcd)







