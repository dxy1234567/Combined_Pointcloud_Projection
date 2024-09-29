"""
    现在已获得在世界坐标系下的拼接点云，现在要将点云转换到相机坐标系下，并投影。
    World -> XT-16 -> RealSense
"""

import open3d as o3d
import numpy as np
import os
import bisect
import cv2
from util.functions import (read_image_list, read_odom, read_pcd_list, list_to_timestamps,
                            find_closest_timestamp, odom_to_T_r, T_to_r_t)
from util.get_one_combined_pcd import get_one_pcd
from util.pcd2depth import pcd_projection

# directory_combined_pcd = ""   # 直接对拼接好的点云组进行操作

directory_pcd = "/home/cjs/rosbag/20240926/2/_hesai_pandar"
directory_image = "/home/cjs/rosbag/20240926/2/_camera_infra1_image_rect_raw"
path_odom = "/home/cjs/rosbag/20240926/odom.txt"

odom_list = read_odom(path_odom)
pcd_list = read_pcd_list(directory_pcd)
image_list = read_image_list(directory_image)

timestamps_pcd = list_to_timestamps(pcd_list)
timestamps_image = list_to_timestamps(image_list)

# 相机内参
## 
camera_intrinsics= np.float64([[431.574523925781, 0, 429.674438476562],
                            [0, 431.574523925781, 237.917541503906],
                            [0, 0, 1]])

## 畸变参数
dist_coeffs = np.float64([0, 0, 0, 0, 0])

# 外参: XT to RealSense
T_RX = np.array([])

N = min(len(pcd_list), len(image_list))

directory_output = "/home/cjs/rosbag/20240926/projection"
# i表示为XT16时间戳序号（下标）
for i in range(N):
    combine_pcd = get_one_pcd(i)

    stamp_pcd = timestamps_pcd[i]

    # 找到最接近当前雷达时间戳的相机时间戳
    stamp_image, j = find_closest_timestamp(timestamps_image, stamp_pcd)

    T_WX = odom_to_T_r(odom_list, i)
    T_WX_inv = np.linalg.inv(T_WX)

    T_RW = T_RX @ T_WX_inv

    rvec, tvec = T_to_r_t(T_RW)

    path_pcd = pcd_list[i]
    path_image = image_list[j]

    image_origin = cv2.imread(path_image)
    cloud_origin = o3d.io.read_point_cloud(path_pcd)

    path_output = os.path.join(directory_output, stamp_image + ".png")

    pts2d = pcd_projection(image_origin, cloud_origin, rvec, tvec, camera_intrinsics, dist_coeffs, path_output)
