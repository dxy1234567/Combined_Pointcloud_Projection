"""
    单独实现一个点云投影，可用于验证实验平台外参。
"""
"""
    现在已获得在世界坐标系下的拼接点云，现在要将点云转换到相机坐标系下，并投影。
    World -> XT-16 -> RealSense
"""

import open3d as o3d
import numpy as np
import os
import cv2
import sys
sys.path.append(".")

from utils.functions import T_to_r_t 
from utils.pcd2depth import pcd_projection

path_pcd = "/home/cjs/rosbag/2024-09-27/gml_2024-09-27-17-10-28/_hesai_pandar"
path_image = "/home/cjs/rosbag/2024-09-27/gml_2024-09-27-17-10-28/_camera_infra1_image_rect_raw"

directory_output = "/home/cjs/rosbag/2024-09-27/gml_2024-09-27-17-10-28/projection"

# 相机内参
## 
camera_intrinsics= np.float64([[431.574523925781, 0, 429.674438476562],
                            [0, 431.574523925781, 237.917541503906],
                            [0, 0, 1]])

## 畸变参数
dist_coeffs = np.float64([0, 0, 0, 0, 0])

# 外参: XT to RealSense
T_RM = np.array([
    [-0.0347702, -0.999329, -0.0115237, 0.113479],
    [0.0359214, 0.0102735, -0.999302, -0.216314],
    [0.99875, -0.0351599, 0.0355401, -0.00212184],
    [0, 0, 0, 1]
])

# T_RM = np.linalg.inv(T_RM)

rvec, tvec = T_to_r_t(T_RM)

image_origin = cv2.imread(path_image)
cloud_origin = o3d.io.read_point_cloud(path_pcd)

filename_image = os.path.basename(path_image)
path_output = os.path.join(directory_output, filename_image)

pts2d = pcd_projection(image_origin, cloud_origin, rvec, tvec, camera_intrinsics, dist_coeffs, path_output)
