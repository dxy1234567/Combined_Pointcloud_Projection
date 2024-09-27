import open3d as o3d
import os
import numpy as np

def extract_timestamp(file_name):
    # 提取文件名中的时间戳部分（假设格式如 "file_<秒数>_<纳秒数>.txt"）
    # 先移除前缀 "file_" 和后缀 ".txt"
    timestamp, _ = os.path.splitext(file_name)
    sec, nsec = map(int, timestamp.split('_'))
    return sec, nsec

def read_odom(path_odom):
    odom_lists = []
    with open(path_odom) as file:
        for line in file:
            line = line.strip()
            data = list(map(float, line.split()))
            odom_lists.append(data)

    return odom_lists

def odom_to_T_r(odom_lists, i):
    """
        将第i行对应的位姿信息转换成旋转矩阵T和平移向量t
    """
    odom = odom_lists[i]

    R = np.zeros((3, 3))
    t = np.zeros((3, 1))

    R[0, :3] = odom[0:3]
    R[1, :3] = odom[4:7]
    R[2, :3] = odom[8:11]

    t[0] = odom[3]
    t[1] = odom[7]
    t[2] = odom[11]
    t = t.flatten()

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T, R, t

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







