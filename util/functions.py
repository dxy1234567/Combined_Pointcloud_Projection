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