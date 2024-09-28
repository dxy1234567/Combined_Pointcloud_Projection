import open3d as o3d
import numpy as np
import os
import bisect
import cv2
from pcd2depth import pcd_projection

def T_to_r_t(T):
    R = T[:3, :3]
    r, _ = cv2.Rodrigues(R)

    t = T[:3, 3]

    return r, t

def path_to_timestamp(file_path):
    """
        单个路径转为整数时间戳
    """
    filename = os.path.basename(file_path)
    timestamp, _ = os.path.splitext(filename)
    timestamp = timestamp.replace('_', '.')      # 将下划线去除
    timestamp = float(timestamp)
    return timestamp

def pose_to_transformation(pose):
    """
        一个位姿转变换矩阵
    """
    # 获取对应的位姿矩阵
    tvec = pose[0], pose[1], pose[2]

    q = pose[6], pose[3], pose[4], pose[5]
    R = o3d.geometry.get_rotation_matrix_from_quaternion(q)

    T = np.eye(4)

    T[:3, :3] = R
    T[:3, 3] = tvec
    return T

def find_closest_timestamp(timestamps, timestamp):
    """
        从相机时间戳组中找到最接近雷达时间戳的时间戳。
    """
    # 使用bisect模块的bisect_left函数查找插入点
    idx = bisect.bisect_left(timestamps, timestamp)
    
    # 边界条件处理
    if idx == 0:
        return timestamps[0]
    elif idx == len(timestamps):
        return timestamps[-1]
    
    # 找到最接近的时间戳
    before = timestamps[idx - 1]
    after = timestamps[idx]
    
    # 比较哪个时间戳更接近雷达时间戳
    if timestamp - before < after - timestamp:
        return before, idx - 1
    else:
        return after, idx

# 假设你已经加载了11帧的点云和它们对应的位姿矩阵

def get_points_path(pcd_directory):
    """
        point_list: 字符串列表
    """
    # points_list 是点云文件路径的列表
    points_list = []
    for file_name in sorted(os.listdir(pcd_directory)):
        if file_name.endswith(".pcd"):
            file_path = os.path.join(pcd_directory, file_name)
            points_list.append(file_path)

    return points_list


def get_poses(pose_path):
    """
        高频位姕列表
    """
    # poses_list 是相应的位姿矩阵列表 (x, y, z, qx, qy, qz, qw)
    poses_list = []

    with open(pose_path) as file:
        for line in file:
            line = line.strip()
            data = list(map(float, line.split()))

            poses_list.append(data)
    return poses_list

def get_images_list(camera_directory):
    images_list = []

    for filename in sorted(os.listdir(camera_directory)):
        images_path = os.path.join(camera_directory, filename)
        images_list.append(images_path)
    
    return images_list

def get_camera_timestamps(camera_directory):
    """
        从相机图像文件中得到所有相机时间戳(float)
        相机文件分隔并不是小数点，而是下划线
    """
    camera_timestamps = []
    
    for file_name in sorted(os.listdir(camera_directory)):
        timestamp, _ = os.path.splitext(file_name)
        timestamp = timestamp.replace('_', '.')
        timestamp = float(timestamp)
        camera_timestamps.append(timestamp)
    return camera_timestamps

def get_one_combined_pcd(i, points_list, poses_timestamps, poses_list, T_MX):
    """
        得到11帧累计的点云
    """
    combined_pcd = o3d.geometry.PointCloud()
    for j in range(i - 5, i + 6):
        # j时刻单个点云 读取点云文件
        pcd = o3d.io.read_point_cloud(points_list[j])
        point_timestamp = path_to_timestamp(points_list[j])     # 点云对应的时间戳

        # 获取对应的位姿矩阵（变换矩阵）【点云与位姿不同步问题】
        t_m, i_m = find_closest_timestamp(poses_timestamps, point_timestamp)
        T = pose_to_transformation(poses_list[i_m][1:8])  # 位姿时间戳不对。

        # 将点云转换到世界坐标系
        pcd.transform(T_MX)     # 先将点云从XT16转换到MID360上
        pcd.transform(T)        # 再将点云从MID360转换到世界系上

        # 拼接到整体点云
        combined_pcd += pcd
        o3d.io.write_point_cloud("combined_pointcloud/output_test/{}.pcd".format(str(j)), combined_pcd)
    return combined_pcd

'''
    MID360 to RealSense
    Transform Matrix:
    -0.00608978   -0.999726   0.0225815   0.0297088
    0.0452806   -0.022834   -0.998712   -0.105938
    0.998957 -0.00505946   0.0454075   -0.226056
            0           0           0           1
'''

"""
    XT16 to MID360
     -0.02  0.999  0.039  0.009
    -1 -0.021  0.008 -0.008
 0.009 -0.038  0.999  -0.06
     0      0      0      1
"""

T_CM = np.array([
    [-0.00608978, -0.999726, 0.0225815, 0.0297088],
    [0.0452806, -0.022834, -0.998712, -0.105938],
    [0.998957, -0.00505946, 0.0454075, -0.226056],
    [0, 0, 0, 1]
])

T_CM = np.linalg.inv(T_CM)

T_MX = np.array(
    [
    [-0.02, 0.999, 0.039, 0.009],
    [-1, -0.021, 0.008, -0.008],
    [0.009, -0.038, 0.999, -0.06],
    [0, 0, 0, 1]
    ])

T_MX = np.linalg.inv(T_MX)

# T_CX = np.dot(T_CM, T_MX)
# T_CX = T_CM @ T_MX

# rvec, tvec = T_to_r_t(T)

camera_intrinsics= np.float64([[431.574523925781, 0, 429.674438476562],
                            [0, 431.574523925781, 237.917541503906],
                            [0, 0, 1]])

dist_coeffs = np.float64([0, 0, 0, 0, 0])


pcd_directory = "/data/gml/20240829/gml_2024-08-29-11-21-17/_hesai_pandar/"
points_list = get_points_path(pcd_directory)

pose_high_path = "/data/gml/20240829/gml_2024-08-29-11-21-17/slam_result/pose/pose_100hz.txt"
poses_high_list = get_poses(pose_high_path)

# pose_low_path = "/data/gml/20240829/gml_2024-08-29-11-21-17/slam_result/pose/pose_10hz.txt"
# poses_low_list = get_poses(pose_low_path)

combined_pcd = o3d.geometry.PointCloud()

camera_directory = "/data/gml/20240829/gml_2024-08-29-11-21-17/_camera_infra1_image_rect_raw/"
images_list = get_images_list(camera_directory)
camera_timestamps = get_camera_timestamps(camera_directory)

poses_timestamps = [e[0] for e in poses_high_list]

num_pcd = len(points_list)
for i in range(88, num_pcd - 5):
    combined_pcd = get_one_combined_pcd(i, points_list, poses_timestamps, poses_high_list, T_MX)
    
    # 写出累计后的点云文件
    o3d.io.write_point_cloud("combined_pointcloud/combined_pcd/{}.pcd".format(str(poses_high_list[i][0])), combined_pcd)

    lidar_path = points_list[i]
    lidar_timestamp = path_to_timestamp(lidar_path)

    # 在相机时间戳组中找到雷达时间戳的对应的额相机时间戳
    t_camera, i_camera = find_closest_timestamp(camera_timestamps, lidar_timestamp)
    # 在高频时间戳中找到近似相机时间戳
    
    t_lidar, i_lidar = find_closest_timestamp(poses_timestamps, t_camera)

    T_WM = pose_to_transformation(poses_high_list[i_lidar][1:8])
    T_CW = np.dot(T_CM, np.linalg.inv(T_WM))

    rvec, tvec = T_to_r_t(T_CW)

    path_camera = images_list[i_camera]
    path_lidar = points_list[i_lidar]

    image_origin = cv2.imread(path_camera)
    cloud_origin = o3d.io.read_point_cloud(path_lidar)

    path_output = "combined_pointcloud/projection"

    image_file_name = os.path.basename(path_camera)

    path_output = os.path.join(path_output, image_file_name)
    pts2d = pcd_projection(image_origin, cloud_origin, rvec, tvec, camera_intrinsics, dist_coeffs, path_output)



# # 合并后的点云可以保存或进一步处理
# o3d.visualization.draw_geometries([combined_pcd])
