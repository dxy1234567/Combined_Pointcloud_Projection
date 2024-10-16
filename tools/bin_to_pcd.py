import numpy as np
import open3d as o3d

def read_kitti_bin_file(bin_path):
    # 读取二进制文件
    point_cloud = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    # 将点的x, y, z坐标提取出来
    points = point_cloud[:, :3]
    return points

def get_point_cloud(bin_path):
    # 将点转换为 Open3D 的 PointCloud 对象
    points = read_kitti_bin_file(bin_path)

    points_3d = o3d.geometry.PointCloud()
    points_3d.points = o3d.utility.Vector3dVector(points)
    return points_3d


path_bin = ""
path_output = ""
pcd = get_point_cloud(path_bin)
o3d.io.write_point_cloud(path_output, pcd)