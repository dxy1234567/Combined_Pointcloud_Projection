import cv2
import numpy as np
import open3d as o3d

# 定义函数根据深度获取颜色
def get_color(cur_depth, max_depth, min_depth):
    scale = (max_depth - min_depth) / 10
    if cur_depth < min_depth:
        return (0, 0, 255)  # 返回蓝色
    elif cur_depth < min_depth + scale:
        green = int((cur_depth - min_depth) / scale * 255)
        return (0, green, 255)  # 返回蓝到黄的渐变色
    elif cur_depth < min_depth + scale * 2:
        red = int((cur_depth - min_depth - scale) / scale * 255)
        return (0, 255, 255 - red)  # 返回黄到红的渐变色
    elif cur_depth < min_depth + scale * 4:
        blue = int((cur_depth - min_depth - scale * 2) / scale * 255)
        return (blue, 255, 0)  # 返回红到绿的渐变色
    elif cur_depth < min_depth + scale * 7:
        green = int((cur_depth - min_depth - scale * 4) / scale * 255)
        return (255, 255 - green, 0)  # 返回绿到黄的渐变色
    elif cur_depth < min_depth + scale * 10:
        blue = int((cur_depth - min_depth - scale * 7) / scale * 255)
        return (255, 0, blue)  # 返回黄到蓝的渐变色
    else:
        return (255, 0, 255)  # 返回紫色

def pcd_projection(image_origin, cloud_origin, rvec, tvec, camera_intrinsics, dist_coeffs, path_output):
    # Extract 3D points within a certain range
    pts_3d = []     # 三位点（以雷达坐标系为基准）
    for point_3d in np.asarray(cloud_origin.points):        # 遍历点云中所有点
        if 0 < point_3d[0] < 5 and -1 < point_3d[2] < 10:   # 筛选x,y,z坐标：（x：0到5；y：-1到10）
            pts_3d.append((point_3d[0], point_3d[1], point_3d[2]))      # 将符合要求的点添加到pts_3d中

    # 找出深度的最大值和最小值
    min_depth = min(point_3d[0] for point_3d in pts_3d)
    max_depth = max(point_3d[0] for point_3d in pts_3d)


    # Project 3D points into image view
    pts_2d, _ = cv2.projectPoints(np.array(pts_3d), rvec, tvec, camera_intrinsics, dist_coeffs)
    image_project = image_origin.copy()

    for i, point_2d in enumerate(pts_2d):
        x, y = point_2d.ravel()
        x, y = int(x), int(y)
        if 0 <= x < image_origin.shape[1] and 0 <= y < image_origin.shape[0]:
            cur_depth = pts_3d[i][0]  # 获取当前点的深度
            color = get_color(cur_depth, max_depth, min_depth)  # 根据深度获取颜色
            image_project[y, x] = color  # 设置点云的颜色

    cv2.imwrite(path_output, image_project)
    cv2.waitKey(0)
    
    return pts_2d

