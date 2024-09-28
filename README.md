# To get 11-frame combined pointclouds as groudtruth for Deprh Completion

Here, we are going to introduce a method as groudtruth for Depth Completion tasks. We tend to divide our approach into two parts basically: 1) Pointcloud Combining, 2) Pointcloud to Depth Map Transforming. 

In our experimental context, we take *Hesai XT-16* Lidar as the device to obtain pointcloud data. Additionally, we hope to convert the point cloud image obtained by the LiDAR to the camera coordinate system and project it into the camera projection plane as a depth map.

## Pointcloud Combining

First, we define the World Coordinate System as the the coordinate system of the initial liDAR point. Our purpose is to combine all pointclouds data that we have to the World Coodinate System.