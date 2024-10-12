# To get 11-frame combined pointclouds as groudtruth for Deprh Completion

Here, we are going to introduce a method as groudtruth for Depth Completion tasks. We tend to divide our approach into two parts basically: 1) Pointcloud Combining, 2) Pointcloud to Depth Map Transforming. 

In our experimental context, we take *Hesai XT-16* Lidar as the device to obtain pointcloud data. Additionally, we hope to convert the point cloud image obtained by the LiDAR to the camera coordinate system and project it into the camera projection plane as a depth map.

## Pointcloud Combining

First, we define the World Coordinate System as the the coordinate system of the initial liDAR point. Our purpose is to combine all pointclouds data that we have to the World Coodinate System.


# 🚀Updating

## 🚀 Update on Oct. 12th.

I discovered an issue with the timestamps of the files, so I revised the naming convention and updated the code in the repository.

### Timestamps

For instance, we have timestamps like `1727428229_50018000` and `1727428229_149986000`, which consist of a `Seconds` part and a `Nanoseconds` part. The `Seconds` part is always correct, consistently having 10 digits. However, the `Nanoseconds` part is not always 9 digits long—one has 8 digits while the other has 9. This discrepancy caused issues when comparing the timestamps. I initially attempted to fix this through code, but the solution was not elegant and complicated the readability of the code. Therefore, I decided to address the problem at its root by changing the file naming convention, which resolved the issue.

Previously, the two timestamps appeared as `1727428229_50018000` and `1727428229_149986000`. When comparing them, the system might incorrectly determine that `1727428229_50018000` is greater than `1727428229_149986000`, despite their `Seconds` part being identical. The difference lies in the `Nanoseconds` part, where we only need to compare these values. Clearly, `149986000` should be greater than `50018000`, but the system didn’t interpret it this way. To fix this, I established a rule to always pad the "Nanoseconds" part to 9 digits, so the timestamps now appear as `1727428229_050018000` and `1727428229_149986000`. This ensures the timestamps are compared correctly.