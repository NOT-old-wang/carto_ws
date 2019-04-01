# Learn_Cartographer
google cartographer 学习和参数配置

## 前端 构建子图
- Local SLAM
  参数表: trajectory_builder_2d.lua 或 trajectory_builder_3d.lua
  作用是构建一个局部一致的 submaps 集，但是它会随时间的推移而变化

## 后端 全局优化 | 稀疏位姿调整
- Global SLAM
  参数表: pose_graph.lua
  在后台线程中运行，主要作用是找到闭环的约束条件,通过扫描 submaps 来完成scan-matching
  也结合其他的传感器数据以获得更高水平的视图，确定最一致的全局解决方案
  在3D模式下，还会尝试找到重力的方向

## Input
- sensor data
  1: Range Date: LaserScan | PointCloud
  2: Odometry
  3: Imu
- fixed frame Pose [和 Local的result 用来做全局优化]
