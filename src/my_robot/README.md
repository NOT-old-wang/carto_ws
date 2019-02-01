# Learn_Cartographer
google cartographer 学习和参数配置

## 前端
- 局部SLAM
  作用是构建一个局部一致的submaps 集，但是它会随时间的推移而变化
  有关的它的大部分选项都能在相对应的trajectory_builder_2d.lua或trajectory_builder_3d.lua中找到

## 后端
- 全局SLAM
  在后台线程中运行，主要作用是找到闭环的约束条件,通过扫描submaps 来完成scan-matching
  也结合其他的传感器数据以获得更高水平的视图，确定最一致的全局解决方案
  在3D模式下，还会尝试找到重力的方向
  有关的大部分选项都能在pose_graph.lua中找到
