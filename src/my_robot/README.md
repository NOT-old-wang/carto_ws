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

## cartographr_ros
- Node类: 交互的主类, 主要包括
```
1.Ros相关: (NodeHandle, Publisher, Subscriber, ServiceServer, tf2_ros::TransformBroadcaster, WallTimer, Timer)
2.构图核心类: MapBuilderBridge (cartographer::mapping::MapBuilderInterface的代理类)
3.位姿外推器: cartographer::mapping::PoseExtrapolator
4.采样器: cartographer::common::FixedRatioSampler
```
- 主要方法
```cpp
// 输入数据(1: 外推器, 2: 传感数据数据采样 3: ros回调函数订阅数据)
int AddTrajectory(const TrajectoryOptions& options,
                  const cartographer_ros_msgs::SensorTopics& topics);

// 发出位置 tf
void PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event);
```

## cartographr
- mapping: 核心模块, 做构建地图相关的操作,约束,优化,scan_match,2D,3D模块,位姿外推器, 主要的类为 `MapBuilder`
- sensor: 传感器相关操作, 传感器数据的转化,数据的预处理(体素滤波器)
- transform: 机器人位姿坐标相关, 地图旋转平移等操作
- common: 1:lua配置文件解算 2:线程池 3:采样器 4:时间相关 5:线程安全的相关队列(阻塞等) 6:直方图打印 7:ceres解算器配置 8:类型映射 port
- pose_graph: 图优化相关
- cloud: 
- io: 
- TODO


## 位姿外推器: cartographer::mapping::PoseExtrapolator
- 作用: 使用运动估计 pose, 使用出了距离传感器之外的其它传感器数据来预测下一个扫描数据应该插入到子地图的什么地方
```
根据位姿pose 推出线速度和角速度
根据里程odom 推出线速度和角速度
```
- 输入
```
1: 带时间的位姿: common::Time time, const transform::Rigid3d& pose
2: IMU数据: const sensor::ImuData& imu_data
3: 里程的数据: const sensor::OdometryData& odometry_data
```
- 输出
```
1: 最后一次位置的时间: common::Time GetLastPoseTime()
2: common::Time GetLastExtrapolatedTime()
3: Eigen::Quaterniond EstimateGravityOrientation(common::Time time)
```

## 采样器: 
- 作用: 从数据流中均匀的按照固定频率采样数据
```
输入采样率 rate, 使用 Pulse()
rate : 0 ~ 1
例: rate == 0.5, 每两桢数据提取一桢
```

## 关于时间
- cartographer_ros 中 time_conversion.cc
```cpp
// 将 ros 时间转化为 carto 内部时间, 例: 把传感器时间戳的时间转化为内部时间
::cartographer::common::Time FromRos(const ::ros::Time& time) 

// 将 carto 内部时间 转化为 ros时间
::ros::Time ToRos(::cartographer::common::Time time)
```

- cartographer common 中 time.cc
```cpp
// TODO
```

## 关于传感器数据
cartographer sensor 里面定义了一系列传感器的数据格式
```
1. point_cloud: 点云相关数据
2. 
```
