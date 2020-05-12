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
- mapping: 核心模块, 做构建地图相关的操作,约束,优化,scan_match,2D,3D模块,位姿外推器, 主要的类为 `MapBuilder`, `TrajectoryBuilderInterface` --> `CollatedTrajectoryBuilder`,
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
- `sensor::Data` 所有传感器都将转化为这个数据, 然后调用 `void AddData(std::unique_ptr<sensor::Data> data)`
所有 `sensor::Data` 通过 `OrderedMultiQueue`(多线程支持的有序多队列) 管理处理传感器数据的回调函数队列 `HandleCollatedSensorData`
- 注入: 在 cartographer/cloud/internal/handles/中
```
add_sensor_data_batch_handler.cc
add_imu_data_handle.h/cc
add_odometry_data_handle.h/cc
add_rangefinder_data_handle.h/cc
```

## 占据概率相关
- 相关代码
```
cartographer/mapping/probablity_value.h/cc
cartographer/mapping/range_data_inserter_interface.h/cc
  1.probability_grid_range_data_inserter_2d.h/cc
  2.tsdf_range_data_inserter_2d.h/cc
cartographer/mapping/2d/probability_grid.h
```
- 理解
```
1: 地图是经过栅格化的地图，且地图中的每个像素值用概率来表示
  p = 1  :表示障碍, 深色
  p = 0.5  :表示未知，地图初始化的时候，每个栅格可以初始化为0.5
  p = 0   :表示无障碍
2: 更新每个栅格的概率
  使用赔率（odd）来更新栅格的概率值. 如果被激光击中(hit), 则概率值会增大; 如果被激光穿透（miss, 则概率值会减小, 每次更新栅格概率, 只更新当前激光看到的栅格
3: 更新公式见论文 Real-Time Loop Closure in 2D LIDAR SLAM (3)
```

## scan match 相关
### ceres scan matching
- 参考论文: A Flexible and Scalable SLAM System with Full 3D Motion Estimation
- 代码相关
```
使用 ceres 将论文中公式(CS) 分解成3个代价函数
1. Occupied space cost function --> 根据概率地图
2. Translation delta cost function  --> 根据位移权重 e = K * (angle' - angle)
3. Rotational drlta cost function  --> 根据旋转权重 e(0) = K * (x' - x); e(1) = K * (y' - y)
相对应优化三种残差: 1.占用栅格与扫描数据的匹配度, 2.优化后的位置相对于target_translation(外推器估计的位姿)的距离 3.旋转角度相对于迭代初值的偏差
对于优化结果添加位置偏差量和角度偏差量的约束，是因为优化后的机器人位置应当与位姿外推出来的估计值的偏差不大
```

- 主要函数, 优化出3个变量, (x,y,theta)
```cpp
/*
 * input:
 * 1.目标 pose
 * 2.当前的 scan 的位姿的初始估计 initial_pose_estimate
 * 3.当前 scan 点云（2D）point_cloud
 * 4.local map 概率分布栅格图 probability_grid
 * output
 * 1. 计算得到的位姿估计 pose_estimate
 * 2. ceres solver 计算的总结 summary
*/
void CeresScanMatcher::Match(const transform::Rigid2d& target_translation,
                             const transform::Rigid2d& initial_pose_estimate,
                             const sensor::PointCloud& point_cloud,
                             const ProbabilityGrid& probability_grid,
                             transform::Rigid2d* const pose_estimate,
                             ceres::Solver::Summary* const summary) const {}
// 除了要求hit点在占用栅格上出现的概率最大化之外， 还通过两个残差项约束了优化后的位姿估计在原始估计的附近。
```
