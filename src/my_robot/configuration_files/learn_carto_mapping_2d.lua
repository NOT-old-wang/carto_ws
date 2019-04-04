include "map_builder.lua"
include "trajectory_builder.lua"

-- 示例 单线激光雷达,不带imu
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_pose_extrapolator = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 150
TRAJECTORY_BUILDER_2D.min_range= 1.
TRAJECTORY_BUILDER_2D.max_range = 10.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_2D.use_imu_data = false

return options

-- lua 配置中都是已 米 和 秒 为单位
-- options块中定义的值定义了Cartographer ROS前端应如何与您的包进行交互。options段落后定义的值用于调整Cartographer的内部工作
-- 你的环境和机器人的TF帧ID中

-- 坐标系
-- map_frame = "map"                全局坐标系
-- tracking_frame = "base_link"     机器人中新坐标系 
-- published_frame = "base_link"    发布坐标系
-- odom_frame = "odom"              用于发布（非循环关闭）本地SLAM结果

-- 启用传感器或功能
-- use_pose_extrapolator            默认为true

-- provide_odom_frame = true        如果启用，则本地非闭环连续姿势将作为map_frame中的odom_frame发布
-- use_odometry = false             如果启用，请在主题“ odom ”上订阅nav_msgs / Odometry
-- use_nav_sat
-- use_landmarks
-- publish_frame_projected_to_2d = fasle  如果启用，则已发布的姿势将限制为纯2D姿势（无滚动，俯仰或z偏移/amo
-- num_laser_scans：                sensor_msgs/LaserScan  雷达  /scan 的个数,多个如"scan_1" ,"scan_2"...
-- num_multi_echo_laser_scans：     sensor_msgs/MultiEchoLaserScan 回波雷达,多个激光，topics为“echoes_1”, “echoes_2”...
-- num_subdivisions_per_laser_scan 
-- num_point_clouds：               sensor_msgs/PointCloud2 点云的个数,多个如“points2_1”, “points2_2”...    

-- 关于时间
-- lookup_transform_timeout_sec     使用tf2查找变换的超时秒数 
-- submap_publish_period_sec        发布子图的时间间隔（例如0.3秒）
-- pose_publish_period_sec          发布姿势的秒数间隔，例如频率为200 Hz的5e-3
-- trajectory_publish_period_sec    发布轨迹标记的间隔（以秒为单位），例如30e-3，持续30毫秒

-- rangefinder_sampling_ratio       测距仪消息的固定比率采样
-- odometry_sampling_ratio          里程消息的固定比率采样
-- fixed_frame_sampling_ratio       固定帧消息的固定比率采样
-- imu_sampling_ratio               IMU消息的固定比率采样
-- landmarks_sampling_ratio         地标消息的固定比率采样 


-- MAP_BUILDER.use_trajectory_builder_2d = true  2D slam
-- MAP_BUILDER.num_background_threads = 8        核数

-- 开启 imu 用作扫描方向的初始猜测，大大降低了扫描匹配的复杂性 3D必须
-- TRAJECTORY_BUILDER_2D.use_imu_data
-- TRAJECTORY_BUILDER_nD.imu_gravity_time_constant

-- 子图尺寸 <-- node 的个数  
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80 

-- 带通滤波器
-- TRAJECTORY_BUILDER_2D.min_range= 1.
-- TRAJECTORY_BUILDER_2D.max_range = 25.
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.
-- TRAJECTORY_BUILDER_3D.num_accumulated_range_data
-- TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 自适应体素滤波 votex filter [先前有一次固定的体素滤波]
-- 较小的立方体大小将导致更密集的数据,较大的立方体大小会导致数据丢失，但会更快
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.2
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400

-- 运动滤波器 motion filter [根据 距离,角度,时间]    
-- TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 1.5
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(9.)
-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 7.

-- 两种扫描匹配策略
-- 1
-- 取初始猜测事先并发现其中扫描匹配适合的子地图的最佳点,通过插入子图和子像素对齐扫描来实现
-- 无法修复明显大于子图分辨率的错误
-- 传感器设置和时间是合理的，那么仅使用CeresScanMatcher它通常是最佳选择
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.1
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.1
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
-- Ceres使用下降算法针对给定的迭代次数优化运动。Ceres可以配置为根据您自己的需要调整收敛速度
-- TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps
-- TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.max_num_iterations
-- TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.num_threads

-- 2
-- 
-- 有其他传感器或者您不信任它们，则可以启用
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 20.
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 子图插入概率的更新
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.62   加快
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.47  0,1是障碍物

-- 关闭全局SLAM：0 
-- POSE_GRAPH.optimize_every_n_nodes = 0

-- 获得格式化为直方图的约束构建器的常规报告
-- POSE_GRAPH.constraint_builder.log_matches

-- 调整局部SLAM和里程计的各个权重
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight
-- POSE_GRAPH.optimization_problem.odometry_translation_weight
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight

-- 限制约束（和计算）的数量
-- POSE_GRAPH.constraint_builder.sampling_ratio      对应于约束的数量上限
-- POSE_GRAPH.min_score = 0.60                       成为约束的最低分数,值越大，计算速度相对越快，约束数量相对越少
-- 备注：上述各参数调整核心是闭环检测（约束检测），
-- 闭环检测是图优化过程中最为重要的部分，也是最为耗时的部分，
-- 因此减少约束总数和搜索范围可以有效提高实时性

-- 实现实时循环闭合扫描匹配
-- FastCorrelativeScanMatcher 依靠“ 分支定界 ”机制在不同的格点分辨率的工作，有效地消除不正确匹配数
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window=math.rad(20.)    对应于闭环检测（约束检测）时的搜索范围

-- 迭代次数
-- POSE_GRAPH.max_num_final_iterations
