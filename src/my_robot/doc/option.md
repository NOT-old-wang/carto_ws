## option 兵分两路输入

### NodeOptions
```
cartographer::mapping::proto::MapBuilderOptions

map_frame = "map"                全局坐标系

lookup_transform_timeout_sec     使用tf2查找变换的超时秒数 
submap_publish_period_sec        发布子图的时间间隔（例如0.3秒）
pose_publish_period_sec          发布姿势的秒数间隔，例如频率为200 Hz的5e-3
trajectory_publish_period_sec    发布轨迹标记的间隔（以秒为单位），例如30e-3，持续30毫秒

use_pose_extrapolator = true
```

### TrajectoryOptions
```
cartographer::mapping::proto::TrajectoryBuilderOptions

tracking_frame = "base_link"     机器人中新坐标系 
published_frame = "base_link"    发布坐标系
odom_frame = "odom"              用于发布（非循环关闭）本地SLAM结果

provide_odom_frame = true        如果启用，则本地非闭环连续姿势将作为map_frame中的odom_frame发布
use_odometry = false             如果启用，请在主题“ odom ”上订阅nav_msgs / Odometry
use_nav_sat
use_landmarks
publish_frame_projected_to_2d    如果启用，则已发布的姿势将限制为纯2D姿势（无滚动，俯仰或z偏移/amo
num_laser_scans：                2d雷达  /scan 的个数,多个如"scan_1" ,"sca.
num_multi_echo_laser_scans：     /MultiEchoLaserScan 回波雷达,多个订阅为“echoes_1”, “echoes_2”.
num_subdivisions_per_laser_scan 
num_point_clouds：

rangefinder_sampling_ratio       测距仪消息的固定比率采样
odometry_sampling_ratio          里程消息的固定比率采样
fixed_frame_sampling_ratio       固定帧消息的固定比率采样
imu_sampling_ratio               IMU消息的固定比率采样
landmarks_sampling_ratio         地标消息的固定比率采样 
```