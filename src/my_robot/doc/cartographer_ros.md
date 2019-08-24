### 代码结构

```
node_main.cc

环境变量标志 源码在/usr/include/"gflags/gflags.h" 定义一系列的宏
可以在代码中更改，也可以在`launch`文件中给定
```

- node_constants.h  &&  node_constants.cc
```
功能1： 命名 topic 的 默认字符串名字(constexper)
功能2： 将数量大于1的传感器名字重命名  如 scan -->  scan_1, scan_2   
```

- msg_conversion.h  &&  msg_conversion.cc 
```
将 `cartographer` 和`ros` 的数据结构转化
sensor、rigid3d、landmark_list、OccupancyGrid、Eigen 等
```

- trajectory_options.h  &&  trajectory_option.cc
```
将 `lua` 配置文件里的参数加载到 struct 中,初始化 trajectory 时重载了不同的加载方式

将 定义的 `rosmsg` 转化为 struct， 用来调用检查函数...
将 struct的数据 转化为 `rosmsg`
```

- urdf_reader.h  &&  urdf_reader.cc
```
输入 `urdf` 文件 
输出  旋转变化位置 geometry_msgs/TransformStamped
```

- time_conversiton.h  &&  time_conversiton.cc
```
ros::time 和 `cartographer` 的 time 数据结构的互相转化
```

- map_builder_bridge.h  &&  map_builder_bridge.cc
```

```

- tf_bridge.h  &&  tf_bridge.cc

  ```
  用来存储 tf2_ros::Buffer 数据的类
  
  输入 tracking_frame、lookup_transform_timeout_sec、tf2_ros::Buffer
  
  调用 msg_conversion 中的 ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
                                                             requested_time, timeout)))
  输出 unique_ptr<::cartographer::transform::Rigid3d>
  
  ```



