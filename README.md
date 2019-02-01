# google 开源的 cartographer 学习版本

## 运行环境
`Ubuntu 16.04 ros-kinetic`

## 项目结构
```
--- src
 |- .vscode
 |- cartographer
 |- cartographer_ros
 |- ceres-solver
 └- my_robot
```

## 项目说明

目前 `cartographer`、`cartographer_ros`、`ceres-solver`都为google远程分支上最新代码，部分注释为自己添加。

`my_robot`为学习用的节点。

## 编译说明
- [官方参考](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)

由于网络、依赖、重复安装等原因，官方编译会出问题;

可按照以下步骤：

```bash
# 下载代码，安装protobuf
$ git clone git@github.com:NOT-old-wang/carto_ws.git
$ cd carto_ws
$ sudo apt-get update
$ sudo apt-get install -y python-wstool python-rosdep ninja-build
$ src/cartographer/scripts/install_proto3.sh

# 以下三步如果报错 可不理会
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# 编译 build
$ catkin_make_isolated --use-ninja
```
如果编译失败，用`find`命令找到编译失败的相关文件(源码除外)删除，重新编译即可。
```
# find 命令示例
$ sudo find / -name "*ceres*" 
```
## my_robot 节点说明

