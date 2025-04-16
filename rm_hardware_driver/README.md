# RM_HARDWARE_DRIVER

机器人硬件驱动与通讯仓库，用于上下位机通讯和硬件传感器的ROS驱动

## 项目依赖

- [rmoss_core](https://github.com/robomaster-oss/rmoss_core)：用于底层串口硬件驱动和通讯帧解包
- [rmoss_interfaces](https://github.com/robomaster-oss/rmoss_interfaces)：云台和底盘控制ROS自定义消息

## 使用说明

下载编译代码
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src

git clone https://e.coding.net/itlkineticpanda/23chaoduiitlkineticpanda/rm_hardware_driver.git
vcs import --recursive < rm_hardware_driver/deps.repos # clone依赖仓库

# 编译
cd ..
colcon build --symlink-install
```

运行测试
```bash
ros2 launch rm_base rm_base.launch.py
```

## 通讯协议详细说明

参考 `rm_base` 的 [README](./rm_base/README.md)中对于串口通讯的详细说明

## License

```
Copyright 2022-2023 ITLKineticPanda

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```