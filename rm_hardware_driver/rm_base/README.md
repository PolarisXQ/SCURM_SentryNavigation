# RM_Base

机器人上下位机通讯仓库

## 项目依赖

- [rmoss_core](https://github.com/robomaster-oss/rmoss_core)：用于底层串口硬件驱动和通讯帧解包
- [rmoss_interfaces](https://github.com/robomaster-oss/rmoss_interfaces)：云台和底盘控制ROS自定义消息类型

## 使用方法

- [ ] TODO: 加入使用方法文档

### 加入新的自定义上位机接收帧

- [ ] TODO: 接收demo

### 加入新的自定义上位机发送帧

- [ ] TODO: 发送demo

## [串口通讯协议](protocol.md)

通讯协议分为两个部分：上位机发送(Send)/上位机接收(Receive)

通讯帧结构使用 `rmoss_core` 中  \[[`rmoss_base`](https://github.com/robomaster-oss/rmoss_core/tree/humble/rmoss_base)\] 提供的 **64** 字节(Byte)定长数据包

128字节数据包数据布局：

| 数据头字节（0xff） | CMD_ID 数据标志位 |   数据字节    |  CRC8 校验字节(bit1 - bit61)   | 数据尾字节（0x0d） |
| :-------------: | :-------------: | :-----------: | :---------: | :----------------: |
|     0（1Byte）   |  1 (1Byte)      | 2-61 (60Byte) | 62（1Byte） |    63（1Byte）     |

PS： 对CMD_ID的定义在文件`include/rm_base/protocol_types.hpp`中

**注意每个结构体都需要以1字节对齐**

1字节对齐方式
- 在GCC中加入 __attribute__((__packed__))编译指令
- 在STM32中加上 __packed 关键字

**具体的通讯协议定义参考：[protocol.md](protocol.md)**

### CRC校验计算方式

使用`include/rm_base/crc.hpp`头文件中的`Get_CRC8_Check_Sum`函数计算**数据字节段**的CRC8值

参考样例:

```cpp
// 引入头文件
#include <rm_base/crc.hpp>
#include <rmoss_base/fixed_packet.hpp>

rmoss_base::FixedPacket<64> packet; // 定义一个数据帧

/*
 * 将待发送数据填入数据帧
 * float yaw_position = 1.57;
 * packet.load_data(yaw_position, 1);
 */

// 计算CRC校验位，注意数据段长度为 61 字节
packet.set_check_byte(Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

```
## ROS2 topic list

### 发布topic

| topic name | msg_type |
| :-------------: | :-------------: | 
|   event_data   | rm_interfaces/EventData |
|   game_status  | rm_interfaces/GameStatus |
|   game_result  | rm_interfaces/GameResult |
|   game_robot_pose  | rm_interfaces/RobotPose |
|   robot_status| rm_interfaces/RobotStatus|
|   game_robot_hp       | rm_interfaces/GameRobotHp |
|   robot_interactive_data | rm_interfaces/RobotInteractiveData |

### 订阅topic

| topic name | msg_type |
| :-------------: | :-------------: | 
|   chassis_cmd   | rm_interfaces/ChassisCmd |
|   gimbal_cmd    | rm_interfaces/GimbalCmd  |
|   shoot_cmd     | rm_interfaces/ShootCmd   |
