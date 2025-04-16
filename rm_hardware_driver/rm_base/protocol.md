# 串口通讯协议

通讯协议分为两个部分：上位机发送(Send)/上位机接收(Receive)

通讯帧结构使用 `rmoss_core` 中  \[[`rmoss_base`](https://github.com/robomaster-oss/rmoss_core/tree/humble/rmoss_base)\] 提供的 **64** 字节(Byte)定长数据包

128字节数据包数据布局：

| 数据头字节（0xff） | CMD_ID 数据标志位 |   数据字节    |  CRC8 校验字节(bit1 - bit61)   | 数据尾字节（0x0d） |
| :-------------: | :-------------: | :-----------: | :---------: | :----------------: |
|     0（1Byte）   |  1 (1Byte)      | 2-61 (60Byte) | 62（1Byte） |    63（1Byte）     |

PS： 对CMD_ID的定义在文件`include/rm_base/protocol_types.hpp`中

## 上位机发送协议(Send)

### CMD_ID定义

| CMD_ID | 定义 |
| :-: | :-: |
| 0x01 | 底盘控制 (SendID::CHASSISCMD) |
| 0x02 | 云台控制 (SendID::GIMBALCMD) |
| 0x03 | 射击控制 (SendID::SHOOTCMD) |
| 0x04 | 裁判系统机器人间交互数据 (SendID::REFREEINTERACT) |

### 数据段定义

**若未加说明默认数据从\[数据字节段\]第一位开始至数据结束，末尾空余位以0x00补齐61字节**

**注意每个结构体都需要以1字节对齐**

1字节对齐方式
- 在GCC中加入 `__attribute__((__packed__))` 编译指令
- 在STM32中加上 `__packed` 关键字

若标有**步兵机器人必须**意味着此为自瞄程序所必须实现的通讯协议，其余未特殊说明为哨兵机器人所必须实现的协议。

### 底盘控制 (SendID::CHASSISCMD)

```cpp
typedef struct __packed
{
  uint8_t type; // 1: 速度控制，(vx,vy,w)在底盘坐标下; 2: 底盘跟随云台(vx,vy)在云台坐标下; 3: 扭腰(vx,vy)在云台坐标下; 4: 陀螺 (vx,vy)在云台坐标下
  float linear_x; // 单位 m/s
  float linear_y;
  float linear_z;
  float angular_x; // 单位 rad/s
  float angular_y;
  float angular_z;
} chassis_cmd_t;
```

### 云台控制 (SendID::GIMBALCMD)
**步兵机器人必须**

```cpp
typedef struct __packed
{
  uint8_t yaw_type; // 1: 绝对角度控制，相对云台imu; 2: 相对角度控制; 3: 纯速度控制
  uint8_t pitch_type; // 同上
  float position_yaw; // 单位 rad
  float position_pitch;
  float velocity_yaw; // 单位 rad/s
  float velocity_pitch;
} gimbal_cmd_t;
```

### 射击控制 (SendID::SHOOTCMD)
**步兵机器人必须**

```cpp
typedef struct __packed
{
  uint8_t type; // 0:停止射击; 1: 一次射击; 2: 连续射击
  uint8_t projectile_num; // (可选参数) 射击子弹数目
} shoot_cmd_t;
```

### 裁判系统机器人间交互数据 (SendID::REFREEINTERACT)

```cpp
typedef struct __packed
{
  uint16_t cmd_id;
  uint16_t send_id;
  uint16_t recv_id;
  uint8_t data[113]; // 128字节帧限制最大数据量 113 字节
} sentry_cmd_t;
```

## 上位机接收协议(Receive)

若标有**步兵机器人必须**意味着此为自瞄程序所必须实现的通讯协议，其余未特殊说明为哨兵机器人所必须实现的协议。

### CMD_ID定义

| CMD_ID | 定义 |
| :-: | :-: |
| 0x01 | 比赛状态 (RecvID::GAMESTATUS) |
| 0x02 | 比赛结果 (RecvID::GAMERESULT) |
| 0x03 | 机器人血量数据 (RecvID::ROBOTHP) |
| 0x04 | 场地事件数据 (RecvID::EVENTDATA) |
| 0x05 | 比赛机器人状态 (RecvID::ROBOTSTATUS) |
| 0x06 | 实时功率热量 (RecvID::POWERHEATDATA) |
| 0x07 | 机器人位置 (RecvID::ROBOTPOS) |
| 0x08 | 机器人增益 (RecvID::ROBOTBUFF) |
| 0x09 | 伤害状态 (RecvID::HURTDATA) |
| 0x10 | 实时射击信息 (RecvID::SHOOTDATA) |
| 0x11 | 哨兵自主决策命令 (RecvID::SENTRYCMD) |
| 0x12 | 裁判系统机器人间交互数据 (RecvID::REFREEINTERACT) |
| 0x13 | 机器人关节状态 (RecvID::ROBOTJOINTSTATE) |
| 0x14 | 云台IMU数据(RecvID::GIMBALIMU) |

### 数据段定义

**若未加说明默认数据从\[数据字节段\]第一位开始至数据结束，末尾空余位以0x00补齐61字节**

#### 比赛状态 (RecvID::GAMESTATUS)

参考：[裁判系统串口协议附录](https://rm-static.djicdn.com/tem/17348/RoboMaster%20%E8%A3%81%E5%88%A4%E7%B3%BB%E7%BB%9F%E4%B8%B2%E5%8F%A3%E5%8D%8F%E8%AE%AE%E9%99%84%E5%BD%95%20V1.4%EF%BC%8820220805%EF%BC%89.pdf)

```cpp
typedef struct __packed
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
} ext_game_status_t;
```

### 比赛结果 (RecvID::GAMERESULT)

```cpp
typedef struct __packed
{
 uint8_t winner;
} ext_game_result_t;
```

### 机器人血量数据 (RecvID::ROBOTHP)

```cpp
typedef struct __packed
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP;
 uint16_t red_3_robot_HP;
 uint16_t red_4_robot_HP;
 uint16_t red_5_robot_HP;
 uint16_t red_7_robot_HP;
 uint16_t red_outpost_HP;
 uint16_t red_base_HP;
 uint16_t blue_1_robot_HP;
 uint16_t blue_2_robot_HP;
 uint16_t blue _3_robot_HP;
 uint16_t blue_4_robot_HP;
 uint16_t blue_5_robot_HP;
 uint16_t blue_7_robot_HP;
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} ext_game_robot_HP_t;
```

### 场地事件数据 (RecvID::EVENTDATA)

```cpp
typedef struct __packed
{
 uint32_t event_data;
} ext_event_data_t;
```

### 比赛机器人状态 (RecvID::ROBOTSTATUS)

```cpp
typedef struct __packed
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t current_hp;
 uint16_t max_HP;
 uint16_t shooter_id1_17mm_cooling_rate;
 uint16_t shooter_id1_17mm_cooling_limit;
 uint16_t shooter_id1_17mm_speed_limit
 uint16_t shooter_id2_17mm_cooling_rate;
 uint16_t shooter_id2_17mm_cooling_limit;
 uint16_t shooter_id2_17mm_speed_limit;
 uint16_t shooter_id1_42mm_cooling_rate;
 uint16_t shooter_id1_42mm_cooling_limit;
 uint16_t shooter_id1_42mm_speed_limit;
 uint16_t chassis_power_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_robot_status_t;
```

### 实时功率热量 (RecvID::POWERHEATDATA)

```cpp
typedef struct __packed
{
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t buffer_energy;
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;
```

### 机器人位置 (RecvID::ROBOTPOS)

```cpp
typedef struct __packed
{
 float x;
 float y;
 float angle;
} ext_robot_pos_t;
```

### 机器人增益 (RecvID::ROBOTBUFF)

```cpp
typedef struct __packed
{
  uint8_t recovery_buff;
  uint8_t cooling_buff;
  uint8_t defence_buff;
  uint8_t vulnerability_buff;
  uint16_t attack_buff;
} ext_buff_t;
```

### 伤害状态 (RecvID::HURTDATA)

```cpp
typedef struct __packed
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_hurt_data_t;
```

### 实时射击信息 (RecvID::SHOOTDATA)
**步兵机器人必须**

```cpp
typedef struct __packed
{
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t launching_frequency;
 float initial_speed;
} ext_shoot_data_t;
```

### 裁判系统机器人间交互数据 (RecvID::ROBOTINTERACT)

```cpp
typedef struct __packed
{
  uint16_t data_cmd_id;
  uint16_t send_id;
  uint16_t recv_id;
  uint8_t data[113]; // 128字节帧限制最大数据量 113 字节
} robot_interact_t;
```

### 机器人关节状态 (RecvID::ROBOTJOINTSTATE)

```cpp
typedef struct __packed
{
  float yaw_position;
  float yaw_velocity;
  float pitch_position;
  float pitch_velocity;
  float wheel_left_front_position;
  float wheel_left_front_velocity;
  float wheel_left_rear_position;
  float wheel_left_rear_velocity;
  float wheel_right_front_position;
  float wheel_right_front_velocity;
  float wheel_right_rear_position;
  float wheel_right_rear_velocity;
} robot_joint_state_t;
```

### 云台IMU数据 (RecvID::GIMBALIMU)
**步兵机器人必须**

```cpp
typedef struct __packed
{
  // 姿态四元数
  float ori_x;
  float ori_y;
  float ori_z;
  float ori_w;
  // 角速度
  float angular_v_x;
  float angular_v_y;
  float angular_v_z;
  // 线速度
  float acc_x;
  float acc_y;
  float acc_z;
} imu_t;
```