# msg for referee system
# robot status

# 本机器人 ID:
# 1:红方英雄机器人
# 2:红方工程机器人
# 3/4/5:红方步兵机器人（与机器人 ID 3~5 对应）
# 6:红方空中机器人
# 7:红方哨兵机器人
# 8:红方飞镖
# 9:红方雷达
# 10:红方前哨站
# 11:红方基地
# 101:蓝方英雄机器人
# 102:蓝方工程机器人
# 103/104/105:蓝方步兵机器人（与机器人 ID 3~5 对应）
# 106:蓝方空中机器人
# 107:蓝方哨兵机器人
# 108:蓝方飞镖
# 109:蓝方雷达
# 110:蓝方前哨站
# 111:蓝方基地

uint8 robot_id # 本机器人 ID
uint8 robot_level # 机器人等级
uint16 current_hp # 机器人当前血量
uint16 max_hp # 机器人血量上限
uint16 shooter_barrel_cooling_value # 机器人枪口热量每秒冷却值
uint16 shooter_barrel_heat_limit # 机器人枪口热量上限
uint16 chassis_power_limit # 机器人底盘功率上限
# 电源管理模块的输出情况:
# bit 0:gimbal 口输出:0 为无输出，1 为 24V 输出
# bit 1:chassis 口输出:0 为无输出，1 为 24V 输出
# bit 2:shooter 口输出:0 为无输出，1 为 24V 输出
# bit 3-7:保留
uint8 power_management_gimbal_output 
uint8 power_management_chassis_output 
uint8 power_management_shooter_output