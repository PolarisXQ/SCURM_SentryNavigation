# msg for referee system
# Hurt data
# TODO

uint8 ARMOR_HURT = 0 # 装甲模块被弹丸攻击导致扣血
uint8 MODULE_OFFLINE_HURT = 1 # 裁判系统重要模块离线导致扣血
uint8 SHOOT_OVERSPEED_HURT = 2 # 射击初速度超限导致扣血
uint8 BULLET_OVERSPEED_HURT = 3 # 枪口热量超限导致扣血
uint8 CHASSIS_OVERSPEED_HURT = 4 # 底盘功率超限导致扣血
uint8 ARMOR_HIT_HURT = 5 # 装甲模块受到撞击导致扣血

uint8 armor_id # 当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线
# 时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致
# 扣血时，该数值为 0
uint8 hurt_type # 伤害类型