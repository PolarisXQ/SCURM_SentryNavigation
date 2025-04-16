# msg for referee system
# robot interactive data
# TODO
uint16 data_cmd_id # 子内容 ID
# 机器人 ID: 1, 英雄(红); 2, 工程(红); 3/4/5, 步兵(红); 6, 空中(红); 7, 哨兵(红); 9, 雷达站红;
# 101, 英雄(蓝); 102, 工程(蓝); 103/104/105, 步兵(蓝); 106, 空中(蓝); 107, 哨兵(蓝);  109, 雷达站蓝
uint16 sender_id # 发送者 ID
uint16 receiver_id #接收者 ID

# 由于串口通讯帧限制最大长度 64Byte 限制, data 段最大 49Byte
uint8[113] data