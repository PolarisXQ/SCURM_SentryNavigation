# msg for referee system
# game status

# Game Type
uint8 RMUC = 1 # RoboMaster 机甲大师赛
uint8 RMUT = 2 # RoboMaster 机甲大师单项赛
uint8 RMUA = 3 # ICRA RoboMaster 人工智能挑战赛
uint8 RMUL3V3 = 4 # RoboMaster 联盟赛 3V3
uint8 RMUL1V1 = 5 # RoboMaster 联盟赛 1V1
# Game Progress
uint8 GAME_UNSTARTED = 0 # 未开始比赛
uint8 GAME_READY = 1 # 准备阶段
uint8 GAME_INITIAL = 2 # 自检阶段
uint8 GAME_START_COUNTDOWN = 3 # 5s 倒计时
uint8 GAME_RUNNING = 4 # 对战中
uint8 GAME_STOP = 5 # 比赛结算中

std_msgs/Header header
uint8 game_type # Game type
uint8 game_progress # Game progress
uint16 stage_remain_time # Remain time of current stage (s)
uint64 sync_time_stamp # Unix time stamp. Only used when valid NTP server provided to MCU