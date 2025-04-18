# msg for referee system
bool supply_1_occupation # 己方补给站前补血点的占领状态，1 为已占领
bool supply_2_occupation # 己方补给站内部补血点的占领状态，1 为已占领
bool supply_3_occupation # 己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
bool rune_hit_occupation # 己方能量机关激活点的占领状态，1 为已占领
bool rune_small_enable # 己方小能量机关的激活状态，1 为已激活
bool rune_big_enable # 己方大能量机关的激活状态，1 为已激活
uint8 highland_1_occupation # 己方环形高地的占领状态，1 为被己方占领，2 为被对方占领
uint8 highland_2_occupation # 己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
uint8 highland_3_occupation # 己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
uint8 base_shield_remain # 己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）
uint8 dart_last_hit_time # 飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
uint8 dart_last_hit_object # 飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
uint8 center_buff_occupation # 中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。（仅 RMUL 适用）