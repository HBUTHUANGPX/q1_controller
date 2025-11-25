# 基于Xbox手柄的标准数据结构
std_msgs/Header header  # 时间戳和帧ID

float32[] axes         # 轴值数组，通常6个（范围-1.0到1.0）：[左X, 左Y, LT, 右X, 右Y, RT]

int32[] buttons        # 按钮状态数组，通常11个（0未按下，1按下）：[A, B, X, Y, LB, RB, Back, Start, Guide, 左摇杆按, 右摇杆按]

int32[] hats           # 帽子状态数组，通常1个（D-pad）：[(X, Y)]，X/Y为-1/0/1