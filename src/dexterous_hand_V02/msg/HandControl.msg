std_msgs/Header header
string[] name       # {"thumb", "index", "middle", "ring", "little", "thumb_rotation"};
uint8 mode          # ANGLE_CONTROL = 1,FORCE_CONTROL = 2; 如果力控模式切换到角度控制，需要先退出力控模式.
float32[] position  # {"thumb", "index", "middle", "ring", "little", "thumb_rotation"}; 转入值范围：0.0(张开) ~ 100.0(握紧)
float32[] force     # {"thumb", "index", "middle", "ring", "little"}; 单位 mN 转入值范围：0.0 ~ 65535.0; 0.0表示退出力控, 手指回弹; -1.0表示保持当前力控状态;