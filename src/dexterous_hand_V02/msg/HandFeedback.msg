# HandFeedback.msg
std_msgs/Header header
string[] name           # {"thumb", "index", "middle", "ring", "little", "thumb_rotation"};
float32[] position      # {"thumb", "index", "middle", "ring", "little", "thumb_rotation"}; 值范围：0.0(张开) ~ 100.0(握紧)
float32[] current       # {"thumb", "index", "middle", "ring", "little", "thumb_rotation"}; 手机电机电流大小(单位: mA)
uint8[] state           # 0 正在展开; 1 正在抓取; 2 位置到位停止; 3 电流保护停止; 4 力控到位停止; 5 电机堵转停止;

