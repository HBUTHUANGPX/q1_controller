#!/bin/bash

# 使用方法一，
# 模拟左手控制topic消息，位置控制
# ./script/sim_ctl_topic.sh l p
# 模拟左手控制topic消息，力度控制
# ./script/sim_ctl_topic.sh l f

# 使用方法二，
# 模拟右手控制topic消息，位置控制
# ./script/sim_ctl_topic.sh r p
# 模拟右手控制topic消息，力度控制
# ./script/sim_ctl_topic.sh r f

source ./install/local_setup.sh
# ====================== 解析第一个参数 根据传入参数配置左右手 ======================
if [ "$1" = "l" ]; then
    # 传参 l → 左手配置
    TOPIC_NAME="/control/hand/left"
    FRAME_ID="left_hand"
elif [ "$1" = "r" ]; then
    # 传参 r → 右手配置
    TOPIC_NAME="/control/hand/right"
    FRAME_ID="right_hand"
else
    # 不传参数 默认左手，非法参数提示并退出
    echo -e "默认使用左手控制！\n使用方法: $0 [l|r]\n  l = left hand  左手\n  r = right hand 右手"
    TOPIC_NAME="/control/hand/left"
    FRAME_ID="left_hand"
    # 如果想让非法参数直接退出，打开下面这行注释
    exit 1
fi


# ====================== 解析第二个参数（控制模式） ======================
if [ "$2" = "p" ]; then
    # 位置控制模式
    CONTROL_MODE="position"
    # 位置控制对应的参数值
elif [ "$2" = "f" ]; then
    # 力控制模式
    CONTROL_MODE="force"
else
    # 第二个参数非法时提示并退出
    echo -e "控制模式参数错误！\n使用方法: $0 [l|r] [p|f]\n  第二个参数：p = position 位置控制 | f = force 力控制"
    exit 1
fi

# ====================== 全局配置区【所有参数在这里修改，一键生效】======================
# TOPIC_NAME="/control/hand/left"
# FRAME_ID="left_hand"
MOTOR_NAMES="['thumb_bend','index_bend','middle_bend','ring_bend','pinky_bend','thumb_rotate']"
ANGLE_ZEROS="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
FORCE_ZEROS="[0.0, 0.0, 0.0, 0.0, 0.0]"
# ====================================================================================

# ====================== 封装发布指令为函数，传参：控制模式、位置数组、力度数组 ==========
publish_hand_control() {
    local mode=$1
    local position=$2
    local force=$3
    ros2 topic pub --once ${TOPIC_NAME} dexterous_hand/msg/HandControl \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: '${FRAME_ID}'}, name: ${MOTOR_NAMES}, mode: ${mode}, position: ${position}, force: ${force}}"
}
# ====================================================================================


# ====================== 捕获Ctrl+C 退出执行自定义命令 ======================
# 定义退出执行的函数
exit_handler() {
    echo -e "\n[INFO] 检测到 Ctrl+C 退出，执行退出命令..."
    # ====================== ↓↓↓ 在这里修改成你的真实退出命令 ↓↓↓ ======================
    # 退出力空模式
    publish_hand_control 2 "[0.0]" "[0.0, 0.0, 0.0, 0.0, 0.0]"
    # ==================================================================================
    echo "[INFO] 退出命令执行完成，脚本正常退出!"
    exit 0
}

# 捕获 Ctrl+C 的中断信号 SIGINT，触发上面的退出函数
trap exit_handler SIGINT
# ====================================================================================

SLEEP_TIME_INTER=5
while true; do

    # 控制方式一：
    # 角度控制 mode=1
    if [ "$CONTROL_MODE" = "position" ]; then
        # 角度归0 mode=1
        echo "--angle control-- finger back -- "
        publish_hand_control 1 "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0.0, 0.0, 0.0, 0.0, 0.0]"
        sleep $SLEEP_TIME_INTER

        echo "--angle control-- finger 1 2 3 4 5 -- "
        publish_hand_control 1 "[50.0, 100.0, 100.0, 100.0, 100.0, 0.0]" "[0.0, 0.0, 0.0, 0.0, 0.0]"
        sleep $SLEEP_TIME_INTER

        echo "--angle control-- finger 2 4 -- "
        publish_hand_control 1 "[0.0, 50.0, 0.0, 50.0, 0.0, 0.0]" "[0.0, 0.0, 0.0, 0.0, 0.0]"
        sleep $SLEEP_TIME_INTER

        # 角度归0 mode=1
        echo "--angle control-- finger back -- "
        publish_hand_control 1 "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0.0, 0.0, 0.0, 0.0, 0.0]"
        sleep $SLEEP_TIME_INTER

        # 角度归0 mode=1 rotation
        echo "--angle control-- finger rotation -- "
        publish_hand_control 1 "[0.0, 0.0, 0.0, 0.0, 0.0, 100.0]" "[0.0, 0.0, 0.0, 0.0, 0.0]"
        sleep $SLEEP_TIME_INTER

    elif [ "$CONTROL_MODE" = "force" ]; then
    # :<<'COMMENT'
        # 控制方式二：
        # 退出力度控制 mode=2
        echo "--exit force control.-- "
        publish_hand_control 2 "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" "[0.0, 0.0, 0.0, 0.0, 0.0]"
        sleep $SLEEP_TIME_INTER

        # 力度控制 mode=2
        echo "--force control.--finger 2 3 4-- "
        publish_hand_control 2 "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"  "[0.0, 200.0, 200.0, 200.0, 0.0]"
        sleep $SLEEP_TIME_INTER

        # 力度控制 mode=2
        # echo "--force control.-- keep finger 3 "
        # publish_hand_control 2 "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"  "[0.0, 0.0, -1.0, 0.0, 0.0]"
        # sleep $SLEEP_TIME_INTER

    # COMMENT
    else
        echo "-- no this mode exit -- "
    fi
    echo "===================================================="
done

