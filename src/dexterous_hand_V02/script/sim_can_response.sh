#!/bin/bash

# =====================================================
# 📌 【全局常量宏定义 - 基础配置】 统一修改区
# =====================================================
# CAN总线设备名称 (修改这里切换 vcan0/can0/can1)
CAN_DEVICE="vcan0"
CAN_BITRATE=1000000
# CAN数据格式适配：定义带空格的匹配数据，和candump输出一致
# 格式说明：两两空格分隔，和candump打印的格式完全匹配
DATA_FORMAT_SPACE=" "
# =====================================================
# 📌 【RECEIVE 接收规则 + SEND 发送响应 宏定义区】核心配置
# 格式说明：一行一条规则，按顺序定义，一一对应，绝对不用改逻辑！
# 规则1：收到什么 → 回复什么
RECEIVE_ID_1="001"          # 接收的CAN ID
RECEIVE_DATA_1="1122334455667788"         # 接收的CAN数据 (精准匹配，空则只匹配ID)
RESPONSE_ID_1="002"             # 响应发送的CAN ID
RESPONSE_DATA_1="8877665544332211"  # 响应发送的CAN数据

# 规则2：收到什么 → 回复什么
RECEIVE_ID_2="002"          # 接收的CAN ID
RECEIVE_DATA_2=""           # 接收的CAN数据 (空值=仅匹配ID，不校验数据)
RESPONSE_ID_2="004"             # 响应发送的CAN ID
RESPONSE_DATA_2="99887766"      # 响应发送的CAN数据

# =====================================================
# 屏蔽下面内容，在终端运行即可，无需修改！
# =====================================================
:<<'COMMENT'
sudo modprobe vcan >/dev/null 2>&1
sudo ip link del ${CAN_DEVICE} >/dev/null 2>&1 || true
sudo ip link add dev ${CAN_DEVICE} type vcan
sudo ip link set up ${CAN_DEVICE}
sleep 1

# 清空CAN总线缓存，重置收发状态 (标准有效命令)
ip link set ${CAN_DEVICE} down > /dev/null 2>&1
ip link set ${CAN_DEVICE} type can bitrate ${CAN_BITRATE} > /dev/null 2>&1
ip link set ${CAN_DEVICE} up   > /dev/null 2>&1
COMMENT

# =====================================================
# 📌 脚本初始化与业务逻辑区 (无需任何修改！)
# =====================================================
echo -e "[$(date '+%H:%M:%S')] ✅ CAN设备模拟器启动，监听总线: ${CAN_DEVICE}"


# ✅ 核心适配：把无空格的RECEIVE_DATA_1，转换成candump带空格的格式
# 实现：1122334455667788 → 11 22 33 44 55 66 77 88
RECEIVE_DATA_1_SPACE=$(echo ${RECEIVE_DATA_1} | sed 's/\(..\)/\1 /g' | sed 's/ $//')

# 无限阻塞监听CAN总线消息，有消息才处理，CPU占用≈0
candump ${CAN_DEVICE} | while read -r can_msg_line; do
    # 统一格式化时间戳
    CURRENT_TIME=$(date '+%H:%M:%S')
    # echo "[$CURRENT_TIME] 📥 接收CAN消息: ${can_msg_line}"

# ===================== 规则1：匹配 RECEIVE_ID_1 + 可选数据 =====================
if [[ -n "${RECEIVE_DATA_1}" ]]; then
    # 精准匹配【指定ID + 指定数据 + 指定8字节长度】
    RECEIVE_DATA_1_SPACE=$(echo ${RECEIVE_DATA_1} | sed 's/\(..\)/\1 /g' | sed 's/ $//')
    if echo "${can_msg_line}" | grep -q "^${CAN_DEVICE}[[:space:]]\+${RECEIVE_ID_1}[[:space:]]\+\[8\][[:space:]]\+${RECEIVE_DATA_1_SPACE}"; then
        echo "[$CURRENT_TIME] ✅ 识别到指定消息 [ID:${RECEIVE_ID_1} DATA:${RECEIVE_DATA_1}]"
        cansend ${CAN_DEVICE} ${RESPONSE_ID_1}#${RESPONSE_DATA_1}
        echo "[$CURRENT_TIME] 📤 发送响应消息 [ID:${RESPONSE_ID_1} DATA:${RESPONSE_DATA_1}]"
        echo "===================================================="
    fi
fi

# ===================== 规则2：匹配 RECEIVE_ID_2 + 可选数据 =====================
if [[ -n "${RECEIVE_DATA_2}" ]]; then
    RECEIVE_DATA_2_SPACE=$(echo ${RECEIVE_DATA_2} | sed 's/\(..\)/\1 /g' | sed 's/ $//')
    if echo "${can_msg_line}" | grep -q "^${CAN_DEVICE}[[:space:]]\+${RECEIVE_ID_2}[[:space:]]\+\[0-9\]\+[[:space:]]\+${RECEIVE_DATA_2_SPACE}"; then
        echo "[$CURRENT_TIME] ✅ 识别到指定消息 [ID:${RECEIVE_ID_2} DATA:${RECEIVE_DATA_2}]"
        cansend ${CAN_DEVICE} ${RESPONSE_ID_2}#${RESPONSE_DATA_2}
        echo "[$CURRENT_TIME] 📤 发送响应消息 [ID:${RESPONSE_ID_2} DATA:${RESPONSE_DATA_2}]"
        echo "===================================================="
    fi
fi


done