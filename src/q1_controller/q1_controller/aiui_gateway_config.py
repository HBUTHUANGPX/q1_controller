AIUI_PORT = 19199
TARGET_IP = "192.168.50.6"   # 现在开发板的 IP
TRUSTED_MAC = "2a:de:ef:f7:8d:6b".lower()      #无线mac

TOPIC_TTS = "/tts_say"

# 交互事件输出（可选，但推荐预留）
TOPIC_AIUI_EVENT  = "/aiui/event"
TOPIC_AIUI_IAT    = "/aiui/iat"
TOPIC_AIUI_NLP    = "/aiui/nlp"
TOPIC_AIUI_INTENT = "/aiui/intent"
