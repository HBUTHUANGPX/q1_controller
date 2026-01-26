#ifndef SOCKET_CAN_HPP_
#define SOCKET_CAN_HPP_

#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h> // 新增：关键修复 ifreq结构体的完整定义
#include <string>
#include <sys/ioctl.h>  // 新增：ioctl系统调用头文件
#include <sys/socket.h> // 新增：补全socket相关头文件
#include <unistd.h>     // 新增：close/write/read 系统调用头文件

class SocketCAN {
  public:
    /**
     * @brief 构造函数
     * @param can_iface CAN接口名 can0/can1
     * @param bitrate CAN波特率，灵巧手默认1000000(1Mbps)
     */
    explicit SocketCAN(const std::string &can_iface, uint32_t bitrate = 1000000);
    ~SocketCAN();

    /**
     * @brief 初始化CAN总线
     * @return true-成功 false-失败
     */
    bool init();

    /**
     * @brief 关闭CAN总线
     */
    void close();

    /**
     * @brief 发送CAN帧
     * @param can_id CAN帧ID
     * @param data 发送数据缓冲区
     * @param len 数据长度(0~8)
     * @return true-成功 false-失败
     */
    bool send_frame(uint32_t can_id, const uint8_t *data, uint8_t len);
    bool send_multi_frame(uint32_t can_id, const uint8_t *data, size_t data_len);

    /**
     * @brief 接收CAN帧 (非阻塞模式)
     * @param frame 接收的CAN帧数据
     * @return true-接收到数据 false-无数据/失败
     */
    bool recv_frame(struct can_frame &frame);

    /**
     * @brief 获取CAN接口名
     */
    std::string get_interface() const
    {
        return can_iface_;
    }

  private:
    std::string can_iface_; // CAN接口名 can0/can1
    uint32_t bitrate_;      // CAN波特率
    int sock_fd_ = -1;      // CAN套接字描述符
    struct sockaddr_can addr_{};
    struct ifreq ifr_{};
};

#endif // SOCKET_CAN_HPP_