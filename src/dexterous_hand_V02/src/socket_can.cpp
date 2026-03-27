#include "dexterous_hand/socket_can.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>

SocketCAN::SocketCAN(const std::string &can_iface, uint32_t bitrate) : can_iface_(can_iface), bitrate_(bitrate)
{
}

SocketCAN::~SocketCAN()
{
    close();
}

bool SocketCAN::init()
{
    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd_ < 0)
    {
        std::cerr << "[CAN] Create socket fail: " << strerror(errno) << std::endl;
        return false;
    }
    strcpy(ifr_.ifr_name, can_iface_.c_str());
    if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr_) < 0)
    {
        std::cerr << "[CAN] Get index fail: " << strerror(errno) << std::endl;
        close();
        return false;
    }
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(sock_fd_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0)
    {
        std::cerr << "[CAN] Bind fail: " << strerror(errno) << std::endl;
        close();
        return false;
    }
    int flags = fcntl(sock_fd_, F_GETFL, 0);
    // fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);
    fcntl(sock_fd_, F_SETFL, flags);
    std::cout << "[CAN] Init OK: " << can_iface_ << std::endl;
    return true;
}

void SocketCAN::close()
{
    if (sock_fd_ >= 0)
    {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

bool SocketCAN::send_frame(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (sock_fd_ < 0 || len > 8 || !data)
        return false;
    struct can_frame frame{};
    frame.can_id = can_id;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);
    return write(sock_fd_, &frame, sizeof(frame)) == sizeof(frame);
}

/**
 * @brief 通用 CAN 发送函数（参数化 + 自动分帧）
 * @param can_id 要发送的 CAN ID（标准11位）
 * @param data 待发送的字节数据
 * @param data_len 待发送数据的总长度
 */
bool SocketCAN::send_multi_frame(uint32_t can_id, const uint8_t *data, size_t data_len)
{
    if (data == nullptr || data_len == 0)
    {
        return false;
    }

    // 计算需要拆分的帧数（每帧最多8字节，向上取整）
    size_t total_frames = (data_len + 7) / 8;
    // RCLCPP_INFO(this->get_logger(), "待发送数据长度: %zu 字节，将拆分为 %zu 帧发送 (CAN ID: 0x%03X)", data_len, total_frames,
    // can_id);

    for (size_t frame_idx = 0; frame_idx < total_frames; frame_idx++)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        // 基础配置
        frame.can_id = can_id;
        // 当前帧的起始位置和有效长度
        size_t start_pos = frame_idx * 8;
        size_t current_len = std::min((size_t)8, data_len - start_pos);
        frame.can_dlc = current_len;

        // 拷贝当前帧数据
        memcpy(frame.data, data + start_pos, current_len);

        // 发送帧
        ssize_t nbytes = write(sock_fd_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame))
        {
            std::cerr << "[CAN] send fail: frame " << (frame_idx + 1) << ", actual send " << nbytes << " bytes" << std::endl;
        }
        else
        {
            // 打印当前帧详情（仅显示有效数据，避免空字节干扰）
            std::string data_str;
            for (size_t i = 0; i < current_len; i++)
            {
                char buf[5];
                snprintf(buf, sizeof(buf), "0x%02x", frame.data[i]);
                data_str += buf;
            }
            // RCLCPP_INFO(this->get_logger(), "发送 CAN 帧 -> ID: 0x%03X, 长度: %d, 数据: %s", frame.can_id, frame.can_dlc,
            //             data_str.c_str());
        }

        // 可选：分帧发送时添加微小延时，避免总线拥塞
        usleep(200); // 0.2ms 延时，可根据实际调整
    }
    return true;
}

bool SocketCAN::recv_frame(struct can_frame &frame)
{
    if (sock_fd_ < 0)
        return false;
    memset(&frame, 0, sizeof(frame));
    return read(sock_fd_, &frame, sizeof(frame)) == sizeof(frame);
}