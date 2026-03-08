#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/char.hpp"
#include "serial/serial.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <mutex>
#include <chrono>
#include <cmath>
#include <array>

class CombinedPublisher : public rclcpp::Node
{
public:
    CombinedPublisher()
        : Node("combined_publisher")
    {
        // 初始化时间
        last_send_time_ = this->now();
        last_position_callback_time_ = this->now();
        // 初始化订阅
        position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/r2_position", 10,
            std::bind(&CombinedPublisher::position_callback, this, std::placeholders::_1));
        yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/now_yaw", 10,
            std::bind(&CombinedPublisher::yaw_callback, this, std::placeholders::_1));
        aruco_sub_ = this->create_subscription<std_msgs::msg::Char>(
            "/connect_aruco", 10,
            std::bind(&CombinedPublisher::connect_callback, this, std::placeholders::_1));
        R1_path_sub_ = this->create_subscription<std_msgs::msg::Char>(
            "/R1_path", 10,
            std::bind(&CombinedPublisher::R1_path_callback, this, std::placeholders::_1));
        // 初始化定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CombinedPublisher::timer_callback, this));
        // 串口重启机制
        if (!open_serial_port())
        {
            start_serial_retry_timer();
        }
    }

private:
    struct Data
    {
        float position_x = 0.0;
        float position_y = 0.0;
        float position_z = 0.0;
        float yaw = 0.0;
        uint8_t connect = 0;
        uint8_t R1_path = 0;
        bool restart_flag = true;
    };

    Data data_;
    rclcpp::Time last_send_time_;
    rclcpp::Time last_position_callback_time_;
    rclcpp::TimerBase::SharedPtr serial_retry_timer_;
    uint8_t serial_retry_count_;
    // 检测串口是否开启
    bool open_serial_port()
    {
        try
        {
            serial_port_.setPort("/dev/ttyUSB0");
            serial_port_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(to);
            serial_port_.open();
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
            if (serial_retry_timer_)
            {
                serial_retry_timer_->cancel();
            }
            return true;
        }
        catch (serial::IOException &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to open serial port: %s", e.what());
            return false;
        }
    }
    // 开启串口重试定时器
    void start_serial_retry_timer()
    {
        serial_retry_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]()
            {
                if (serial_retry_count_++ < 10)
                { // 最多重试10次
                    if (open_serial_port())
                    {
                        serial_retry_timer_->cancel();
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Max retry count reached. Giving up.");
                    serial_retry_timer_->cancel();
                }
            });
    }
    // 位置订阅回调
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_position_callback_time_ = this->now();
        data_.position_x = msg->x;
        data_.position_y = msg->y;
        data_.position_z = msg->z;
    }

    void yaw_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.yaw = msg->data;
    }

    void connect_callback(const std_msgs::msg::Char::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.connect = msg->data;
    }

    void R1_path_callback(const std_msgs::msg::Char::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.R1_path = msg->data;
    }

    // 定时器回调
    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        send_data_to_serial();
    }
    // 串口发送函数
    void send_data_to_serial()
    {
        auto now = this->now();
        if ((now - last_send_time_).seconds() < 0.05)
        {
            return;
        }
        // 是否需要重启雷达
        if ((now - last_position_callback_time_).seconds() > 1)
        {
            data_.restart_flag = true;
        }
        else
        {
            if ((data_.position_x == data_.position_y) == 0)
            {
                data_.restart_flag = true;
            }
            if (data_.position_x > 100.0f || data_.position_y > 100.0f)
            {
                data_.restart_flag = true;
            }
            else
            {
                data_.restart_flag = false;
            }
        }

        std::array<uint8_t, 22> frame;
        frame[0] = 0xFF;

        size_t offset = 1;
        auto copy_float = [&](float value)
        {
            uint8_t *bytes = reinterpret_cast<uint8_t *>(&value);
            for (int i = 0; i < 4; i++)
            {
                frame[offset++] = bytes[i];
            }
        };

        copy_float(data_.position_x);
        copy_float(data_.position_y);
        copy_float(data_.position_z);
        copy_float(data_.yaw);
        frame[offset++] = data_.connect;
        frame[offset++] = data_.R1_path;
        frame[offset++] = data_.restart_flag;

        // 计算校验和
        uint8_t checksum = 0;
        for (size_t i = 1; i < frame.size() - 2; i++)
        {
            checksum += frame[i];
        }
        frame[offset++] = checksum;
        frame[offset++] = 0xFE;

        if (serial_port_.isOpen())
        {
            try
            {
                serial_port_.write(frame.data(), frame.size());

                RCLCPP_INFO(this->get_logger(), "Sent data: x=%.5f, y=%.5f, z=%.5f, yaw=%.5f, connect=%d, r1_path=%d, restart=%d",
                            data_.position_x, data_.position_y, data_.position_z, data_.yaw,
                            data_.connect, data_.R1_path, data_.restart_flag);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
            }
        }
        last_send_time_ = now;
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr aruco_sub_;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr R1_path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_port_;
    std::mutex mutex_; // 线程锁
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CombinedPublisher>());
    rclcpp::shutdown();
    return 0;
}