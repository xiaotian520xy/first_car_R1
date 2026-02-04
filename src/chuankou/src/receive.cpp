#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/char.hpp"
#include <vector>
class SerialReceive : public rclcpp::Node{

public:
    SerialReceive():Node("serial_receive")
    {
        aruco_pub = this->create_publisher<std_msgs::msg::Char>("/aruco_flag", 10);
        try
        {
            my_serial = std::make_shared<serial::Serial>("/dev/ros2_", 115200, serial::Timeout::simpleTimeout(100));
            if(!my_serial->isOpen())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to opened serial");
            }
            RCLCPP_INFO(this->get_logger(), "sucessful opened serial");
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "serial error: %s", e.what());
        }
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialReceive::timer_callback, this));
    }
private:
    
    void pub_flag(uint8_t num)
    {
        auto msg = std_msgs::msg::Char();
        msg.data = num;
        aruco_pub->publish(msg);
    }

    void timer_callback()
    {
        try{
            if(my_serial->available() > 0)
            {
                size_t available_bytes = my_serial->available();
                std::vector<uint8_t> buffer(available_bytes);
                size_t bytes_read = my_serial->read(buffer.data(), buffer.size());
                if(bytes_read == 0 || buffer.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "no data");
                }
                if(buffer[0] == 0xAA && buffer.back() == 0xBB)
                {
                    uint8_t num = buffer[1];
                    switch (num)
                    {
                        case 0:{
                            pub_flag(num);
                            break;
                        }
                        case 1:{
                            pub_flag(num);
                            break;
                        }
                        case 2:{
                            pub_flag(num);
                            break;
                        }
                        default:{
                            RCLCPP_WARN(this->get_logger(), "未知数据包长度");
                            break;
                        }
                    }
                }
            }
        }catch(const std::exception& e){
            RCLCPP_ERROR(this->get_logger(), "failed to read serial data: %s", e.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr aruco_pub;
    std::shared_ptr<serial::Serial> my_serial;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReceive>());
    rclcpp::shutdown();
    return 0;
}