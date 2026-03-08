#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "chuankou/autopath.h"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/bool.hpp"
#include "fyt_msg/msg/int_array.hpp"
#include <vector>
#include <string>
class SerialReceive : public rclcpp::Node{

public:
    SerialReceive():Node("serial_receive")
    {
        aruco_pub = this->create_publisher<std_msgs::msg::Char>("/aruco_flag", 10);
        array_pub = this->create_publisher<fyt_msg::msg::IntArray>("/path_result", 10);
        flag_pub = this->create_publisher<std_msgs::msg::Bool>("/path_flag", 10);
        R1_path_pub = this->create_publisher<std_msgs::msg::Char>("R1_path", 10);
        try
        {
            my_serial = std::make_shared<serial::Serial>("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(100));
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
        grid = std::vector<std::vector<BlockType>>(4, std::vector<BlockType>(3, EMPTY));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialReceive::timer_callback, this));
    }
private:
    std::vector<uint8_t> receive_block;//接收块列表
    std::vector<uint8_t> tramsfrom_block;//转换块列表
    std::vector<uint8_t> R1Buffer;//r1块储存列表
    std::vector<uint8_t> R2Buffer;//r2块储存列表
    std::vector<uint8_t> FaBuffer;//假块列表
    std::vector<int> path;//最终路径结果
    std::vector<std::vector<int>> mapID = {{},
            {0,2},{0,1},{0,0},
            {1,2},{1,1},{1,0},
            {2,2},{2,1},{2,0},
            {3,2},{3,1},{3,0}
        };//id在地图的位置，参考autopath.h
    std::vector<std::vector<BlockType>> grid;

    void transform_block(std::vector<uint8_t> old_buffer)
    {
        if(tramsfrom_block.size() != 0)
        {
            tramsfrom_block.clear();
        }
        for(uint8_t i = 1; i <=4; i++)
        {
            for(uint8_t j = 1; j <=3; j++)
            {
                uint8_t id = 4*j - i;
                tramsfrom_block.push_back(old_buffer[id]);
            }
        }
    }

    void allocation_block()
    {
        if(tramsfrom_block.size() == 12)
        {
            for(uint8_t i = 0; i < 12; i++)
            {
                uint8_t num = tramsfrom_block[i];
                if(num == 1)
                {
                    R1Buffer.push_back(i + 1);
                }
                else if(num == 2)
                {
                    R2Buffer.push_back(i + 1);
                }
                else if(num == 3)
                {
                    FaBuffer.push_back(i + 1);
                }
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "数据长度错误");
        }
    }

    template<typename T>
    bool contains(const std::vector<T>& vec, const T& value) {
        return std::find(vec.begin(), vec.end(), value) != vec.end();
    }

    void setGrid() {
        for(uint8_t i = 1; i < 13; i++){
            uint8_t value = static_cast<uint8_t>(i);
            
            if(contains(R1Buffer, value)){
                grid[mapID[i][0]][mapID[i][1]] = R1;
            }
            else if(contains(R2Buffer, value)){
                grid[mapID[i][0]][mapID[i][1]] = R2;
            }
            else if(contains(FaBuffer, value)){
                grid[mapID[i][0]][mapID[i][1]] = FA;
            }
            else{
                grid[mapID[i][0]][mapID[i][1]] = EMPTY;
            }
        }
    }

    uint8_t get_r1_path()
    {
        uint8_t k, l = 0;
        uint8_t r2_path = path[0];
        for(uint8_t id : R1Buffer)
        {
            if((id - r2_path) % 3 == 0){
                k++;
            }
            if(id % 3 == 0){
                l++;
            }
        }
        if(k > 0){
            return r2_path;
        }else{
            int8_t dif = R1Buffer[0] - r2_path;
            if(l > 0){
                return 3;
            }else if(dif == -1){
                return 2;
            }else if(dif == -2 || dif % 3 == 1){
                return 1;
            }else{
                return 0;
            }
        }
    }

    void get_result()
    {
        setGrid();
        AutoPath ap;
        PathResult result = ap.planBestPath(grid);//执行算法
        path = ap.outPutPath(result);//输出结果
        if (contains(R1Buffer, static_cast<uint8_t>(1)) && path[0] == 1) {
            path.insert(path.begin() + 1, 1);
        }
        else if (contains(R1Buffer, static_cast<uint8_t>(2)) && path[0] == 2) {
            path.insert(path.begin() + 1, 1);
        }
        else if (contains(R1Buffer, static_cast<uint8_t>(3)) && path[0] == 3) {
            path.insert(path.begin() + 1, 1);
        }
        else {
            path.insert(path.begin() + 1, 0);
        }
        path.insert(path.begin() + 1, get_r1_path());
        path.insert(path.begin(), 0xAA);
        path.push_back(0xBB);  
    }

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
                    bool flag = buffer[2];
                    auto flag_msg = std_msgs::msg::Bool();
                    flag_msg.data = flag;
                    flag_pub->publish(flag_msg);
                    
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
                if(buffer[0] == 0xEF && buffer.back() == 0xFE)
                {
                    std::string array_str;
                    receive_block = std::vector<uint8_t>(buffer.begin() + 1, buffer.end() - 1);
                    transform_block(receive_block);
                    allocation_block();
                    get_result();
                    fyt_msg::msg::IntArray msg;
                    for (const auto& value : path) {
                        msg.data.push_back(value);
                        array_str += std::to_string(value) + " ";
                    }
                    array_pub->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "get array:");
                    RCLCPP_INFO(this->get_logger(), "%s", array_str.c_str());
                    auto r1_msg = std_msgs::msg::Char();
                    r1_msg.data = path[2];
                    R1_path_pub->publish(r1_msg);
                    path.clear();
                    R1Buffer.clear();
                    R2Buffer.clear();
                    FaBuffer.clear();
                }
            }
        }catch(const std::exception& e){
            RCLCPP_ERROR(this->get_logger(), "failed to read serial data: %s", e.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr aruco_pub;
    rclcpp::Publisher<fyt_msg::msg::IntArray>::SharedPtr array_pub;
    std::shared_ptr<serial::Serial> my_serial;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr R1_path_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReceive>());
    rclcpp::shutdown();
    return 0;
}