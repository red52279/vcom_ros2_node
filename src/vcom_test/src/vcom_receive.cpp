#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "vcom_interfaces/msg/vcom_data.hpp" 

struct VCOMData { // 自定义的通讯数据
    uint8_t func_code;
    uint16_t id;
    uint16_t len;
    std::vector<uint8_t> data;
};

std::ostream& operator<<(std::ostream& cout, VCOMData &data) {
    std::cout << "func_code : " << (unsigned int)data.func_code << std::endl;
    std::cout << "id : " << data.id << std::endl;
    std::cout << "len : " << data.len << std::endl;
    std::cout << "data :" ;
    for (long unsigned int i = 0; i < data.data.size(); i++)
        std::cout << " " << std::hex << (unsigned int) (unsigned char)data.data[i];
    return cout;
}


using namespace std::chrono_literals;
using std::placeholders::_1;

class TestReceive : public rclcpp::Node {
public:
  TestReceive() : Node("vcom_test_receive") {
        subscription_ = this->create_subscription<vcom_interfaces::msg::VCOMData>(
            "receive_data", 1, std::bind(&TestReceive::topic_callback, this, _1));
    }
private:
    rclcpp::Subscription<vcom_interfaces::msg::VCOMData>::SharedPtr subscription_;

    void topic_callback(const vcom_interfaces::msg::VCOMData &msg) {
        VCOMData data;
        data.id = msg.id;
        data.func_code = msg.func_code;
        data.len = msg.len;
        data.data = msg.data;
        std::cout << "receive : \n" << data << std::endl;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestReceive>());
  rclcpp::shutdown();
  return 0;
}
