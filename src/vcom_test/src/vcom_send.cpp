#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "vcom_interfaces/msg/vcom_data.hpp" 

using namespace std::chrono_literals;

class TestSend : public rclcpp::Node {
public:
  TestSend() : Node("vcom_test_send") {
        publisher_ = this->create_publisher<vcom_interfaces::msg::VCOMData>("send_data", 1);
        std::string s;
        std::cout << "输入任意内容后回车发送一条消息" << std::endl;
        while (rclcpp::ok()) {
            auto message = vcom_interfaces::msg::VCOMData();
            message.id = 8;
            message.func_code = 8;
            message.len = 6;
            message.data = std::vector<uint8_t> {1, 1, 4, 5, 1, 4};
            publisher_->publish(message);
            std::cin >> s;
        }
    }
private:
    rclcpp::Publisher<vcom_interfaces::msg::VCOMData>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSend>());
  rclcpp::shutdown();
  return 0;
}
