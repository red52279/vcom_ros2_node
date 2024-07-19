#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "VCOM.h"
#include "vcom_interfaces/msg/vcom_data.hpp" 

using namespace std::chrono_literals;
using std::placeholders::_1;


class VCOM : public rclcpp::Node {
public:
  VCOM() : Node("vcom_io") {
        publisher_ = this->create_publisher<vcom_interfaces::msg::VCOMData>("receive_data", 1);
        
        subscription_ = this->create_subscription<vcom_interfaces::msg::VCOMData>(
            "send_data", 1, std::bind(&VCOM::topicCallback, this, _1));

        int is_find_device = COM.FindConnectableDeviceDir();
        
        if (is_find_device > 0) {
            ReceiveThread = std::thread(&VCOM::VCOMPortRead, this);
        } else {
            rclcpp::shutdown();
        }
    }
    ~VCOM() {
        if (ReceiveThread.joinable()) {
            ReceiveThread.join();
        }
    }
private:
    std::thread ReceiveThread; 
    
    rclcpp::Publisher<vcom_interfaces::msg::VCOMData>::SharedPtr publisher_;
    rclcpp::Subscription<vcom_interfaces::msg::VCOMData>::SharedPtr subscription_;

    VCOMCOMM::VCOMCOMM COM;

    void topicCallback(const vcom_interfaces::msg::VCOMData &msg) {
        VCOMCOMM::VCOMData data;
        data.id = msg.id;
        data.func_code = msg.func_code;
        data.len = msg.len;
        data.data = msg.data;
        COM.Transmit(data);
    }

    void VCOMPortRead() {
        while (rclcpp::ok()) {
            if (COM.PortRead() > 0) {
                VCOMCOMM::VCOMData data = this->COM.m_data;
                std::cout << "receive : \n" << data << std::endl;
                auto message = vcom_interfaces::msg::VCOMData();
                message.id = data.id;
                message.func_code = data.func_code;
                message.len = data.len;
                message.data = data.data;
                this->publisher_->publish(message);
            }
        }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VCOM>());
  rclcpp::shutdown();
  return 0;
}
