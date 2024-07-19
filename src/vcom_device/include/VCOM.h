#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iconv.h>
#include <iostream>
#include <dirent.h>
#include <vector>
#include "CRC.h"
#include <regex>
#include <vector>

#define BUFFER_SIZE 64
#define DEVICES_PATH "/sys/devices/pci0000:00"

namespace VCOMCOMM {

constexpr int TYPE_NUMBER = 6;
const std::string DEVICES_TYPE[TYPE_NUMBER] = {
        "product",
        "version",
        "manufacturer",
        "idProduct",
        "idVendor",
        "serial" 
};

struct VCOMData { // 自定义的通讯数据
    uint8_t func_code;
    uint16_t id;
    uint16_t len;
    std::vector<uint8_t> data;
};

struct uart_data {
    int fd;
    int baud_rate;
    int data_bits;
    int stop_bits;
    char parity;
};

struct device_data {
    bool operator <(const device_data & a) const {
        return idProduct < a.idProduct;
    }
    bool operator ==(const device_data & a) const {
        return (this->product == a.product && 
            this->version == a.version &&
            this->manufacturer == a.manufacturer &&
            this->idProduct == a.idProduct &&
            this->idVendor == a.idVendor &&
            this->serial == a.serial);
    }
    bool empty() {
        return this->product.empty() && 
            this->version.empty() &&
            this->manufacturer.empty() &&
            this->idProduct.empty() &&
            this->idVendor.empty() &&
            this->serial.empty() &&
            this->port.empty();
    }
    std::string product;
    std::string version;
    std::string manufacturer;
    std::string idProduct;
    std::string idVendor;
    std::string serial;
    std::string port;
};
std::ostream& operator<<(std::ostream& cout, device_data &data);
std::ostream& operator<<(std::ostream& cout, VCOMData &data);
class VCOMCOMM {
private:
    device_data ListDevicesInformation(std::string path); // 读取当前路径下的设备信息，并存储
    char m_buffer[64]; // 接收缓冲区
    int SetUartConfig() const; // 配置串口
public:
    ~VCOMCOMM() {
        close(m_uart_data.fd);
    }
    int OpenPort(const char* dev = "", int baud_rate = 115200, int data_bits = 8,
                    char parity = 'S', int stop_bits = 1); // 打开串口
    int FindConnectableDeviceDir(const std::string base_path = DEVICES_PATH, 
                                device_data data = device_data()); // 寻找可用设备

    int PortRead(); // 已识别设备后读取通讯信息
    void Transmit(VCOMData data); // 已识别设备后传输通讯信息
    device_data m_device_data; // 要连接的设备信息
    VCOMData m_data; // 发送/接受的信息
    uart_data m_uart_data;  // 串口传输相关配置
    std::vector<device_data> m_connectable_port_devices; // 检索到的可连接设备
};
}

