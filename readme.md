ros2 虚拟串口通讯包（含CRC16校验）

可自动查找串口设备（ttyACM）

main.cpp中有使用实例（连接串口设备程序为收到什么发回什么）

本串口通讯是在Ubuntu中使用的，如使用其他linux发行版，请更改VCOM.h中的 DEVICES_PATH 路径 （到devices即可）

数据包结构为 ： 包头（0x5a） 功能码（一位） ID（两位） 数据位长度（两位） 数据位  CRC_16校验位（两位）

vcom_test 为发送与接收示例

使用colcon build