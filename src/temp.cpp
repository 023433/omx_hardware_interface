#include "omx_hardware_interface/omx_hardware_interface.hpp"

using namespace omx_hardware_interface;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 노드 생성
    auto node = rclcpp::Node::make_shared("example_node");
    HardwareInterface har(node);

    har.read();

    rclcpp::shutdown();
    return 0;
}