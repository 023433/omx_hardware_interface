#include "omx_hardware_interface/omx_hardware_interface.hpp"

using namespace omx_hardware_interface;

// 노드를 매개변수로 받는 함수 정의
void use_node_example(const rclcpp::Node::SharedPtr& node) {
    // 파라미터 선언 및 기본값 설정
    node->declare_parameter<int>("example_param", 42);

    // 파라미터 값 가져오기
    int64_t param_value = node->get_parameter("example_param").as_int();

    // 로그 출력
    RCLCPP_INFO(node->get_logger(), "Parameter value: %ld", param_value);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 노드 생성
    auto node = rclcpp::Node::make_shared("example_node");
    HardwareInterface har(node);

    har.read();

    // 함수 호출
    use_node_example(node);

    rclcpp::shutdown();
    return 0;
}