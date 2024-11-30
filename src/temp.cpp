#include "omx_hardware_interface/omx_hardware_interface.hpp"

using namespace omx_hardware_interface;

void timerCallback(
  std::shared_ptr<HardwareInterface> hardware_interface,
  rclcpp::Time &last_time,
  rclcpp::Clock::SharedPtr clock
){
  rclcpp::Time curr_time = clock->now();
  rclcpp::Duration elapsed_time = curr_time - last_time;
  last_time = curr_time;

  // hardware_interface->read();
  // hardware_interface->write();
}

int main(int argc, char **argv){
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<rclcpp::Node>("open_manipulator_x_hardware");

  // Initialize hardware interface and controller manager
  auto hardware_interface = std::make_shared<HardwareInterface>(node);

  // Timer variables
  rclcpp::Time last_time = node->now();
  auto clock = node->get_clock();

  // Create timer with 10ms interval
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(10),
    [hardware_interface,  &last_time, clock]() {
      timerCallback(hardware_interface,  last_time, clock);
    }
  );

  // Spin
  rclcpp::spin(node);
  return 0;
}