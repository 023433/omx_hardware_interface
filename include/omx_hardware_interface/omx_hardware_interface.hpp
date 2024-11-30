// Copyright (c) 2024, 
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OMX_HARDWARE_INTERFACE__OMX_HARDWARE_INTERFACE_HPP_
#define OMX_HARDWARE_INTERFACE__OMX_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

typedef struct _ItemValue{
  std::string item_name;
  int32_t value;
} ItemValue;

typedef struct _WayPoint{
  double position;
  double velocity;
  double acceleration;
} WayPoint;

typedef struct _Joint{
  double position;
  double velocity;
  double current;
  double effort;
  double position_command;
  double velocity_command;
  double effort_command;
} Joint;

namespace omx_hardware_interface{

class HardwareInterface : public hardware_interface::ResourceManager{

public:
  HardwareInterface(rclcpp::Node::SharedPtr& ne);
  ~HardwareInterface() {}
  void read();
  void write();

private:
  void registerActuatorInterfaces();
  void registerControlInterfaces();
  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);

  rclcpp::Node::SharedPtr& node_;
  rclcpp::Logger logger_;

  // ROS Parameters
  std::string port_name_;
  int64_t baud_rate_;
  std::string yaml_file_;
  std::string _interface;

  // Variables
  DynamixelWorkbench *dxl_wb_;
  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  std::vector<Joint> joints_;

  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_effort_commands_;

};

}  // namespace omx_hardware_interface

#endif  // OMX_HARDWARE_INTERFACE__OMX_HARDWARE_INTERFACE_HPP_
