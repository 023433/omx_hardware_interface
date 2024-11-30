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

#include <limits>
#include <vector>

#include "omx_hardware_interface/omx_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omx_hardware_interface
{

HardwareInterface::HardwareInterface(rclcpp::Node::SharedPtr& node) : node_(node), logger_(node->get_logger()){
    /************************************************************
  ** Initialize ROS parameters
  ************************************************************/
  // port_name_ = priv_node_handle_.param<std::string>("usb_port", "/dev/ttyUSB0");
  // baud_rate_ = priv_node_handle_.param<int32_t>("baud_rate", 1000000);
  // yaml_file_ = priv_node_handle_.param<std::string>("yaml_file", "");
  // interface_ = priv_node_handle_.param<std::string>("interface", "position");

  /************************************************************
  ** Register Interfaces
  ************************************************************/

  node_->declare_parameter<std::string>("usb_port", "/dev/ttyUSB0");
  node_->declare_parameter<int>("baud_rate", 1000000);
  node_->declare_parameter<std::string>("yaml_file", "");
  node_->declare_parameter<std::string>("interface", "position");

  port_name_ = node->get_parameter("usb_port").as_string();
  baud_rate_ = node->get_parameter("baud_rate").as_int();
  yaml_file_ = node->get_parameter("yaml_file").as_string();
  _interface = node->get_parameter("interface").as_string();


  // 로그 출력
  RCLCPP_INFO(logger_, "usb_port: %s", port_name_.c_str());
  RCLCPP_INFO(logger_, "baud_rate: %ld", baud_rate_);
  RCLCPP_INFO(logger_, "yaml_file: %s", yaml_file_.c_str());
  RCLCPP_INFO(logger_, "interface: %s", _interface.c_str());
  registerActuatorInterfaces();
  // registerControlInterfaces();
}

void HardwareInterface::read(){
  bool result = false;
  const char* log = NULL;
  size_t _dynamixelsize = dynamixel_.size();

  RCLCPP_INFO(logger_, "_dynamixelsize: %ld", _dynamixelsize);

  int32_t get_position[_dynamixelsize];
  int32_t get_velocity[_dynamixelsize];
  int32_t get_current[_dynamixelsize];

  uint8_t id_array[_dynamixelsize];
  std::string name_array[_dynamixelsize];
  uint8_t id_cnt = 0;

  uint8_t sync_read_handler = 0; // 0 for present position, velocity, current

  for(auto const& dxl:dynamixel_){
    name_array[id_cnt] = dxl.first.c_str();
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

  result = dxl_wb_->syncRead(
    sync_read_handler,
    id_array,
    static_cast<uint8_t>(_dynamixelsize),
    &log
  );

  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
  }

  result = dxl_wb_->getSyncReadData(
    sync_read_handler,
    id_array,
    id_cnt,
    control_items_["Present_Current"]->address,
    control_items_["Present_Current"]->data_length,
    get_current,
    &log
  );

  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
  }

  result = dxl_wb_->getSyncReadData(
    sync_read_handler,
    id_array,
    id_cnt,
    control_items_["Present_Velocity"]->address,
    control_items_["Present_Velocity"]->data_length,
    get_velocity,
    &log
  );

  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
  }

  result = dxl_wb_->getSyncReadData(
    sync_read_handler,
    id_array,
    id_cnt,
    control_items_["Present_Position"]->address,
    control_items_["Present_Position"]->data_length,
    get_position,
    &log
  );

  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
  }

  for(uint8_t index = 0; index < id_cnt; index++){
    // Position
    int arIdx = id_array[index]-1;

    joints_[arIdx].position = dxl_wb_->convertValue2Radian((uint8_t)id_array[index], (int32_t)get_position[index]);

    if(strcmp(name_array[index].c_str(), "gripper") == 0){
      joints_[arIdx].position /= 150.0;
    }

    // Velocity
    joints_[arIdx].velocity = dxl_wb_->convertValue2Velocity((uint8_t)id_array[index], (int32_t)get_velocity[index]);

    // Effort
    if(strcmp(dxl_wb_->getModelName((uint8_t)id_array[index]), "XL-320") == 0){
      joints_[arIdx].effort = dxl_wb_->convertValue2Load((int16_t)get_current[index]);
    }else{
      joints_[arIdx].effort = dxl_wb_->convertValue2Current((int16_t)get_current[index]) * (1.78e-03);
    }

    joints_[arIdx].position_command = joints_[arIdx].position;
  }

}

void HardwareInterface::write(){
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  int32_t _dynamixelposition[dynamixel_.size()];
  int32_t _dynamixeleffort[dynamixel_.size()];

  if(strcmp(_interface.c_str(), "position") == 0){
    for(auto const& dxl:dynamixel_){
      id_array[id_cnt] = (uint8_t)dxl.second;
      _dynamixelposition[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)dxl.second, static_cast<float>(joints_[(uint8_t)dxl.second-1].position_command));

      if(strcmp(dxl.first.c_str(), "gripper") == 0){
        _dynamixelposition[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)dxl.second, static_cast<float>(joints_[(uint8_t)dxl.second-1].position_command * 150.0));
      }
      id_cnt ++;
    }
    uint8_t sync_write_handler = 0; // 0: position, 1: velocity, 2: effort
    result = dxl_wb_->syncWrite(sync_write_handler, id_array, id_cnt, _dynamixelposition, 1, &log);
  }else if(strcmp(_interface.c_str(), "effort") == 0) {
    for(auto const& dxl:dynamixel_){
      id_array[id_cnt] = (uint8_t)dxl.second;
      _dynamixeleffort[id_cnt] = dxl_wb_->convertCurrent2Value((uint8_t)dxl.second, static_cast<float>(joints_[(uint8_t)dxl.second-1].effort_command / (1.78e-03)));

      if(strcmp(dxl.first.c_str(), "gripper") == 0){
        _dynamixelposition[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)dxl.second, static_cast<float>(joints_[(uint8_t)dxl.second-1].position_command * 150.0));
      }
      id_cnt ++;
    }
    uint8_t sync_write_handler = 2; // 0: position, 1: velocity, 2: effort
    result = dxl_wb_->syncWrite(sync_write_handler, id_array, id_cnt, _dynamixeleffort, 1, &log);
  }

  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
  }
}


void HardwareInterface::registerActuatorInterfaces(){  
  dxl_wb_ = new DynamixelWorkbench;

  if(!initWorkbench(port_name_, (uint32_t)baud_rate_)){
    RCLCPP_ERROR(logger_, "Please check USB port name");
    return;
  }

  if(!getDynamixelsInfo(yaml_file_)){
    RCLCPP_ERROR(logger_, "Please check YAML file");
    return;
  }


  if(!loadDynamixels()){
    RCLCPP_ERROR(logger_, "Please check Dynamixel ID or BaudRate");
    return;
  }

  if(!initDynamixels()){
    RCLCPP_ERROR(logger_, "Please check control table (http://emanual.robotis.com/#control-table)");
    return;
  }

  if(!initControlItems()){
    RCLCPP_ERROR(logger_, "Please check control items");
    return;
  }

  if(!initSDKHandlers()){
    RCLCPP_ERROR(logger_, "Failed to set Dynamixel SDK Handler");
    return;
  }
}


void HardwareInterface::registerControlInterfaces(){
  // resize vector
  uint8_t joint_size = 0;
  for(auto const& dxl:dynamixel_){
    if(joint_size < (uint8_t)dxl.second){
      joint_size = (uint8_t)dxl.second;
    }
  }

  joints_.resize(joint_size);

  for(auto iter = dynamixel_.begin(); iter != dynamixel_.end(); iter++){
    // initialize joint vector
    Joint joint;
    joints_[iter->second - 1] = joint;
    RCLCPP_ERROR(logger_, "joint_name : %s, servo ID: %d", iter->first.c_str(), iter->second);

    // // connect and register the joint state interface
    // hardware_interface::CommandInterface joint_state_handle(iter->first.c_str(),
    //                                                         &joints_[iter->second - 1].position,
    //                                                         &joints_[iter->second - 1].velocity,
    //                                                         &joints_[iter->second - 1].effort);
    // joint_state_interface_.registerHandle(joint_state_handle);

    // // connect and register the joint position, velocity and effort interface
    // hardware_interface::JointHandle position_joint_handle(joint_state_handle, &joints_[iter->second - 1].position_command);
    // position_joint_interface_.registerHandle(position_joint_handle);
    // hardware_interface::JointHandle velocity_joint_handle(joint_state_handle, &joints_[iter->second - 1].velocity_command);
    // velocity_joint_interface_.registerHandle(velocity_joint_handle);
    // hardware_interface::JointHandle effort_joint_handle(joint_state_handle, &joints_[iter->second - 1].effort_command);
    // effort_joint_interface_.registerHandle(effort_joint_handle);
  }

  // registerInterface(&joint_state_interface_);
  // registerInterface(&position_joint_interface_);
  // registerInterface(&velocity_joint_interface_);
  // registerInterface(&effort_joint_interface_);
}

bool HardwareInterface::initWorkbench(const std::string port_name, const uint32_t baud_rate){
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
  }

  return result;
}


bool HardwareInterface::getDynamixelsInfo(const std::string yaml_file){
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if(dynamixel.IsNull()){
    RCLCPP_ERROR(logger_, "Please check YAML file");
    return false;
  }

  for(YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++){
    std::string name = it_file->first.as<std::string>();
    if(name.size() == 0){
      continue;
    }

    YAML::Node item = dynamixel[name];
    for(YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++){
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if(item_name == "ID"){
        dynamixel_[name] = value;
      }

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }

  return true;
}

bool HardwareInterface::loadDynamixels(void){
  bool result = false;
  const char* log;

  for(auto const& dxl:dynamixel_){
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if(result == false){
      RCLCPP_ERROR(logger_, "%s", log);
      RCLCPP_ERROR(logger_, "Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }else{
      RCLCPP_INFO(logger_, "Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}


bool HardwareInterface::initDynamixels(void){
  const char* log;

  for(auto const& dxl:dynamixel_){
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for(auto const& info:dynamixel_info_){
      if(dxl.first == info.first){
        if(info.second.item_name != "ID" && info.second.item_name != "Baud_Rate"){
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if(result == false){
            RCLCPP_ERROR(logger_, "%s", log);
            RCLCPP_ERROR(logger_, "Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }
  }

  // Torque On after setting up all servo
  for (auto const& dxl:dynamixel_){
    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}

bool HardwareInterface::initControlItems(void){
  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo((uint8_t)it->second, "Goal_Position");

  if(goal_position == NULL) {
    return false;
  }

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo((uint8_t)it->second, "Goal_Velocity");
  
  if(goal_velocity == NULL){
    goal_velocity = dxl_wb_->getItemInfo((uint8_t)it->second, "Moving_Speed");
  }

  if(goal_velocity == NULL){
    return false;
  }

  const ControlItem *goal_current = dxl_wb_->getItemInfo((uint8_t)it->second, "Goal_Current");
  
  if(goal_current == NULL){
    goal_current = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Load");
  }

  if(goal_current == NULL){
    return false;
  }

  const ControlItem *present_position = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Position");
  
  if(present_position == NULL){
    return false;
  }

  const ControlItem *present_velocity = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Velocity");
  
  if(present_velocity == NULL){
    present_velocity = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Speed");
  }

  if(present_velocity == NULL){
    return false;
  }

  const ControlItem *present_current = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Current");
  
  if(present_current == NULL){
    present_current = dxl_wb_->getItemInfo((uint8_t)it->second, "Present_Load");
  }

  if(present_current == NULL){
    return false;
  }

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;
  control_items_["Goal_Current"] = goal_current;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}


bool HardwareInterface::initSDKHandlers(void){
  bool result = false;
  const char* log = NULL;

  dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);

  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
    return result;
  }

  RCLCPP_INFO(logger_, "%s", log);

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);

  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
    return result;
  }

  RCLCPP_INFO(logger_, "%s", log);

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
  
  if(result == false){
    RCLCPP_ERROR(logger_, "%s", log);
    return result;
  }

  RCLCPP_INFO(logger_, "%s", log);

  if(dxl_wb_->getProtocolVersion() == 2.0f){
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /* As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.*/
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    int read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + 2;

    result = dxl_wb_->addSyncReadHandler(
      start_address,
      (uint16_t)read_length,
      &log
    );

    if(result == false){
      RCLCPP_ERROR(logger_, "%s", log);
      return result;
    }
  }

  return result;
}



}  // namespace omx_hardware_interface
