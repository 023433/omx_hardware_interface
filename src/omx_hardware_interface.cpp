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

HardwareInterface::HardwareInterface(rclcpp::Node::SharedPtr& node) : _node(node), _logger(node->get_logger()){
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

  _node->declare_parameter<std::string>("usb_port", "/dev/ttyUSB0");
  _node->declare_parameter<int>("baud_rate", 1000000);
  _node->declare_parameter<std::string>("yaml_file", "");
  _node->declare_parameter<std::string>("interface", "position");

  _port_name = node->get_parameter("usb_port").as_string();
  _baud_rate = node->get_parameter("baud_rate").as_int();
  _yaml_file = node->get_parameter("yaml_file").as_string();
  _interface = node->get_parameter("interface").as_string();


  // 로그 출력
  RCLCPP_INFO(_node->get_logger(), "usb_port: %s", _port_name.c_str());
  RCLCPP_INFO(_node->get_logger(), "baud_rate: %ld", _baud_rate);
  RCLCPP_INFO(_node->get_logger(), "yaml_file: %s", _yaml_file.c_str());
  RCLCPP_INFO(_node->get_logger(), "interface: %s", _interface.c_str());
  registerActuatorInterfaces();
  // registerControlInterfaces();
}

void HardwareInterface::read(){
  bool result = false;
  const char* log = NULL;
  size_t _dynamixelsize = _dynamixel.size();

  RCLCPP_INFO(_node->get_logger(), "_dynamixelsize: %ld", _dynamixelsize);

  int32_t get_position[_dynamixelsize];
  int32_t get_velocity[_dynamixelsize];
  int32_t get_current[_dynamixelsize];

  uint8_t id_array[_dynamixelsize];
  std::string name_array[_dynamixelsize];
  uint8_t id_cnt = 0;

  uint8_t sync_read_handler = 0; // 0 for present position, velocity, current

  for(auto const& dxl:_dynamixel){
    name_array[id_cnt] = dxl.first.c_str();
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

  result = _dxl_wb->syncRead(
    sync_read_handler,
    id_array,
    static_cast<uint8_t>(_dynamixelsize),
    &log
  );

  if(result == false){
    RCLCPP_ERROR(_logger, "%s", log);
  }

  result = _dxl_wb->getSyncReadData(
    sync_read_handler,
    id_array,
    id_cnt,
    _control_items["Present_Current"]->address,
    _control_items["Present_Current"]->data_length,
    get_current,
    &log
  );

  if(result == false){
    RCLCPP_ERROR(_logger, "%s", log);
  }

  result = _dxl_wb->getSyncReadData(
    sync_read_handler,
    id_array,
    id_cnt,
    _control_items["Present_Velocity"]->address,
    _control_items["Present_Velocity"]->data_length,
    get_velocity,
    &log
  );

  if(result == false){
    RCLCPP_ERROR(_logger, "%s", log);
  }

  result = _dxl_wb->getSyncReadData(
    sync_read_handler,
    id_array,
    id_cnt,
    _control_items["Present_Position"]->address,
    _control_items["Present_Position"]->data_length,
    get_position,
    &log
  );

  if(result == false){
    RCLCPP_ERROR(_logger, "%s", log);
  }

  for(uint8_t index = 0; index < id_cnt; index++){
    // Position
    int arIdx = id_array[index]-1;

    _joints[arIdx].position = _dxl_wb->convertValue2Radian((uint8_t)id_array[index], (int32_t)get_position[index]);

    if(strcmp(name_array[index].c_str(), "gripper") == 0){
      _joints[arIdx].position /= 150.0;
    }

    // Velocity
    _joints[arIdx].velocity = _dxl_wb->convertValue2Velocity((uint8_t)id_array[index], (int32_t)get_velocity[index]);

    // Effort
    if(strcmp(_dxl_wb->getModelName((uint8_t)id_array[index]), "XL-320") == 0){
      _joints[arIdx].effort = _dxl_wb->convertValue2Load((int16_t)get_current[index]);
    }else{
      _joints[arIdx].effort = _dxl_wb->convertValue2Current((int16_t)get_current[index]) * (1.78e-03);
    }

    _joints[arIdx].position_command = _joints[arIdx].position;
  }

}

void HardwareInterface::write(){
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[_dynamixel.size()];
  uint8_t id_cnt = 0;

  int32_t _dynamixelposition[_dynamixel.size()];
  int32_t _dynamixelvelocity[_dynamixel.size()];
  int32_t _dynamixeleffort[_dynamixel.size()];

  if(strcmp(_interface.c_str(), "position") == 0){
    for(auto const& dxl:_dynamixel){
      id_array[id_cnt] = (uint8_t)dxl.second;
      _dynamixelposition[id_cnt] = _dxl_wb->convertRadian2Value((uint8_t)dxl.second, static_cast<float>(_joints[(uint8_t)dxl.second-1].position_command));

      if(strcmp(dxl.first.c_str(), "gripper") == 0){
        _dynamixelposition[id_cnt] = _dxl_wb->convertRadian2Value((uint8_t)dxl.second, static_cast<float>(_joints[(uint8_t)dxl.second-1].position_command * 150.0));
      }
      id_cnt ++;
    }
    uint8_t sync_write_handler = 0; // 0: position, 1: velocity, 2: effort
    result = _dxl_wb->syncWrite(sync_write_handler, id_array, id_cnt, _dynamixelposition, 1, &log);
  }else if(strcmp(_interface.c_str(), "effort") == 0) {
    for(auto const& dxl:_dynamixel){
      id_array[id_cnt] = (uint8_t)dxl.second;
      _dynamixeleffort[id_cnt] = _dxl_wb->convertCurrent2Value((uint8_t)dxl.second, static_cast<float>(_joints[(uint8_t)dxl.second-1].effort_command / (1.78e-03)));

      if(strcmp(dxl.first.c_str(), "gripper") == 0){
        _dynamixelposition[id_cnt] = _dxl_wb->convertRadian2Value((uint8_t)dxl.second, static_cast<float>(_joints[(uint8_t)dxl.second-1].position_command * 150.0));
      }
      id_cnt ++;
    }
    uint8_t sync_write_handler = 2; // 0: position, 1: velocity, 2: effort
    result = _dxl_wb->syncWrite(sync_write_handler, id_array, id_cnt, _dynamixeleffort, 1, &log);
  }

  if(result == false){
    RCLCPP_ERROR(_logger, "%s", log);
  }
}


void HardwareInterface::registerActuatorInterfaces(){  
  _dxl_wb = new DynamixelWorkbench;

  if(!initWorkbench(_port_name, (uint32_t)_baud_rate)){
    RCLCPP_ERROR(_logger, "Please check USB port name");
    return;
  }

  if(!getDynamixelsInfo(_yaml_file)){
    RCLCPP_ERROR(_logger, "Please check YAML file");
    return;
  }


  if(!loadDynamixels()){
    RCLCPP_ERROR(_logger, "Please check Dynamixel ID or BaudRate");
    return;
  }

  if(!initDynamixels()){
    RCLCPP_ERROR(_logger, "Please check control table (http://emanual.robotis.com/#control-table)");
    return;
  }

  // if(!initControlItems()){
  //   RCLCPP_ERROR(logger, "Please check control items");
  //   return;
  // }

  // if(!initSDKHandlers()){
  //   RCLCPP_ERROR(logger, "Failed to set Dynamixel SDK Handler");
  //   return;
  // }
}


void HardwareInterface::registerControlInterfaces(){
  // resize vector
  uint8_t joint_size = 0;
  for(auto const& dxl:_dynamixel){
    if(joint_size < (uint8_t)dxl.second){
      joint_size = (uint8_t)dxl.second;
    }
  }

  _joints.resize(joint_size);

  for(auto iter = _dynamixel.begin(); iter != _dynamixel.end(); iter++){
    // initialize joint vector
    Joint joint;
    _joints[iter->second - 1] = joint;
    RCLCPP_ERROR(_logger, "joint_name : %s, servo ID: %d", iter->first.c_str(), iter->second);

    // // connect and register the joint state interface
    // hardware_interface::CommandInterface joint_state_handle(iter->first.c_str(),
    //                                                         &_joints[iter->second - 1].position,
    //                                                         &_joints[iter->second - 1].velocity,
    //                                                         &_joints[iter->second - 1].effort);
    // joint_state_interface_.registerHandle(joint_state_handle);

    // // connect and register the joint position, velocity and effort interface
    // hardware_interface::JointHandle position_joint_handle(joint_state_handle, &_joints[iter->second - 1].position_command);
    // position_joint_interface_.registerHandle(position_joint_handle);
    // hardware_interface::JointHandle velocity_joint_handle(joint_state_handle, &_joints[iter->second - 1].velocity_command);
    // velocity_joint_interface_.registerHandle(velocity_joint_handle);
    // hardware_interface::JointHandle effort_joint_handle(joint_state_handle, &_joints[iter->second - 1].effort_command);
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

  result = _dxl_wb->init(port_name.c_str(), baud_rate, &log);
  if(result == false){
    RCLCPP_ERROR(_logger, "%s", log);
  }

  return result;
}


bool HardwareInterface::getDynamixelsInfo(const std::string yaml_file){
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if(dynamixel.IsNull()){
    RCLCPP_ERROR(_logger, "Please check YAML file");
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
        _dynamixel[name] = value;
      }

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      _dynamixel_info.push_back(info);
    }
  }

  return true;
}

bool HardwareInterface::loadDynamixels(void){
  bool result = false;
  const char* log;

  for(auto const& dxl:_dynamixel){
    uint16_t model_number = 0;
    result = _dxl_wb->ping((uint8_t)dxl.second, &model_number, &log);
    if(result == false){
      RCLCPP_ERROR(_logger, "%s", log);
      RCLCPP_ERROR(_logger, "Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }else{
      RCLCPP_INFO(_logger, "Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}


bool HardwareInterface::initDynamixels(void){
  const char* log;

  for(auto const& dxl:_dynamixel){
    _dxl_wb->torqueOff((uint8_t)dxl.second);

    for(auto const& info:_dynamixel_info){
      if(dxl.first == info.first){
        if(info.second.item_name != "ID" && info.second.item_name != "Baud_Rate"){
          bool result = _dxl_wb->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if(result == false){
            RCLCPP_ERROR(_logger, "%s", log);
            RCLCPP_ERROR(_logger, "Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }
  }

  // Torque On after setting up all servo
  for (auto const& dxl:_dynamixel){
    _dxl_wb->torqueOn((uint8_t)dxl.second);
  }

  return true;
}



}  // namespace omx_hardware_interface
