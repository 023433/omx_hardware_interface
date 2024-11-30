import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  ld = LaunchDescription()
  share_dir = get_package_share_directory('omx_hardware_interface')
  hardware_config = os.path.join(share_dir, 'config', 'hardware.yaml')


  ld.add_action(
    Node(
      package='omx_hardware_interface',
      executable='temp',
      parameters=[{
        'yaml_file': hardware_config,
      }]
    )
  )

  return ld