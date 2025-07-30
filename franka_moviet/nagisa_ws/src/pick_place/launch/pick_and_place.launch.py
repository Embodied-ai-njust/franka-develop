#!/usr/bin/env python3  
  
import os  
import sys  
from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction  
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  
from launch_ros.actions import Node  
from launch_ros.substitutions import FindPackageShare  
from ament_index_python.packages import get_package_share_directory  
  
# Add utils path for launch_utils  
package_share = get_package_share_directory('franka_bringup')  
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')  
sys.path.append(os.path.abspath(utils_path))  
  
from launch_utils import load_yaml  
  
def generate_robot_nodes(context):  
    """Generate all required nodes for pick and place task"""  
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)  
    configs = load_yaml(robot_config_file)  
    nodes = []  
      
    for item_name, config in configs.items():  
        namespace = config['namespace']  
        robot_ip = config['robot_ip']  
        load_gripper = config['load_gripper']  
        use_fake_hardware = config['use_fake_hardware']  
        fake_sensor_commands = config['fake_sensor_commands']  
          
        # 只使用 moveit.launch.py，它已经包含了所有必要的组件  
        nodes.append(  
            IncludeLaunchDescription(  
                PythonLaunchDescriptionSource(  
                    PathJoinSubstitution([  
                        FindPackageShare('franka_fr3_moveit_config'),  
                        'launch',  
                        'moveit.launch.py',  
                    ])  
                ),  
                launch_arguments={  
                    'robot_ip': str(robot_ip),  
                    'namespace': str(namespace),  
                    'use_fake_hardware': str(use_fake_hardware),  
                    'fake_sensor_commands': str(fake_sensor_commands),  
                    'load_gripper': str(load_gripper),
                }.items(),  
            )  
        )  
          
        # 添加自定义的夹取任务节点  
        nodes.append(  
            Node(  
                package='pick_place',  
                executable='task.py',  
                namespace=namespace,  
                output='screen',  
                parameters=[{  
                    'robot_ip': robot_ip,  
                    'use_fake_hardware': use_fake_hardware,  
                }]  
            )  
        )  
      
    return nodes  
  
def generate_launch_description():  
    return LaunchDescription([  
        DeclareLaunchArgument(  
            'robot_config_file',  
            default_value=PathJoinSubstitution([  
                FindPackageShare('pick_place'), 'config', 'franka.config.yaml'  
            ]),  
            description='Path to the robot configuration file'  
        ),  
        OpaqueFunction(function=generate_robot_nodes),  
    ])