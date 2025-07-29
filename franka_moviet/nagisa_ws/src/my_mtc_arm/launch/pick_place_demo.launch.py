from launch import LaunchDescription    
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription    
from launch.launch_description_sources import PythonLaunchDescriptionSource    
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution    
from launch_ros.actions import Node    
from launch_ros.substitutions import FindPackageShare    
from launch.actions import TimerAction  
def generate_launch_description():    
    # ========== 1. 声明可配置参数 ==========    
    robot_ip_arg = DeclareLaunchArgument(    
        'robot_ip',    
        default_value='172.16.0.3',    
        description='Real robot IP (ignored if use_fake_hardware=true)'    
    )    
    use_fake_hardware_arg = DeclareLaunchArgument(    
        'use_fake_hardware',    
        default_value='true',    
        description='Use fake hardware for simulation'    
    )    
    fake_sensor_commands_arg = DeclareLaunchArgument(    
        'fake_sensor_commands',    
        default_value='true',    
        description='Enable fake sensor commands'    
    )    
    namespace_arg = DeclareLaunchArgument(    
        'namespace',    
        default_value='',    
        description='Namespace for the robot'    
    )    
    
    # ========== 2. 包含完整的 MoveIt 启动文件 ==========    
    moveit_launch = IncludeLaunchDescription(    
        PythonLaunchDescriptionSource([    
            PathJoinSubstitution([    
                FindPackageShare('franka_fr3_moveit_config'),    
                'launch',    
                'moveit.launch.py'  # 使用完整的 MoveIt 启动文件  
            ])    
        ]),    
        launch_arguments={    
            'robot_ip': LaunchConfiguration('robot_ip'),    
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),    
            'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands'),    
            'namespace': LaunchConfiguration('namespace'),    
        }.items(),    
    )    
    
    # ========== 3. 您的 MTC 节点 ==========    
    pick_place_demo = Node(    
        package="my_mtc_arm",    
        executable="my_mtc_arm",    
        namespace=LaunchConfiguration('namespace'),    
        output="screen",    
        parameters=[    
            {"use_sim_time": False}    
        ]    
    )    
    
    # ========== 4. 返回启动描述 ==========    
    return LaunchDescription([    
        robot_ip_arg,    
        use_fake_hardware_arg,    
        fake_sensor_commands_arg,    
        namespace_arg,    
        moveit_launch,    
        pick_place_demo    
    ])