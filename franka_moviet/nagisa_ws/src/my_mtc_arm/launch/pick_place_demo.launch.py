import os  
  
from ament_index_python.packages import get_package_share_directory  
from launch import LaunchDescription  
from launch.actions import (  
    DeclareLaunchArgument,  
    IncludeLaunchDescription,  
    TimerAction  
)  
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch.substitutions import (  
    Command,  
    FindExecutable,  
    LaunchConfiguration,  
    PathJoinSubstitution  
)  
from launch_ros.actions import Node  
from launch_ros.parameter_descriptions import ParameterValue  
from launch_ros.substitutions import FindPackageShare  
  
import yaml  
  
  
def load_yaml(package_name, file_path):  
    package_path = get_package_share_directory(package_name)  
    absolute_file_path = os.path.join(package_path, file_path)  
  
    try:  
        with open(absolute_file_path, 'r') as file:  
            return yaml.safe_load(file)  
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available  
        return None  
  
  
def generate_launch_description():  
    robot_ip_parameter_name = 'robot_ip'  
    use_fake_hardware_parameter_name = 'use_fake_hardware'  
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'  
    namespace_parameter_name = 'namespace'  
  
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)  
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)  
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)  
    namespace = LaunchConfiguration(namespace_parameter_name)  
  
    # ========== 1. 声明可配置参数 ==========  
    robot_arg = DeclareLaunchArgument(  
        robot_ip_parameter_name,  
        default_value='172.16.0.3',  
        description='Real robot IP (ignored if use_fake_hardware=true)'  
    )  
    use_fake_hardware_arg = DeclareLaunchArgument(  
        use_fake_hardware_parameter_name,  
        default_value='true',  
        description='Use fake hardware for simulation'  
    )  
    fake_sensor_commands_arg = DeclareLaunchArgument(  
        fake_sensor_commands_parameter_name,  
        default_value='true',  
        description='Enable fake sensor commands'  
    )  
    namespace_arg = DeclareLaunchArgument(  
        namespace_parameter_name,  
        default_value='',  
        description='Namespace for the robot'  
    )  
  
    # ========== 2. 生成robot_description ==========  
    franka_xacro_file = os.path.join(  
        get_package_share_directory('franka_description'),  
        'robots', 'fr3', 'fr3.urdf.xacro'  
    )  
  
    robot_description_config = Command(  
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',  
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,  
         ' fake_sensor_commands:=', fake_sensor_commands, ' ros2_control:=true'])  
  
    robot_description = {'robot_description': ParameterValue(  
        robot_description_config, value_type=str)}  
  
    # ========== 3. 生成robot_description_semantic ==========  
    franka_semantic_xacro_file = os.path.join(  
        get_package_share_directory('franka_description'),  
        'robots', 'fr3', 'fr3.srdf.xacro'  
    )  
  
    robot_description_semantic_config = Command(  
        [FindExecutable(name='xacro'), ' ',  
         franka_semantic_xacro_file, ' hand:=true']  
    )  
  
    robot_description_semantic = {'robot_description_semantic': ParameterValue(  
        robot_description_semantic_config, value_type=str)}  
  
    # ========== 4. 加载MoveIt配置 ==========  
    kinematics_yaml = load_yaml(  
        'franka_fr3_moveit_config', 'config/kinematics.yaml'  
    )  
  
    # Planning Functionality  
    ompl_planning_pipeline_config = {  
        'move_group': {  
            'planning_plugin': 'ompl_interface/OMPLPlanner',  
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '  
                                'default_planner_request_adapters/ResolveConstraintFrames '  
                                'default_planner_request_adapters/FixWorkspaceBounds '  
                                'default_planner_request_adapters/FixStartStateBounds '  
                                'default_planner_request_adapters/FixStartStateCollision '  
                                'default_planner_request_adapters/FixStartStatePathConstraints',  
            'start_state_max_bounds_error': 0.1,  
        }  
    }  
    ompl_planning_yaml = load_yaml(  
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'  
    )  
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)  
  
    # Trajectory Execution Functionality  
    moveit_simple_controllers_yaml = load_yaml(  
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml'  
    )  
    moveit_controllers = {  
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,  
        'moveit_controller_manager': 'moveit_simple_controller_manager'  
                                     '/MoveItSimpleControllerManager',  
    }  
  
    trajectory_execution = {  
        'moveit_manage_controllers': True,  
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,  
        'trajectory_execution.allowed_goal_duration_margin': 0.5,  
        'trajectory_execution.allowed_start_tolerance': 0.01,  
    }  
  
    planning_scene_monitor_parameters = {  
        'publish_planning_scene': True,  
        'publish_geometry_updates': True,  
        'publish_state_updates': True,  
        'publish_transforms_updates': True,  
    }  
  
    # ========== 5. 包含完整的 MoveIt 启动文件 ==========  
    moveit_launch = IncludeLaunchDescription(  
        PythonLaunchDescriptionSource([  
            PathJoinSubstitution([  
                FindPackageShare('franka_fr3_moveit_config'),  
                'launch',  
                'moveit.launch.py'  
            ])  
        ]),  
        launch_arguments={  
            'robot_ip': robot_ip,  
            'use_fake_hardware': use_fake_hardware,  
            'fake_sensor_commands': fake_sensor_commands,  
            'namespace': namespace,  
        }.items(),  
    )  
  
    # ========== 6. 您的 MTC 节点（延迟启动）==========  
    pick_place_demo = Node(  
        package="my_mtc_arm",  
        executable="my_mtc_arm",  
        parameters=[  
            robot_description,  
            robot_description_semantic,  
            kinematics_yaml,  
            ompl_planning_pipeline_config,  
            trajectory_execution,  
            moveit_controllers,  
            planning_scene_monitor_parameters,  
            {"use_sim_time": False}  
        ],  
        output="screen"  
    )  

#     mtc_executor = Node(  
#     package='moveit_task_constructor_core',  
#     executable='execute_task_solution',  
#     name='execute_task_solution',  
#     output='screen',  
#     parameters=[  
#         robot_description,  
#         robot_description_semantic,  
#         kinematics_yaml,  
#         ompl_planning_pipeline_config,  
#         trajectory_execution,  
#         moveit_controllers,  
#         planning_scene_monitor_parameters,  
#     ],  
# )  
    # 延迟启动MTC节点，确保MoveIt完全初始化  
    delayed_mtc_node = TimerAction(  
        period=15.0,  # 延迟15秒启动  
        actions=[pick_place_demo]  
    )  
  
    # ========== 7. 返回启动描述 ==========  
    return LaunchDescription([  
        robot_arg,  
        use_fake_hardware_arg,  
        fake_sensor_commands_arg,  
        namespace_arg,  
        moveit_launch,  
        pick_place_demo
        # mtc_executor
    ])