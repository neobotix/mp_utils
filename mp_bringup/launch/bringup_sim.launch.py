# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from param_file_utils import generate_final_yaml
import os
import xacro

def execution_stage(context: LaunchContext,
                    robot_namespace,
                    robot_type,
                    world,
                    arm_type,
                    imu_enable,
                    d435_enable,
                    uss_enable,
                    scanner_type,
                    gripper_type,
                    docking_adapter):    
    
    launch_actions = []

    # Resolve launch arguments
    robot_typ = str(robot_type.perform(context))
    world_name = str(world.perform(context))
    arm_typ = arm_type.perform(context)
    imu_enabl = imu_enable.perform(context)
    d435_enabl = d435_enable.perform(context)
    uss_enable = uss_enable.perform(context)
    scanner_typ = str(scanner_type.perform(context))
    gripper_typ = gripper_type.perform(context)
    use_docking_adapter = docking_adapter.perform(context)
    use_sim_time = True

    bridge_dir = get_package_share_directory('mp_bringup')
    default_world_path = os.path.join(get_package_share_directory('neo_gz_worlds'), 'worlds', f'{world_name}.sdf')
    bridge_config_file = os.path.join(bridge_dir, 'configs/gz_bridge', 'gz_bridge_config.yaml')

    # Robot description package for the default mpo_700 robot
    robot_description_pkg = get_package_share_directory('neo_mpo_700-2')

    # Dictionary that maps robot types to their respective package names
    robot_type_packages = {
        'mpo_700': 'neo_mpo_700-2',
        'mpo_500': 'neo_mpo_500-2',
        'mp_400': 'neo_mp_400-2',
        'mp_500': 'neo_mp_500-2'
    }

    # Get the robot-specific package if it exists
    robot_description_pkg = get_package_share_directory(robot_type_packages[robot_typ])
    
    # Remove arm_type if robot does not support it
    if (arm_typ != ''):
        if (robot_typ != "mpo_700" and robot_typ != "mpo_500"):
            print("Robot does not support arm, setting arm_type to empty")
            arm_typ = ''

    # Getting the robot description xacro for the robot type
    urdf = os.path.join(robot_description_pkg, 'robot_model', f'{robot_typ}.urdf.xacro')
    
    # Simulation Controllers for the arm
    arm_manufacturer = None
    initial_joint_controller_name = "joint_trajectory_controller"
    if arm_typ:
        if arm_typ in ['ec66', 'cs66']:
            arm_manufacturer = 'elite'
            initial_joint_controller_name = 'arm_controller'
        elif arm_typ in ['ur5', 'ur10', 'ur5e', 'ur10e']:
            arm_manufacturer = 'ur'

    if arm_manufacturer is not None:
        controllers_yaml = os.path.join(
            bridge_dir,
            'configs', 
            arm_manufacturer, 
            'controllers.yaml'
        )
        # Generates a final YAML parameter file from the controllers template (with substitutions applied),
        # and returns file_path, shutdown_handler
        simulation_controllers, shutdown_handler = generate_final_yaml(
            context,
            controllers_yaml,
            file_name='simulation_controllers.yaml',
            cleanup_enabled=False
        )
        launch_actions.extend(shutdown_handler)

    else:
        simulation_controllers = ""

    # The urdf arguments are different for each robot type
    xacro_args_for_robot_type = {

        'mpo_700': {
            'use_gz': 'true',
            'arm_type': arm_typ,
            'use_imu': imu_enabl,
            'use_d435': d435_enabl,
            'scanner_type': scanner_typ,
            'gripper_type': gripper_typ,
            'force_abs_paths': 'true',
            'simulation_controllers': simulation_controllers,
            'use_docking_adapter': use_docking_adapter
        },

        'mpo_500': {
            'use_gz': 'true',
            'arm_type': arm_typ,
            'use_imu': imu_enabl,
            'use_d435': d435_enabl,
            'scanner_type': scanner_typ,
            'gripper_type': gripper_typ,
            'force_abs_paths': 'true',
            'simulation_controllers': simulation_controllers,
        },

        'mp_500': {
            'use_gz': 'true',
            'use_imu': imu_enabl,
            'use_uss': uss_enable,
            'scanner_type': scanner_typ,
        },

        'mp_400': {
            'use_gz': 'true',
            'use_imu': imu_enabl,
            'use_d435': d435_enabl,
            'use_uss': uss_enable,
            'scanner_type': scanner_typ,
        }
    }

    # Get the robot description xacro file based on the robot type
    robot_description_file = xacro.process_file(urdf,mappings=xacro_args_for_robot_type[robot_typ]).toxml()
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        output='screen',
        arguments=[
            '-topic', "robot_description",
            '-name', robot_typ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        )
        , launch_arguments={'gz_args': ['-r ', default_world_path]}.items()
      )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                    'robot_description': robot_description_file}]
    )

    teleop =  Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        name='teleop',
        parameters=[{'stamped': True}]  # Set stamped parameter to true for TwistStamped /cmd_vel
    )
  
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_file}])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller_name, "-c", "/controller_manager"],
    )

    # Set environment variable
    env_var_value = (
        os.path.join(get_package_share_directory('neo_gz_worlds'), 'models') +
        ':' +
        os.path.dirname(robot_description_pkg)+
        ':' +
        os.path.dirname(get_package_share_directory('mp_components'))
    )
    if arm_typ == 'ec66' or arm_typ == 'cs66':
        env_var_value += ':' + os.path.dirname(get_package_share_directory('elite_description'))
    if arm_typ == 'ur5' or arm_typ == 'ur10' or arm_typ == 'ur5e' or arm_typ == 'ur10e':
        env_var_value += ':' + os.path.dirname(get_package_share_directory('ur_description'))
    set_env_vars_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', env_var_value)

    launch_actions.append(set_env_vars_resources)
    launch_actions.append(start_robot_state_publisher_cmd)
    launch_actions.append(gz_sim)
    launch_actions.append(gz_bridge)
    launch_actions.append(teleop)
    launch_actions.append(spawn_robot)

    if arm_typ != '':
        launch_actions.append(joint_state_broadcaster_spawner)
        launch_actions.append(initial_joint_controller_spawner_started)

    return launch_actions

def generate_launch_description():

    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='', description='Top-level namespace'
        )
    
    declare_robot_type_arg = DeclareLaunchArgument(
            'robot_type', 
            default_value='mpo_700',
            choices=['', 'mpo_700', 'mpo_500', 'mp_400', 'mp_500'],
            description='Robot Types'
        )
    
    declare_world_name_arg = DeclareLaunchArgument(
            'world',
            default_value='neo_workshop',
            choices=['', 'neo_workshop'],
            description='Simulation world to load'
        )

    declare_arm_type_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            choices=['', 'ur5', 'ur10', 'ur5e', 'ur10e', 'ec66', 'cs66'],
            description='Supported Arm Types'        
        )
    
    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )
    
    declare_realsense_cmd = DeclareLaunchArgument(
            'd435_enable', default_value='False',
            description='Enable Realsense - Options: True/False'
        )
    
    declare_use_uss_cmd = DeclareLaunchArgument(
            'use_uss', default_value='false',
            description='Enable Ultrasonic sensors if true'
        )
    
    declare_scanner_type_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='sick_s300',
            choices=['', 'sick_s300', 'sick_microscan3'],
            description='Type of laser scanner to use'
        )
    
    declare_gripper_type_cmd = DeclareLaunchArgument(
            'gripper_type', default_value='',
            choices=['', '2f_140', '2f_85', 'epick'],
            description='Type of gripper to use'
        )

    declare_use_docking_adapter_cmd = DeclareLaunchArgument(
            'use_docking_adapter', default_value='false',
            description='Enable docking adapter - Options: True/False'
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('robot_namespace'),
            LaunchConfiguration('robot_type'),
            LaunchConfiguration('world'),
            LaunchConfiguration('arm_type'),
            LaunchConfiguration('imu_enable'),
            LaunchConfiguration('d435_enable'),
            LaunchConfiguration('use_uss'),
            LaunchConfiguration('scanner_type'),
            LaunchConfiguration('gripper_type'),
            LaunchConfiguration('use_docking_adapter')
        ])
    
    return LaunchDescription([
        declare_namespace_cmd,
        declare_robot_type_arg,
        declare_world_name_arg,
        declare_arm_type_cmd,
        declare_imu_cmd,
        declare_realsense_cmd,
        declare_use_uss_cmd,
        declare_scanner_type_cmd,
        declare_gripper_type_cmd,
        declare_use_docking_adapter_cmd,
        opq_function
    ])


