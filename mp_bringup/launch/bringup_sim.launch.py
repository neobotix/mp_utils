# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    gripper_typ = str(gripper_type.perform(context))
    use_sim_time = True

    bringup_dir = get_package_share_directory('mp_bringup')
    default_world_path = os.path.join(get_package_share_directory('neo_gz_worlds'), 'worlds', f'{world_name}.sdf')
    bridge_config_file = os.path.join(bringup_dir, 'configs/gz_bridge', 'gz_bridge_config.yaml')

    include_gripper_ros2_control = "false"
    include_arm_ros2_control = "false"

    """
    Robot specific packages
    -> mpo_700: neo_mpo_700-2
    -> mpo_500: neo_mpo_500-2
    -> mp_400: neo_mp_400-2
    -> mp_500: neo_mp_500-2
    """
    # Handle invalid robot type condition
    try:
        robot_description_pkg = get_package_share_directory(f"neo_{robot_typ}-2")
    except PackageNotFoundError:
        print(f"WARNING: Package for {robot_typ} not found, using mpo_700")
        robot_typ = "mpo_700"
        # Robot description package for the default mpo_700 robot
        robot_description_pkg = get_package_share_directory('neo_mpo_700-2')
        
    # Getting the robot description xacro
    urdf = os.path.join(robot_description_pkg, 'robot_model', f'{robot_typ}.urdf.xacro')
    
    # Simulation Controllers for the arm
    arm_manufacturer = None
    initial_joint_controller_name = "joint_trajectory_controller"
    initial_gripper_controller_name = ""
    if arm_typ:
        include_arm_ros2_control = "true"
        if arm_typ in ['ec66', 'cs66']:
            arm_manufacturer = 'elite'
            initial_joint_controller_name = 'arm_controller'
        elif arm_typ in ['ur5', 'ur10', 'ur5e', 'ur10e']:
            arm_manufacturer = 'ur'

        controllers_yaml = os.path.join(
            bringup_dir,
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
        if gripper_typ:
            include_gripper_ros2_control = "true"
            gripper_category = None
            if gripper_typ == 'epick':
                gripper_category = 'epick'
                initial_gripper_controller_name = 'epick_controller'
            elif gripper_typ in ['2f_140', '2f_85']:
                gripper_category = 'robotiq'
                initial_gripper_controller_name = f'robotiq_{gripper_typ}_gripper_controller'
            include_gripper_ros2_control = "true"

    else:
        simulation_controllers = ""

    # The urdf arguments are different for each robot type
    xacro_args = {
        'use_gz': 'true', # To force set fixed wheel joints
        'arm_type': arm_type.perform(context),
        'use_imu': imu_enable.perform(context),
        'use_d435': d435_enable.perform(context),
        'use_uss': uss_enable.perform(context),
        'scanner_type': scanner_type.perform(context),
        'gripper_type': gripper_type.perform(context),
        'force_abs_paths': 'true',
        'simulation_controllers': simulation_controllers,
        'use_docking_adapter': docking_adapter.perform(context),
        'include_arm_ros2_control': include_arm_ros2_control,
        'include_gripper_ros2_control': include_gripper_ros2_control,
    }

    # Get the robot description xacro file based on the robot type
    robot_description_file = xacro.process_file(urdf,mappings=xacro_args).toxml()
    
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

    teleop = Node(
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

    # Arm specific nodes
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager",  "--controller-manager-timeout", "60"],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller_name, "-c", "/controller_manager"],
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_gripper_controller_name, "-c", "/controller_manager"]
    )

    # Set environment variable
    env_var_value = (
        os.path.join(get_package_share_directory('neo_gz_worlds'), 'models') +
        ':' +
        os.path.dirname(robot_description_pkg)+
        ':' +
        os.path.dirname(get_package_share_directory('mp_components'))
    )

    if arm_typ:
        # Set environment variable for arm description packages
        if arm_typ == 'ec66' or arm_typ == 'cs66':
            env_var_value += ':' + os.path.dirname(get_package_share_directory('elite_description'))
        elif arm_typ == 'ur5' or arm_typ == 'ur10' or arm_typ == 'ur5e' or arm_typ == 'ur10e':
            env_var_value += ':' + os.path.dirname(get_package_share_directory('ur_description'))
        # Set environment variable for gripper description packages
        if gripper_typ == 'epick':
            env_var_value += ':' + os.path.dirname(get_package_share_directory('epick_description'))
        else:
            env_var_value += ':' + os.path.dirname(get_package_share_directory('robotiq_description'))
            
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
        if gripper_typ == '2f_140' or gripper_typ == '2f_85':
            launch_actions.append(robotiq_gripper_controller_spawner)

    return launch_actions

def generate_launch_description():

    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='', description='Top-level namespace'
        )
    
    declare_robot_type_arg = DeclareLaunchArgument(
            'robot_type', 
            default_value='mpo_700',
            choices=['', 'mpo_700', 'mpo_500', 'mp_400', 'mp_500'],
            description='Robot Types\n\t'
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
            description='Arm Types - Supported Robots [mpo-700, mpo-500]\n\t'        
        )
    
    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )
    
    declare_realsense_cmd = DeclareLaunchArgument(
            'd435_enable', default_value='False',
            description='Enable Intel RealSense D435 camera if true\n'
                        '\tSupported Robots [mpo-700, mpo-500, mp-400]'
        )
    
    declare_use_uss_cmd = DeclareLaunchArgument(
            'use_uss', default_value='False',
            description='Enable Ultrasonic sensors if true\n'
                        '\tSupported Robots [mp-500, mp-400]'
        )
    
    declare_scanner_type_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='sick_s300',
            choices=['', 'sick_s300', 'sick_microscan3'],
            description='Type of laser scanner to use\n\t'
        )
    
    declare_gripper_type_cmd = DeclareLaunchArgument(
            'gripper_type', default_value='',
            choices=['', '2f_140', '2f_85', 'epick'],
            description='Gripper Types - Supported Robots [mpo-700, mpo-500]\n\t'
        )

    declare_use_docking_adapter_cmd = DeclareLaunchArgument(
            'use_docking_adapter', default_value='False',
            description='Enable docking adapter if true\n'
                        '\tSupported Robots [mpo-700]'
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
