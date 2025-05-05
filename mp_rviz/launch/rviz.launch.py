# Neobotix GmbH
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
import os
from pathlib import Path
import xacro
"""
This code is used for debugging, quick testing, and visualization of the robot with arm and gripper in Rviz. 
"""

def execution_stage(context: LaunchContext, 
                    use_sim_time, 
                    use_joint_state_publisher_gui, 
                    robot_type, 
                    arm_type,
                    imu_enable,
                    d435_enable,
                    uss_enable,
                    scanner_type,
                    gripper_type,
                    docking_adapter,
                    display_mode):
    
    launch_actions = []

    # Resolve launch arguments
    robot_typ = str(robot_type.perform(context))
    use_joint_state_publisher_gui = use_joint_state_publisher_gui.perform(context)
    display_mod = display_mode.perform(context)

    rviz_config = os.path.join(get_package_share_directory("mp_rviz"), 'rviz', 'robot_description_rviz.rviz')
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
    
    xacro_args = {
        'use_gz': 'true', # To force set fixed wheel joints
        'arm_type': arm_type.perform(context),
        'use_imu': imu_enable.perform(context),
        'use_d435': d435_enable.perform(context),
        'use_uss': uss_enable.perform(context),
        'scanner_type': scanner_type.perform(context),
        'gripper_type': gripper_type.perform(context),
        'force_abs_paths': 'true',
        'use_docking_adapter': docking_adapter.perform(context)
    }

    # Get the robot description xacro file based on the robot type
    robot_description_file = xacro.process_file(urdf,mappings=xacro_args).toxml()

    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(PythonExpression([use_joint_state_publisher_gui, ' and ', display_mod])),
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    launch_actions.append(start_joint_state_publisher_gui_cmd)

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(PythonExpression(['not ', use_joint_state_publisher_gui, ' and ', display_mod])),
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    launch_actions.append(start_joint_state_publisher_cmd)

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition=IfCondition(PythonExpression([display_mod])),
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                    'robot_description': robot_description_file}]
    )

    launch_actions.append(start_robot_state_publisher_cmd)

    # Rviz node
    start_rviz_cmd = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + rviz_config]
    )

    launch_actions.append(start_rviz_cmd)

    return launch_actions

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments with default values and descriptions
    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation clock if True (True/False)'
        )

    declare_use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='True',
            description='Use joint state publisher gui if True (True/False)'
        )

    declare_robot_type_arg = DeclareLaunchArgument(
            'robot_type', 
            default_value='mpo_700',
            choices=['', 'mpo_700', 'mpo_500', 'mp_400', 'mp_500'],
            description='Robot Types\n\t'
        )

    declare_arm_type_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            choices=['', 'ur5', 'ur10', 'ur5e', 'ur10e', 'ec66', 'cs66'],
            description='Arm Types - Supported Robots [mpo-700, mpo-500]\n\t'
        )

    declare_use_imu_cmd = DeclareLaunchArgument(
            'use_imu', default_value='false',
            description='Enable IMU sensors if true'
        )

    declare_realsense_cmd = DeclareLaunchArgument(
            'use_d435', default_value='false',
            description='Enable Intel RealSense D435 camera if true\n'
                        '\tSupported Robots [mpo-700, mpo-500, mp-400]'
        )

    declare_use_uss_cmd = DeclareLaunchArgument(
            'use_uss', default_value='false',
            description='Enable Ultrasonic sensors if true\n'
                        '\tSupported Robots [mp-500, mp-400]'
        )
    
    declare_scanner_type_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='sick_s300',
            choices=['sick_s300', 'sick_microscan3'],
            description='Type of laser scanner to use\n\t'
        )
    
    declare_gripper_type_cmd = DeclareLaunchArgument(
            'gripper_type', default_value='',
            choices=['', '2f_140', '2f_85', 'epick'],
            description='Gripper Types - Supported Robots [mpo-700, mpo-500]\n\t'
        )

    declare_use_docking_adapter_cmd = DeclareLaunchArgument(
            'use_docking_adapter', default_value='false',
            description='Enable docking adapter if true\n'
                        '\tSupported Robots [mpo-700]'
        )

    declare_use_display_mode_cmd = DeclareLaunchArgument(
            'display_mode', default_value='false',
            description='Enable robot and joint state publishers if true\n'
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('use_joint_state_publisher_gui'),
            LaunchConfiguration('robot_type'),
            LaunchConfiguration('arm_type'),
            LaunchConfiguration('use_imu'),
            LaunchConfiguration('use_d435'),
            LaunchConfiguration('use_uss'),
            LaunchConfiguration('scanner_type'),
            LaunchConfiguration('gripper_type'),
            LaunchConfiguration('use_docking_adapter'),
            LaunchConfiguration('display_mode')
        ])

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_use_joint_state_publisher_gui_arg,
        declare_robot_type_arg,
        declare_arm_type_cmd,
        declare_use_imu_cmd,
        declare_realsense_cmd,
        declare_use_uss_cmd,
        declare_scanner_type_cmd,
        declare_gripper_type_cmd,
        declare_use_docking_adapter_cmd,
        declare_use_display_mode_cmd,
        opq_function
    ])
