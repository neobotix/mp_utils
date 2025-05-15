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
from launch_ros.parameter_descriptions import ParameterValue
import os
from pathlib import Path
import xacro
"""
This code is used for debugging, quick testing, and visualization of MP series robots in Rviz. 
"""

def execution_stage(context: LaunchContext, 
                    use_sim_time, 
                    use_joint_state_publisher_gui, 
                    display_mode,
                    rviz_config,
                    robot_description_content):
    
    launch_actions = []

    # Resolve launch arguments
    rviz_config_file = str(rviz_config.perform(context))
    use_joint_state_publisher_gui = use_joint_state_publisher_gui.perform(context)
    display_mod = display_mode.perform(context)

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
                    'robot_description': ParameterValue(
                        robot_description_content,
                        value_type=str
                    )
                }],
            )

    launch_actions.append(start_robot_state_publisher_cmd)

    # Rviz node
    start_rviz_cmd = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + rviz_config_file]
    )

    launch_actions.append(start_rviz_cmd)

    return launch_actions

def generate_launch_description():

    # Declare launch arguments with default values and descriptions
    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation clock if True (True/False)'
        )

    declare_use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='True',
            description='Use joint state publisher gui if True (True/False)'
        )

    declare_use_display_mode_cmd = DeclareLaunchArgument(
            'display_mode', default_value='True',
            description='Disable robot and joint state publishers if true (True/False)'
        )

    declare_rviz_cfg_arg = DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
            get_package_share_directory('mp_rviz'),
            'rviz', 'robot_description_rviz.rviz'),
            description='Full path to an RViz config file'
        )

    declare_robot_description_content_arg = DeclareLaunchArgument(
            name='robot_description_content',
            default_value='',
            description='Robot description XML content',
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('use_joint_state_publisher_gui'),
            LaunchConfiguration('display_mode'),
            LaunchConfiguration('rviz_config'),
            LaunchConfiguration('robot_description_content')
        ])

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_use_joint_state_publisher_gui_arg,
        declare_use_display_mode_cmd,
        declare_rviz_cfg_arg,
        declare_robot_description_content_arg,
        opq_function
    ])
