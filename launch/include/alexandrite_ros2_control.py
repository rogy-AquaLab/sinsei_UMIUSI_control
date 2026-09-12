from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                LaunchConfiguration('robot_description_file'),
                ' thruster1_vesc_id:=',
                LaunchConfiguration('thruster1_vesc_id'),
                ' thruster2_vesc_id:=',
                LaunchConfiguration('thruster2_vesc_id'),
                ' thruster3_vesc_id:=',
                LaunchConfiguration('thruster3_vesc_id'),
                ' crawler_left_vesc_id:=',
                LaunchConfiguration('crawler_left_vesc_id'),
                ' crawler_right_vesc_id:=',
                LaunchConfiguration('crawler_right_vesc_id'),
                ' period_led_tape_per_actuators:=',
                LaunchConfiguration('period_led_tape_per_actuators'),
            ],
        ),
        value_type=str,
    )
    return LaunchDescription(
        [
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='both',
                parameters=[LaunchConfiguration('controllers_param_file')],
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='both',
                parameters=[{'robot_description': robot_description}],
            ),
        ]
    )
