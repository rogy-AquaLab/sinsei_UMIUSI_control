from launch import LaunchDescription
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    LaunchConfiguration,
    FindExecutable,
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
                ' vesc1_id:=',
                LaunchConfiguration('vesc1_id'),
                ' vesc2_id:=',
                LaunchConfiguration('vesc2_id'),
                ' vesc3_id:=',
                LaunchConfiguration('vesc3_id'),
                ' vesc4_id:=',
                LaunchConfiguration('vesc4_id'),
                ' gpiochip_device:=',
                LaunchConfiguration('gpiochip_device'),
                ' high_beam_line_offset:=',
                LaunchConfiguration('high_beam_line_offset'),
                ' low_beam_line_offset:=',
                LaunchConfiguration('low_beam_line_offset'),
                ' ir_line_offset:=',
                LaunchConfiguration('ir_line_offset'),
                ' indicator_led_line_offset:=',
                LaunchConfiguration('indicator_led_line_offset'),
                ' period_led_tape_per_thrusters:=',
                LaunchConfiguration('period_led_tape_per_thrusters'),
                ' imu_i2c_device:=',
                LaunchConfiguration('imu_i2c_device'),
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
