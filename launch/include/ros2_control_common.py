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
                ' thruster_mode:=',
                LaunchConfiguration('thruster_mode'),
                ' vesc1_id:=',
                LaunchConfiguration('vesc1_id'),
                ' vesc2_id:=',
                LaunchConfiguration('vesc2_id'),
                ' vesc3_id:=',
                LaunchConfiguration('vesc3_id'),
                ' vesc4_id:=',
                LaunchConfiguration('vesc4_id'),
                ' high_beam_pin:=',
                LaunchConfiguration('high_beam_pin'),
                ' low_beam_pin:=',
                LaunchConfiguration('low_beam_pin'),
                ' ir_pin:=',
                LaunchConfiguration('ir_pin'),
                ' indicator_led_pin:=',
                LaunchConfiguration('indicator_led_pin'),
                ' servo_direct1_min_pulse_width:=',
                LaunchConfiguration('servo_direct1_min_pulse_width'),
                ' servo_direct1_max_pulse_width:=',
                LaunchConfiguration('servo_direct1_max_pulse_width'),
                ' servo_direct2_min_pulse_width:=',
                LaunchConfiguration('servo_direct2_min_pulse_width'),
                ' servo_direct2_max_pulse_width:=',
                LaunchConfiguration('servo_direct2_max_pulse_width'),
                ' servo_direct3_min_pulse_width:=',
                LaunchConfiguration('servo_direct3_min_pulse_width'),
                ' servo_direct3_max_pulse_width:=',
                LaunchConfiguration('servo_direct3_max_pulse_width'),
                ' servo_direct4_min_pulse_width:=',
                LaunchConfiguration('servo_direct4_min_pulse_width'),
                ' servo_direct4_max_pulse_width:=',
                LaunchConfiguration('servo_direct4_max_pulse_width'),
                ' servo_direct1_pin:=',
                LaunchConfiguration('servo_direct1_pin'),
                ' servo_direct2_pin:=',
                LaunchConfiguration('servo_direct2_pin'),
                ' servo_direct3_pin:=',
                LaunchConfiguration('servo_direct3_pin'),
                ' servo_direct4_pin:=',
                LaunchConfiguration('servo_direct4_pin'),
                ' esc_direct1_pin:=',
                LaunchConfiguration('esc_direct1_pin'),
                ' esc_direct2_pin:=',
                LaunchConfiguration('esc_direct2_pin'),
                ' esc_direct3_pin:=',
                LaunchConfiguration('esc_direct3_pin'),
                ' esc_direct4_pin:=',
                LaunchConfiguration('esc_direct4_pin'),
                ' period_led_tape_per_thrusters:=',
                LaunchConfiguration('period_led_tape_per_thrusters'),
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
                parameters=[
                    LaunchConfiguration('controllers_param_file'),
                    {'thruster_mode': LaunchConfiguration('thruster_mode')},
                ],
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='both',
                parameters=[{'robot_description': robot_description}],
            ),
        ]
    )
