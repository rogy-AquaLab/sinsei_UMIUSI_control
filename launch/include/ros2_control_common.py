from launch import LaunchDescription
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    LaunchConfiguration,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = 'sinsei_umiusi_control'


def generate_launch_description():
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution(
                    [FindPackageShare(PACKAGE_NAME), 'urdf', 'main.urdf.xacro']
                ),
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
                    PathJoinSubstitution(
                        [FindPackageShare(PACKAGE_NAME), 'params', 'controllers.yaml']
                    ),
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
