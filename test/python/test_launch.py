import os

import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python import get_package_share_directory

from controller_manager.hardware_spawner import is_hardware_component_loaded
from controller_manager.test_utils import check_controllers_running, check_node_running

import pytest

import launch_pytest

from helper import (
    generate_arguments_list,
    arguments_list_to_dict,
    display_arguments_list,
    HelperNode,
)

PACKAGE_NAME = 'sinsei_umiusi_control'

LAUNCH_ARGUMENTS: dict[str, set[str]] = {
    'thruster_mode': {'can', 'direct'},
}


def hardware_components(largs: dict[str, str]) -> set[str]:
    base = {
        'can',
        'headlights',
        'imu',
        'indicator_led',
    }
    if largs['thruster_mode'] == 'can':
        return base
    elif largs['thruster_mode'] == 'direct':
        return (
            base
            | {f'thruster_direct{i}/servo_direct' for i in range(1, 5)}
            | {f'thruster_direct{i}/esc_direct' for i in range(1, 5)}
        )


def controllers(largs: dict[str, str]) -> set[str]:
    """Return the set of controllers based on launch arguments."""
    _ = largs

    base = {
        'gate_controller',
        'app_controller',
        'thruster_controller1',
        'thruster_controller2',
        'thruster_controller3',
        'thruster_controller4',
    }
    return base


@launch_pytest.fixture
def generate_launch_description(launch_arguments: dict[str, str]):
    """Fixture to generate launch description."""
    args = launch_arguments
    print(f'Launch arguments: {args}')
    launch_file = FrontendLaunchDescriptionSource(
        os.path.join(get_package_share_directory(PACKAGE_NAME), 'launch', 'launch.xml')
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                launch_file,
                launch_arguments=args.items(),
            ),
        ]
    )


@pytest.mark.launch(fixture=generate_launch_description)
def test_robot_state_publisher_running(helper_node, launch_arguments):
    """Test if the robot state publisher is running."""
    _ = launch_arguments
    check_node_running(helper_node, 'robot_state_publisher', 2.0)


@pytest.mark.launch(fixture=generate_launch_description)
def test_hardware_loaded(helper_node, launch_arguments):
    """Test if the hardware is loaded."""
    components = hardware_components(launch_arguments)
    for component in components:
        assert is_hardware_component_loaded(
            helper_node,
            'controller_manager',
            component,
            3.0,
        ), f'Hardware component {component} is not loaded.'


@pytest.mark.launch(fixture=generate_launch_description)
def test_controllers_running(helper_node, launch_arguments):
    """Test if the controllers are running."""
    cnames = controllers(launch_arguments)
    check_controllers_running(helper_node, cnames)


@pytest.fixture(
    params=generate_arguments_list(LAUNCH_ARGUMENTS), ids=display_arguments_list
)
def launch_arguments(request):
    """Fixture to provide launch arguments."""
    return arguments_list_to_dict(request.param)


@pytest.fixture
def helper_node():
    """Fixture to provide a helper node."""
    rclpy.init()
    node = HelperNode()
    node.start()
    yield node
    rclpy.shutdown()
