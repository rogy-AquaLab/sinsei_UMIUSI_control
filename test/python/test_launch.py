import os

import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python import get_package_share_directory

from controller_manager.hardware_spawner import list_hardware_components
from controller_manager.controller_manager_services import list_controllers
from controller_manager.test_utils import check_node_running

import pytest
import time

import launch_pytest

from helper import (
    generate_arguments_list,
    arguments_list_to_dict,
    display_arguments_list,
)

PACKAGE_NAME = 'sinsei_umiusi_control'

LAUNCH_ARGUMENTS: dict[str, set[str]] = {
    'thruster_driver_type': {'can', 'direct'},  # URDFに渡される引数代表
    'enable_cameras': {'false'},
}


def hardware_components(largs: dict[str, str]) -> set[str]:
    base = {
        'can',
        'headlights',
        'imu',
        'indicator_led',
    }
    if largs['thruster_driver_type'] == 'can':
        return base
    elif largs['thruster_driver_type'] == 'direct':
        return (
            base
            | {f'thruster_direct{i}/servo' for i in range(1, 5)}
            | {f'thruster_direct{i}/esc' for i in range(1, 5)}
        )


def controllers(largs: dict[str, str]) -> set[str]:
    """Return the set of controllers based on launch arguments."""
    _ = largs

    base = {
        'gate_controller',
        'attitude_controller',
        'thruster_controller_lf',
        'thruster_controller_lb',
        'thruster_controller_rb',
        'thruster_controller_rf',
    }
    return base


def wait_for_controllers_active(
    helper_node: Node,
    controller_manager_name: str,
    expected: set[str],
    timeout: float,
):
    """Wait until all expected controllers are reported as active."""
    start = time.time()
    while time.time() - start < timeout:
        controllers = list_controllers(helper_node, controller_manager_name, 5.0).controller
        states = {controller.name: controller.state for controller in controllers}
        active = {
            name for name in expected
            if states.get(name) == 'active'
        }
        if active == expected:
            return
        time.sleep(0.1)

    missing = sorted(expected - active)
    raise AssertionError(
        f'Controller(s) not active: {missing}. Last observed states: {states}'
    )


def wait_for_hardware_loaded(
    helper_node: Node,
    controller_manager_name: str,
    expected: set[str],
    timeout: float,
):
    """Wait until all expected hardware components are listed."""
    start = time.time()
    while time.time() - start < timeout:
        loaded = {
            component.name
            for component in list_hardware_components(
                helper_node,
                controller_manager_name,
                10.0,
            ).component
        }
        if expected <= loaded:
            return
        time.sleep(0.1)

    missing = sorted(expected - loaded)
    raise AssertionError(
        f'Hardware component(s) {missing} are not loaded. Last observed: {sorted(loaded)}'
    )


@launch_pytest.fixture
def generate_launch_description(launch_arguments: dict[str, str]):
    """Fixture to generate launch description."""
    args = launch_arguments
    print(f'Launch arguments: {args}')
    launch_file = FrontendLaunchDescriptionSource(
        os.path.join(get_package_share_directory(PACKAGE_NAME), 'launch', 'main.yaml')
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
    check_node_running(helper_node, 'robot_state_publisher', 1.0)


@pytest.mark.launch(fixture=generate_launch_description)
def test_hardware_loaded(helper_node, launch_arguments):
    """Test if the hardware is loaded."""
    components = hardware_components(launch_arguments)
    ns = launch_arguments.get('namespace', '')
    controller_manager_name = f'{ns}/controller_manager' if ns else 'controller_manager'
    wait_for_hardware_loaded(helper_node, controller_manager_name, components, 120)


@pytest.mark.launch(fixture=generate_launch_description)
def test_controllers_running(helper_node, launch_arguments):
    """Test if the controllers are running."""
    cnames = controllers(launch_arguments)
    ns = launch_arguments.get('namespace', '')
    controller_manager_name = f'{ns}/controller_manager' if ns else 'controller_manager'
    wait_for_controllers_active(helper_node, controller_manager_name, cnames, 120)


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
    node = Node('test_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()
