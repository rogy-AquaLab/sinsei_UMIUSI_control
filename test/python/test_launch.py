import os
import time

import launch_pytest
import pytest
import rclpy
from ament_index_python import get_package_share_directory
from controller_manager.controller_manager_services import (
    ServiceNotFoundError,
    list_controllers,
)
from controller_manager.hardware_spawner import list_hardware_components
from controller_manager.test_utils import check_node_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from rclpy.node import Node

from helper import (
    HelperNode,
    arguments_list_to_dict,
    display_arguments_list,
    generate_arguments_list,
)

PACKAGE_NAME = 'sinsei_umiusi_control'

LAUNCH_ARGUMENTS: dict[str, set[str]] = {
    'thruster_driver_type': {'can', 'direct'},
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
    if largs['thruster_driver_type'] == 'direct':
        return (
            base
            | {f'thruster_direct{i}/servo' for i in range(1, 5)}
            | {f'thruster_direct{i}/esc' for i in range(1, 5)}
        )
    raise AssertionError(f'Unknown thruster_driver_type: {largs["thruster_driver_type"]}')


def controllers(_: dict[str, str]) -> set[str]:
    """Return the set of controllers based on launch arguments."""
    return {
        'gate_controller',
        'attitude_controller',
        'thruster_controller_lf',
        'thruster_controller_lb',
        'thruster_controller_rb',
        'thruster_controller_rf',
    }


def wait_for_hardware_loaded(
    node: Node,
    controller_manager_name: str,
    expected: set[str],
    timeout: float,
):
    """Wait until all expected hardware components are listed."""
    start = time.time()
    loaded: set[str] = set()
    while time.time() - start < timeout:
        try:
            loaded = {
                component.name
                for component in list_hardware_components(
                    node,
                    controller_manager_name,
                    service_timeout=1.0,
                    call_timeout=1.0,
                ).component
            }
        except (RuntimeError, ServiceNotFoundError):
            time.sleep(0.1)
            continue
        if expected <= loaded:
            return
        time.sleep(0.1)

    missing = sorted(expected - loaded)
    raise AssertionError(
        f'Hardware component(s) {missing} are not loaded. Last observed: {sorted(loaded)}'
    )


def wait_for_controllers_active(
    node: Node,
    controller_manager_name: str,
    expected: set[str],
    timeout: float,
):
    """Wait until all expected controllers are reported as active."""
    start = time.time()
    states: dict[str, str] = {}
    while time.time() - start < timeout:
        try:
            current = list_controllers(
                node,
                controller_manager_name,
                service_timeout=1.0,
                call_timeout=1.0,
            ).controller
        except (RuntimeError, ServiceNotFoundError):
            time.sleep(0.1)
            continue
        states = {controller.name: controller.state for controller in current}
        if all(states.get(name) == 'active' for name in expected):
            return
        time.sleep(0.1)

    missing = sorted(name for name in expected if states.get(name) != 'active')
    raise AssertionError(
        f'Controller(s) not active: {missing}. Last observed states: {states}'
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
def test_hardware_loaded(service_node, launch_arguments):
    """Test if the hardware is loaded."""
    components = hardware_components(launch_arguments)
    ns = launch_arguments.get('namespace', '')
    controller_manager_name = f'{ns}/controller_manager' if ns else 'controller_manager'
    wait_for_hardware_loaded(service_node, controller_manager_name, components, 120.0)


@pytest.mark.launch(fixture=generate_launch_description)
def test_controllers_running(service_node, launch_arguments):
    """Test if the controllers are running."""
    cnames = controllers(launch_arguments)
    ns = launch_arguments.get('namespace', '')
    controller_manager_name = f'{ns}/controller_manager' if ns else 'controller_manager'
    wait_for_controllers_active(service_node, controller_manager_name, cnames, 120.0)


@pytest.fixture(
    params=generate_arguments_list(LAUNCH_ARGUMENTS), ids=display_arguments_list
)
def launch_arguments(request):
    """Fixture to provide launch arguments."""
    return arguments_list_to_dict(request.param)


@pytest.fixture
def helper_node():
    """Fixture to provide a helper node for graph-based checks."""
    rclpy.init()
    node = HelperNode()
    node.start()
    yield node
    rclpy.shutdown()


@pytest.fixture
def service_node():
    """Fixture to provide a non-spinning node for service-based checks."""
    if not rclpy.ok():
        rclpy.init()
    node = Node('service_test_node')
    yield node
    node.destroy_node()
