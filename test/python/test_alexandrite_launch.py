import os

import launch_pytest
import pytest
import rclpy
from ament_index_python import get_package_share_directory
from controller_manager.hardware_spawner import is_hardware_component_loaded
from controller_manager.test_utils import check_controllers_running, check_node_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from rclpy.node import Node

PACKAGE_NAME = 'sinsei_umiusi_control'


@launch_pytest.fixture
def generate_launch_description():
    launch_file = FrontendLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(PACKAGE_NAME),
            'launch',
            'alexandrite.yaml',
        )
    )
    return LaunchDescription([IncludeLaunchDescription(launch_file)])


@pytest.fixture
def helper_node():
    rclpy.init()
    node = Node('alexandrite_test_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.launch(fixture=generate_launch_description)
def test_alexandrite_nodes_and_hardware(helper_node):
    check_node_running(helper_node, 'robot_state_publisher', 1.0)
    assert is_hardware_component_loaded(helper_node, 'controller_manager', 'can', 10.0)


@pytest.mark.launch(fixture=generate_launch_description)
def test_alexandrite_controller_running(helper_node):
    check_controllers_running(
        helper_node,
        {'alexandrite_controller'},
        '',
        'active',
        120,
    )
