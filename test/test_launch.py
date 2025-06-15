import os
import unittest

from ament_index_python.packages import get_package_share_directory

from controller_manager.hardware_spawner import (
    is_hardware_component_loaded
)
from controller_manager.test_utils import (
    check_controllers_running,
    check_node_running,
)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource

from launch_testing.actions import ReadyToTest
import launch_testing.asserts

import pytest

import rclpy


PACKAGE_NAME = 'sinsei_umiusi_control'
HARDWARE_COMPONENTS = [
    'can',
    'headlights',
    'imu',
    'indicator_led',
    'raspi_camera',
    'usb_camera',
]
CONTROLLERS = [
    'gate_controller',
    'app_controller',
    'thruster_controller'
]


@pytest.mark.rostest
def generate_test_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(PACKAGE_NAME),
                    'launch',
                    'launch.xml'
                )
            ),
            launch_arguments={}.items()
        ),
        ReadyToTest(),
    ])


class TestFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f'{PACKAGE_NAME}_test')

    def tearDown(self):
        self.node.destroy_node()

    def test_hardware_components_loaded(self, proc_output):
        for component in HARDWARE_COMPONENTS:
            with self.subTest(component=component):
                self.assertTrue(
                    is_hardware_component_loaded(
                        self.node,
                        'controller_manager',
                        component,
                        2.0),
                    f"Hardware component '{component}' is not loaded."
                )

    def test_node_running(self, proc_output):
        check_node_running(self.node, 'robot_state_publisher', 3.0)

    def test_controllers_running(self, proc_output):
        check_controllers_running(self.node, CONTROLLERS)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check that all processes exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
