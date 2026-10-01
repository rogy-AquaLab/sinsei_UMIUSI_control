"""Pin the shipped `params/controllers.yaml` against settings that silently kill thrust.

Background (B-20): `servo_max_angular_velocity: 0.0` is a legal value for
`ServoAngleEstimator` and means "the estimate is never resolved"
(servo_angle_estimator.cpp:43-52). The mixer refuses to emit any command while a
single unit is unresolved (mixer.hpp:66-69, 91-96), so shipping 0.0 makes the whole
vehicle produce no thrust at all -- and nothing in the build or the logs says so.
The file shipped 0.0 for every thruster from the estimator's introduction (52ef389)
until this test existed.
"""

import os

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'sinsei_umiusi_control'

THRUSTER_CONTROLLERS = (
    'thruster_controller_lf',
    'thruster_controller_lb',
    'thruster_controller_rf',
    'thruster_controller_rb',
)

# Servo travel is [-pi/2, +pi/2], and the estimator must sweep the full pi before it
# resolves. Below this the arm-to-first-thrust dead time exceeds ~3 s, which is longer
# than the shortest commands the mission sends.
MIN_SERVO_ANGULAR_VELOCITY = 1.0  # rad/s


@pytest.fixture(scope='module')
def controllers_params() -> dict:
    path = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'params', 'controllers.yaml'
    )
    with open(path) as f:
        return yaml.safe_load(f)['/**']


@pytest.mark.parametrize('controller', THRUSTER_CONTROLLERS)
def test_servo_max_angular_velocity_allows_thrust(
    controllers_params: dict, controller: str
) -> None:
    """0 leaves the servo angle permanently unresolved, so no thrust is ever emitted."""
    params = controllers_params[controller]['ros__parameters']
    value = params['servo_max_angular_velocity']

    assert value >= MIN_SERVO_ANGULAR_VELOCITY, (
        f'{controller}: servo_max_angular_velocity={value} rad/s leaves the servo '
        'angle estimate unresolved (or too slow to converge), and the mixer emits no '
        'command at all while any unit is unresolved -- the vehicle produces no thrust. '
        'See the comment at the top of params/controllers.yaml (B-20).'
    )
