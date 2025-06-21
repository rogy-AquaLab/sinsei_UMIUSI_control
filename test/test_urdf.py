import os
import shutil
import subprocess
import tempfile

import pytest

from ament_index_python.packages import get_package_share_directory

from helper import (
    generate_arguments_list,
    arguments_list_to_xacro_args,
    display_arguments_list,
)

PACKAGE_NAME = 'sinsei_umiusi_control'

# '' means the parameter is not set (default value will be used)
XACRO_ARGUMENTS: dict[str, set[str]] = {
    'thruster_mode': {'', 'can', 'direct'},
}


@pytest.fixture(
    params=generate_arguments_list(XACRO_ARGUMENTS),
    ids=display_arguments_list,
)
def xacro_command(request) -> str:
    """Fixture to provide the xacro command."""
    xacro_path = shutil.which('xacro')
    params = arguments_list_to_xacro_args(request.param)
    xacro_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'urdf',
        'sinsei_umiusi_control.urdf.xacro',
    )
    cmd = f'{xacro_path} {xacro_file} {params}'
    return cmd


def test_urdf(xacro_command: str):
    with tempfile.TemporaryFile(suffix='.urdf') as temp_urdf_file:
        xacro_process = subprocess.run(
            f'{xacro_command} > {temp_urdf_file.name}',
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True,
        )
        assert xacro_process.returncode == 0, 'xacro command failed'

        check_urdf_process = subprocess.run(
            f'{shutil.which("check_urdf")} {temp_urdf_file.name}',
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True,
        )
        assert check_urdf_process.returncode == 0, 'check URDF command failed'
