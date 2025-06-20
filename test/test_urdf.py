from collections.abc import Iterator
import itertools
import os
import shutil
import subprocess
import tempfile

import pytest

from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'sinsei_umiusi_control'

XACRO_ARGUMENTS: dict[str, list[str]] = {'thruster_mode': ['can', 'direct']}


def generate_xacro_params(
    args: dict[str, list[str]],
) -> Iterator[Iterator[tuple[str, str]]]:
    # [[('thruster_mode', 'can'), ('thruster_mode', 'direct')], ...]
    key_val_pairs = map(
        lambda key: itertools.product([key], args[key]),
        args.keys(),
    )
    # [[('thruster_mode', 'can'), ...], [('thruster_mode', 'can), ...], ..., [('thruster_mode', 'direct'), ...], ...]
    return itertools.product(*key_val_pairs)


@pytest.fixture(
    params=generate_xacro_params(XACRO_ARGUMENTS),
    ids=lambda p: ', '.join(f'{k}={v}' for k, v in p),
)
def xacro_command(request) -> str:
    """Fixture to provide the xacro command."""
    args = ' '.join(f'{key}:=\'"{value}"\'' for key, value in request.param)
    xacro_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'urdf',
        'sinsei_umiusi_control.urdf.xacro',
    )
    cmd = f'{shutil.which("xacro")} {xacro_file} {args}'
    print(cmd)
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
