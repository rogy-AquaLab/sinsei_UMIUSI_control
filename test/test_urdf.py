from collections.abc import Iterator
import itertools
import os
import shutil
import subprocess
import tempfile

import pytest

from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'sinsei_umiusi_control'

# '' means the parameter is not set (default value will be used)
XACRO_ARGUMENTS: dict[str, list[str]] = {
    'thruster_mode': ['', 'can', 'direct'],
}


def generate_xacro_params(
    # {A: [a, b], B: [c, d], ...}
    args: dict[str, list[str]],
) -> Iterator[str]:
    # [[(A, a), (A, b)], [(B, c), (B, d)], ...]
    key_val_pairs = map(
        lambda key: itertools.product([key], args[key]),
        args.keys(),
    )
    # [[(A, a), (B, c), ...], [(A, b), (B, c), ...], ..., [(A, a), (B, d), ...], ...]
    params = itertools.product(*key_val_pairs)
    # if a value (e.g. a, b, ...) equals to '', the pair will be filtered out
    params_filtered = map(
        lambda ps: itertools.filterfalse(
            lambda pair: pair[1] == '',
            ps,
        ),
        params,
    )
    # [args0, args1, ...]
    # where args0: A:="'a'" B:="'c'" ...
    #       args1: A:="'b'" B:="'c'" ...
    params_str = map(
        lambda ps: ' '.join(f'{key}:=\'"{value}"\'' for key, value in ps),
        params_filtered,
    )

    return params_str


@pytest.fixture(params=generate_xacro_params(XACRO_ARGUMENTS))
def xacro_command(request) -> str:
    """Fixture to provide the xacro command."""
    xacro_path = shutil.which('xacro')
    params = request.param
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
