import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'sinsei_umiusi_control'


def test_urdf():
    xacro_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'urdf',
        'sinsei_umiusi_control.urdf.xacro',
    )

    with tempfile.TemporaryFile(suffix='.urdf') as temp_urdf_file:
        xacro_process = subprocess.run(
            f'{shutil.which("xacro")} {xacro_file} > {temp_urdf_file.name}',
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True,
        )
        assert xacro_process.returncode == 0, ' --- Xacro command failed ---'

        check_urdf_process = subprocess.run(
            f'{shutil.which("check_urdf")} {temp_urdf_file.name}',
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True,
        )
        assert check_urdf_process.returncode == 0, ' --- Check URDF command failed ---'
