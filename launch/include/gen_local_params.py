import os
import shutil
import fs

from launch import LaunchDescription
from ament_index_python import get_package_share_directory

PACKAGE_NAME = 'sinsei_umiusi_control'


def generate_launch_description():
    BEGIN_GREEN = '\033[32m'
    END = '\033[m'

    print(f'{BEGIN_GREEN}Generating parameter files ...{END}')

    launch_args_path = fs.path.join(
        get_package_share_directory(PACKAGE_NAME), 'params', 'launch_args.yaml'
    )
    controllers_path = fs.path.join(
        get_package_share_directory(PACKAGE_NAME), 'params', 'controllers.yaml'
    )

    dst_dir = 'local_params'
    dst_launch_args = fs.path.join('.', dst_dir, 'args.yaml')
    dst_controllers = fs.path.join('.', dst_dir, 'controllers.yaml')
    os.makedirs(dst_dir, exist_ok=True)
    shutil.copy(launch_args_path, dst_launch_args)
    shutil.copy(controllers_path, dst_controllers)

    replaced = ''
    with open(dst_launch_args, 'r') as f:
        body = f.read()
        replaced = body.replace(
            r'$(find-pkg-share $(var pkg_name))/params/controllers.yaml',
            dst_controllers,
        )
    with open(dst_launch_args, 'w') as f:
        f.write(replaced)

    print(f'{BEGIN_GREEN}Parameter files generated in {dst_dir}!{END}')

    return LaunchDescription()
