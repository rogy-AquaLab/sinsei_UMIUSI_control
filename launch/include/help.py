from launch import LaunchDescription

PACKAGE_NAME = 'sinsei_umiusi_control'


def generate_launch_description():
    BEGIN_RED = '\033[31m'
    BEGIN_GREEN = '\033[32m'
    BEGIN_BLUE = '\033[34m'
    END = '\033[m'

    print(f'{BEGIN_RED}Working{END}')
    print(f'{BEGIN_GREEN}In{END}')
    print(f'{BEGIN_BLUE}Progress{END}')

    return LaunchDescription()
