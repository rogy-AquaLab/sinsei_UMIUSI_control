from launch import LaunchDescription

PACKAGE_NAME = 'sinsei_umiusi_control'

ESC = '\033'
BOLD = f'{ESC}[1m'
GREEN = f'{ESC}[32m'
SKY = f'{ESC}[36m'
PLAIN = f'{ESC}[m'


def header(text: str) -> str:
    return f'{BOLD}{GREEN}{text}'


def key(text: str) -> str:
    return f'{BOLD}{SKY}{text}'


def generate_launch_description():
    print(PACKAGE_NAME)
    print()

    print(
        f'{header("Usage:")} {key(f"ros2 launch {PACKAGE_NAME}")} {PLAIN}{SKY}<LAUNCH_FILE> [ARGS:=VALUES]...'
    )
    print()

    print(header('Launch files:'))
    print(f'  {key("main.yaml")} {PLAIN}{SKY}[ARGS:=VALUES]...   {PLAIN}Run all nodes')
    print(
        f'  {key("gen_local_params.yaml")}         {PLAIN}Generate parameter files into the current directory'
    )
    print(
        f'  {key("launch_args.yaml")}{PLAIN}{SKY} --show-args  {PLAIN}Show launch arguments for {BOLD}main.yaml'
    )
    print(f'  {key("help.yaml")}                     {PLAIN}Print help')
    print()

    print(header('Examples:'))
    print(
        f'  {key("$ ros2 launch sinsei_umiusi_control main.yaml")} {PLAIN}{SKY}thruster_mode:=direct                          {PLAIN}Run all nodes without VESCs'
    )
    print(
        f'  {key("$ ros2 launch sinsei_umiusi_control main.yaml")} {PLAIN}{SKY}vesc1_id:=16 vesc2_id:=32                      {PLAIN}Run all nodes, remapping VESC IDs'
    )
    print(
        f'  {key("$ ros2 launch sinsei_umiusi_control main.yaml")} {PLAIN}{SKY}high_beam_pin:=17 low_beam_pin:=27 ir_pin:=22  {PLAIN}Run all nodes, remapping GPIO pins'
    )
    print(
        f'  {key("$ ros2 launch sinsei_umiusi_control gen_local_params.yaml")}                                    {PLAIN}Generate local parameter files'
    )
    print(
        f'  {key("$ ros2 launch sinsei_umiusi_control launch_args.yaml")} {PLAIN}{SKY}--show-args                             {PLAIN}Show launch arguments for {BOLD}main.yaml'
    )
    print(
        f'  {key("$ ros2 launch sinsei_umiusi_control help.yaml")}                                                {PLAIN}Print help'
    )

    print(PLAIN)

    return LaunchDescription()
