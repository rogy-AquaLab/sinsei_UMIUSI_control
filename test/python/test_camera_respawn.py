import os
import subprocess
import tempfile

import yaml
from ament_index_python import get_package_share_directory
from launch.frontend import Parser

PACKAGE_NAME = 'sinsei_umiusi_control'


def camera_nodes(launch: dict) -> list[dict]:
    found = []

    def walk(entries):
        for entry in entries:
            if 'node' in entry and entry['node'].get('exec') == 'gst_camera_node':
                found.append(entry['node'])
            if 'group' in entry:
                walk(entry['group'])

    walk(launch['launch'])
    return found


def main_launch_path() -> str:
    return os.path.join(get_package_share_directory(PACKAGE_NAME), 'launch', 'main.yaml')


def test_main_launch_parses():
    """launch の型の誤り (例: respawn_delay を文字列で書く) は起動時まで気づけないので、ここで読む"""
    root_entity, parser = Parser.load(main_launch_path())
    parser.parse_description(root_entity)


def test_camera_nodes_respawn():
    """カメラノードはパイプラインのエラーで終了するので、launch で上げ直すこと"""
    with open(main_launch_path()) as f:
        nodes = camera_nodes(yaml.safe_load(f))

    assert {node['name'] for node in nodes} == {'pi_camera', 'usb_camera'}
    for node in nodes:
        assert node.get('respawn') == 'true', node['name']
        assert isinstance(node.get('respawn_delay'), float), node['name']
        assert node['respawn_delay'] > 0.0, node['name']


def test_camera_node_comes_back_after_pipeline_ends():
    """パイプラインが終わる (EOS) と gst_camera_node は終了し、respawn で再びパイプラインを立てる"""
    launch = {
        'launch': [
            {
                'node': {
                    'pkg': PACKAGE_NAME,
                    'exec': 'gst_camera_node',
                    'name': 'respawn_test_camera',
                    'respawn': 'true',
                    'respawn_delay': 0.5,
                    'param': [
                        {'name': 'pipeline', 'value': 'videotestsrc num-buffers=5 ! fakesink'}
                    ],
                }
            }
        ]
    }
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(launch, f)
        launch_file = f.name

    try:
        result = subprocess.run(
            ['timeout', '--signal=INT', '6', 'ros2', 'launch', launch_file],
            capture_output=True,
            text=True,
        )
    finally:
        os.unlink(launch_file)

    output = result.stdout + result.stderr
    assert output.count('Starting GStreamer pipeline') >= 3, output
