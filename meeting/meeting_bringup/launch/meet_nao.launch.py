import os
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    youtube_id = ''
    workspace = '~/ws_aivtuber'
    for arg in sys.argv:
        if 'workspace' in arg:
            workspace = arg.split('workspace:=')[1]


    print("============== args ================")
    print("workspace: " + workspace)
    print("====================================")


    # Start simulation
    start_sim_cmd = ExecuteProcess(
        cmd=['bash', os.path.expanduser(workspace) + '/src/ROS2-AI-VTuber-Projects/webots_world/nao/start_sim.bash'],
        shell=True,
        cwd=os.path.expanduser(workspace)
    )

    # Run voicevox_engine container
    voicevox_engine_cmd = ExecuteProcess(
        cmd=['docker', 'run', '--rm', '-p', '127.0.0.1:50021:50021', 'voicevox/voicevox_engine:cpu-ubuntu20.04-latest'],
        shell=True
    )

    print("============== nodes ================")
    speak_ros = Node(package='speak_ros', executable='speak_ros_node', parameters=[{'plugin_name': 'voicevox_plugin::VoiceVoxPlugin'}])
    motion_socket = Node(package='motion_socket', executable='motion_socket_node')
    chatgpt_socket = Node(package='chatgpt_socket', executable='chatgpt_socket_node',
        parameters=[{'ros_installed_setup_bash': os.path.expanduser(workspace) + '/install/setup.bash'},
                    {'text_path': os.path.expanduser(workspace) + '/src/ROS2-AI-VTuber-Projects/chatgpt_socket/config/prompt.txt'}])
    mic2text = Node(package='mic2text', executable='mic2text_node')
    comment_tool = Node(package='comment_tool', executable='comment_tool_node')
    print("============== timer ================")
    delay_node = TimerAction(period=10.0, actions=[motion_socket, chatgpt_socket, mic2text])

    ld.add_action(start_sim_cmd)
    # ld.add_action(voicevox_engine_cmd)

    ld.add_action(delay_node)

    return ld