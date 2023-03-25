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
        if 'youtube_id' in arg:
            youtube_id = arg.split('youtube_id:=')[1]
        if 'workspace' in arg:
            workspace = arg.split('workspace:=')[1]


    if youtube_id == '':
        print('Please specify youtube_id')
        exit()
    else:
        if 'watch?v=' in youtube_id:
            youtube_id = youtube_id.split('watch?v=')[1]
        if '&' in youtube_id:
            youtube_id = youtube_id.split('&')[0]
        video_id = youtube_id

    print("============== args ================")
    print("workspace: " + workspace)
    print("video_id: " + video_id)
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
                    {'text_path': os.path.expanduser(workspace) + '/src/ROS2-AI-VTuber-Projects/chatgpt_socket/config/prompt.txt'},
                    {'speak_enable': True}])
    youtube_comment = Node(package='youtube_comment', executable='youtube_comment_node', parameters=[{'video_id': video_id}])
    print("============== timer ================")
    delay_node = TimerAction(period=10.0, actions=[speak_ros, motion_socket, chatgpt_socket, youtube_comment])

    ld.add_action(start_sim_cmd)
    ld.add_action(voicevox_engine_cmd)

    ld.add_action(delay_node)

    return ld