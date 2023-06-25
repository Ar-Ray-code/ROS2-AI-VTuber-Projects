from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import (OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown)
from launch_ros.actions import Node
from launch.events import Shutdown
import os

def generate_launch_description():
    ld = LaunchDescription()

    # launch demo_nodes_cpp
    launch_share_dir = os.path.join(get_package_share_directory('demo_nodes_cpp'), 'launch')
    talker_listener_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_share_dir, 'topics/talker_listener.launch.py')
    )

    # xeyes
    xeyes_cmd = ExecuteProcess(
        cmd=['xeyes']
    )
    # node
    turtlesim_node = Node(package='turtlesim', executable='turtlesim_node')


    # OnProcessIO for talker
    turtlesim_node_io = RegisterEventHandler(
        OnProcessIO(
            target_action=turtlesim_node,
            on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                ),
            on_stderr=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                ),
        )
    )

    xeyes_delay = TimerAction(period=3.0, actions=[xeyes_cmd])
    turtlesim_after_xeyes = RegisterEventHandler(
        OnProcessExit(
            target_action=xeyes_cmd,
            on_exit=[
                LogInfo(msg="==== xeyes exited ===="),
                turtlesim_node
            ]
        )
    )


    shutdown_action = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(msg="==== Shutdown ====")]
        )
    )


    start_turtlesim_action = RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim_node,
            on_start=[
                LogInfo(msg="==== turtlesim_node started ====")
            ]
        )
    )
    end_turtlesim_action = RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim_node,
            on_exit=[
                LogInfo(msg="==== turtlesim_node exited ===="),
                EmitEvent(event=Shutdown(
                    reason='turtlesim_node exited'
                ))
            ]
        )
    )

    ld.add_action(talker_listener_launch)
    ld.add_action(xeyes_delay)

    # ==== turtlesim_node ====
    ld.add_action(turtlesim_after_xeyes)
    ld.add_action(start_turtlesim_action)
    ld.add_action(turtlesim_node_io)
    ld.add_action(end_turtlesim_action)
    # ==== end turtlesim_node ====

    ld.add_action(shutdown_action)

    return ld
