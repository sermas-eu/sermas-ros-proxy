from launch import LaunchDescription
import launch_ros.actions
import sys


def generate_launch_description():

    proxy = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=['/ros_ws/src/ros_proxy/ros_proxy/sermas_ros_proxy.py'],
        parameters=[]
    )
    
    return (
        LaunchDescription([
            proxy
        ])
    )
