# turtlesim/launch/multisim.launch.py

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
             package='function', executable='server', output='screen'),
             
        launch_ros.actions.Node(
             package='function', executable='listener_client', output='screen',
             arguments=['src/matrix/matrix.json']),

        launch_ros.actions.Node(
             package='function', executable='talker', output='screen'),
    ])