from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory  # Make sure this import is present

def generate_launch_description():
    # Move variable assignment OUTSIDE the return
    package_path = get_package_share_directory('camera_ros2')

    return LaunchDescription([
        # Aruco tracker node
        Node(
            package='camera_ros2',
            executable='fb',
            name='fb',
            output='screen',  
        ),
        Node(
            package='camera_ros2',
            executable='cam',
            name='cam',
            output='screen',   
        )
    ])

