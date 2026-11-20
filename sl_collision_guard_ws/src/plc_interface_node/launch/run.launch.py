from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'plc_interface'
    
    pkg_share = FindPackageShare(pkg_name)
    params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'config.yaml'
    ])
    
    yolo_lidar_fusion_node = Node(
        package=pkg_name,
        executable='plc_interface_node',
        name='plc_interface_node',
        parameters=[params_file],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    return LaunchDescription([
        yolo_lidar_fusion_node
    ])