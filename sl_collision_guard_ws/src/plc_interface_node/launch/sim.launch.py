from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    plc_interface_node = Node(
        package="plc_interface",
        executable="plc_interface_node",
        name="plc_interface_node",
        output="screen",
        parameters=[
            {
                "plc_ip": "127.0.0.1",
                "plc_read_port": 2105,
                "plc_write_port": 2107,
                "read_register_count": 20,
            }
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([plc_interface_node])
