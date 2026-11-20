from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    simulator_node = Node(
        package="plc_simulator",
        executable="plc_simulator_node",
        name="plc_simulator_node",
        output="screen",
        parameters=[
            {
                "bind_ip": "0.0.0.0",
                "read_port": 2105,
                "write_port": 2107,
                "update_period_ms": 100,
                "auto_increment_watchdog": True,
            }
        ],
    )

    return LaunchDescription([simulator_node])
