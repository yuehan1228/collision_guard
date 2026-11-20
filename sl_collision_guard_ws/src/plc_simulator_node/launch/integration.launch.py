from launch import LaunchDescription
from launch.actions import TimerAction
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

    interface_node = Node(
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

    return LaunchDescription(
        [
            simulator_node,
            TimerAction(period=1.0, actions=[interface_node]),
        ]
    )
