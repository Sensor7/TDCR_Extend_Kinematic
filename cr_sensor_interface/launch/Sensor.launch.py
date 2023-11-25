from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, PythonExpression


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("cr_sensor_interface"),
                    "urdf",
                    "Continuum_Robot_with_sensor.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("cr_sensor_interface"),
            "config",
            "cr_sensor.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # node for sensor 
    # TODO: how to specify the broadcaster for undefined sensor?
    fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["fts_broadcaster", "--controller-manager", "/controller_manager"],
    )
    nodes = [
        control_node,
        fts_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
