from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_arm_with_gripper", package_name="my_arm_config")
        .to_moveit_configs()
    )

    launch_description = generate_demo_launch(moveit_config)

    motion_server = Node(
        package="my_arm_config",
        executable="moveit_motion_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"planning_group": "arm", "base_frame": "world"},
        ],
    )

    qt_client = Node(
        package="my_arm_config",
        executable="qt_arm_client",
        output="screen",
        emulate_tty=True,
    )

    launch_description.add_action(motion_server)
    launch_description.add_action(qt_client)
    return launch_description

