"""Launch Gazebo Sim (Jetty) with the Farm-NG Amiga and Kinova Gen3 manipulator."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the Gazebo Sim launch description."""
    world_path = Path(__file__).resolve().parent.parent.parent / "worlds" / "amiga_world.sdf"
    model_path = Path(__file__).resolve().parent.parent.parent / "models"
    config_path = Path(__file__).resolve().parent.parent.parent / "config" / "ros2_control.yaml"

    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Gazebo Sim without the GUI.",
    )

    env_model = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(model_path),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            ThisLaunchFileDir(),
            "/../templates/gz_sim.launch.py",
        ]),
        launch_arguments={
            "world": str(world_path),
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/amiga_kinova/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/model/amiga_kinova/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/world/amiga_test_world/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
        ],
        output="screen",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "amiga_base_controller",
            "kinova_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}],
        remappings=[("/robot_description", "/amiga_kinova_description")],
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    ros2_control_params = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[str(config_path)],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_headless,
            env_model,
            ros2_control_params,
            gazebo_launch,
            ros_gz_bridge,
            controller_spawner,
            robot_state_publisher,
        ]
    )
