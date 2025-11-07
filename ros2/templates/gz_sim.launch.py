"""Generic launch file to run Gazebo Sim (Jetty) with ROS 2 integration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Generate the Gazebo Sim launch processes."""
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="worlds/amiga_world.sdf",
        description="Path to the SDF world file to load",
    )

    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="If true, run Gazebo Sim without GUI",
    )

    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", world, "-g"],
        output="screen",
        condition=IfCondition(headless),
    )

    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen",
        condition=UnlessCondition(headless),
    )

    return LaunchDescription([declare_world, declare_headless, gz_headless, gz_gui])
