from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory as gpsd

def generate_launch_description():

    gazebo_path = os.path.join(gpsd("arm_6dof_description"),"launch","gazebo.launch.py")
    controller_path = os.path.join(gpsd("arm_6dof_controller"),"launch","controller.launch.py")

    use_plotjuggler_arg = DeclareLaunchArgument(
        name="use_plotjuggler",
        default_value="false"
    )
    use_plotjuggler = LaunchConfiguration("use_plotjuggler")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path)
    )

    controller = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controller_path)
            )
        ]
    )

    plotjuggler = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="plotjuggler",
                executable="plotjuggler",
                condition=IfCondition(use_plotjuggler)
            )
        ]
    )

    return LaunchDescription([
        use_plotjuggler_arg,
        gazebo,
        controller,
        plotjuggler
    ])