from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory as gpsd
import os

def generate_launch_description():

    gazebo_launch_path = os.path.join(gpsd("drone_description"),"launch","gazebo_localisation.launch.py")
    controller_launch_path = os.path.join(gpsd("drone_controllers"),"launch","controller.launch.py")

    use_plotjuggler_arg = DeclareLaunchArgument(
        name="use_plotjuggler",
        default_value="False"
    )    

    use_plotjuggler = LaunchConfiguration("use_plotjuggler")


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    controller_launch = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch_path)
        )]
    )

    plotjuggler_launch = TimerAction(
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
        gazebo_launch,
        controller_launch,
        plotjuggler_launch
    ])