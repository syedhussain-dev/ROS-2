from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        name="use_python", default_value="true"
    )
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[os.path.join(get_package_share_directory("drone_controllers"), "config", "joy_config.yaml")],
        output="screen"
    )
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("drone_controllers"), "config", "joy_teleop.yaml")],
        output="screen"
    )  

    use_python = LaunchConfiguration("use_python")


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster","--controller-manager","/controller_manager"
        ]
    )

    mellinger_spawner = GroupAction(
        actions=[
            Node(
                package="drone_controllers",
                executable="mellinger_controller.py",
                parameters=[os.path.join(get_package_share_directory("drone_controllers"),"config","mellinger_controller.yaml")],
                output="screen",
                condition=IfCondition(use_python)
            ),
            Node(
                package="drone_controllers",
                executable="mellinger_controller",
                parameters=[os.path.join(get_package_share_directory("drone_controllers"),"config","mellinger_controller.yaml")],
                output="screen",
                condition=UnlessCondition(use_python)
            )
        ]
    )

    kalman_filter = GroupAction(
        actions=[
            Node(
                package="drone_localisation",
                executable="kalman_filter.py",
                parameters=[os.path.join(get_package_share_directory("drone_localisation"),"config","kalman_filter.yaml")],
                output="screen",
                condition=IfCondition(use_python)
            ),
            Node(
                package="drone_localisation",
                executable="kalman_filter",
                parameters=[os.path.join(get_package_share_directory("drone_localisation"),"config","kalman_filter.yaml")],
                output="screen",
                condition=UnlessCondition(use_python)
            )
        ]
    )
    


    return LaunchDescription([
        use_python_arg,
        joy_node,
        joy_teleop,
        joint_state_broadcaster_spawner,
        mellinger_spawner,
        kalman_filter
    ])