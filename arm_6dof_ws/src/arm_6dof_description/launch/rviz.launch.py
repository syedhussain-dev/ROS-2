from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_arm_description_path = get_package_share_directory("arm_6dof_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(robot_arm_description_path,"urdf","robot_arm.urdf.xacro")
    )

    model = LaunchConfiguration("model")

    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d",os.path.join(robot_arm_description_path,"rviz","display2.rviz")]

    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2_node
    ])