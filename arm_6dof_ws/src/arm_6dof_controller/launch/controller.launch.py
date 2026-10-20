from launch import LaunchDescription
import os
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory as gpsd

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        name="use_python",
        default_value="true"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[os.path.join(gpsd("arm_6dof_controller"),"config","joy_node.yaml")],
        output="screen"
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[os.path.join(gpsd("arm_6dof_controller"),"config","joy_teleop.yaml")],
        output="screen"
    )

    use_python = LaunchConfiguration("use_python")

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster","--controller-manager","/controller_manager"
        ]
    )

    arm_controller = GroupAction(
        actions=[
            Node(
                package="arm_6dof_controller",
                executable="arm_controller.py",
                parameters=[os.path.join(gpsd("arm_6dof_controller"),"config","arm_controller.yaml")],
                output="screen",
                condition=IfCondition(use_python)
            ),
            Node(
                package="arm_6dof_controller",
                executable="arm_controller",
                parameters=[os.path.join(gpsd("arm_6dof_controller"),"config","arm_controller.yaml")],
                output="screen",
                condition=UnlessCondition(use_python)
            )
        ]
    )

    claw_controller = GroupAction(
        actions=[
            Node(
                package="arm_6dof_controller",
                executable="claw_controller.py",
                output="screen",
                condition=IfCondition(use_python)
            ),
            Node(
                package="arm_6dof_controller",
                executable="claw_controller",
                output="screen",
                condition=UnlessCondition(use_python)
            )
        ]
    )

    return LaunchDescription([
        use_python_arg,
        joy_node,
        joy_teleop,
        joint_state_broadcaster,
        arm_controller,
        claw_controller
    ])