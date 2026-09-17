from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from os import pathsep
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():

    drone_description_package_location = get_package_share_directory("drone_description")
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"


    model_args = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(drone_description_package_location,"urdf","quadcopter.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    world_name_args = DeclareLaunchArgument(
        name="world_name",
        default_value="empty"
    )

    world_path = PathJoinSubstitution([
        drone_description_package_location,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'" , " + '.world'"])
    ])

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")," is_ignition:=", is_ignition]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description" : robot_description}]
    )

    model_path = str(Path(drone_description_package_location).parent.resolve())
    #model_path += pathsep + os.path.join(drone_description_package_location,"models")

    rotor_plugin_path = os.path.join(
        get_package_share_directory("drone_description"), "lib"
    )

    gazebo_plugin_path = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        rotor_plugin_path
    )

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", model_path
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"),"/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args":PythonExpression(["'",world_path, " -v 4 -r'"])
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic","robot_description","-name","quadcopter"],
        output="screen"
    )

    gazebo_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat"
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/quadcopter/simple_velocity_controller/commands@actuator_msgs/msg/Actuators]gz.msgs.Actuators"
        ],
        remappings=[
            ("/imu","/imu/out"),
            ("/gps","/gps/out")
        ]
    )

    return LaunchDescription([
        model_args,
        world_name_args,
        robot_state_publisher,
        gazebo_plugin_path,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gazebo_ros2_bridge
    ])