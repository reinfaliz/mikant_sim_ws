from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from scripts import GazeboRosPaths

def generate_launch_description():
    model_pkg_dir = get_package_share_directory('mikant_description')
    model_xacro_file = os.path.join(model_pkg_dir, 'urdf', 'mikant.xacro')
    robot_description_config = xacro.process_file(model_xacro_file)
    robot_urdf = robot_description_config.toxml()

    world_pkg_dir = get_package_share_directory("mikant_sim")
    world_file = os.path.join(world_pkg_dir, "worlds", "rimsra.world")

    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path
    }

    gazebo_with_env = ExecuteProcess(
        cmd=[
            "gazebo",
            "-u",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world_file
       ],
        output="screen",
        additional_env=env
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    # gazebo_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gzserver.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'pause': 'true'
    #     }.items()
    # )

    # gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gzclient.launch.py'
    #         ])
    #     ])
    # )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mikant',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_with_env,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # gazebo_server,
        # gazebo_client,
        urdf_spawn_node
    ])
