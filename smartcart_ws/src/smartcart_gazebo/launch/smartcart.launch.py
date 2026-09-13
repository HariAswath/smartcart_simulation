import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_smartcart_gazebo = get_package_share_directory('smartcart_gazebo')
    pkg_smartcart_description = get_package_share_directory('smartcart_description')

    world_path = os.path.join(pkg_smartcart_gazebo, 'worlds', 'smartcart_world.sdf')
    models_path = os.path.join(pkg_smartcart_gazebo, 'models')
    xacro_file = os.path.join(pkg_smartcart_description, 'urdf', 'smartcart.urdf.xacro')

    # Process Xacro to URDF XML string
    robot_description_doc = xacro.process_file(xacro_file)
    robot_description_content = robot_description_doc.toxml()

    # Environment variable for Gazebo models resource path
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    # Start Gazebo Harmonic with the supermarket world
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Spawn SmartCart in Gazebo
    spawn_smartcart_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_smartcart',
        output='screen',
        arguments=[
            '-name', 'smartcart',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
            '-Y', '0.0'
        ]
    )

    return LaunchDescription([
        set_gz_resource_path,
        start_gazebo,
        robot_state_publisher_node,
        spawn_smartcart_node
    ])
