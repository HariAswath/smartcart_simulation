import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    # Launch arguments
    human_mode_arg = DeclareLaunchArgument(
        'human_mode',
        default_value='auto',
        description='Human control mode: auto (autonomous patrol) or manual (keyboard teleop)'
    )

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

    # ROS-Gazebo Bridge for cmd_vel and odometry
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/wheel/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/smartcart_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Human Controller Node
    human_controller_node = Node(
        package='smartcart_human',
        executable='human_controller',
        name='human_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': LaunchConfiguration('human_mode'),
        }]
    )

    # Follow Controller Node
    follow_controller_node = Node(
        package='smartcart_human',
        executable='follow_controller',
        name='follow_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        human_mode_arg,
        set_gz_resource_path,
        start_gazebo,
        robot_state_publisher_node,
        spawn_smartcart_node,
        bridge_node,
        human_controller_node,
        follow_controller_node
    ])
