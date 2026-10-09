import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
        default_value='manual',
        description='Human control mode: manual (user keyboard teleop) or auto (autonomous patrol)'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode without GUI (true/false)'
    )

    # Environment variables for Gazebo models resource path
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )
    set_gz_file_path = SetEnvironmentVariable(
        name='GZ_FILE_PATH',
        value=[models_path, ':' + os.environ.get('GZ_FILE_PATH', '')]
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[models_path, ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
    )

    # Start Gazebo Harmonic with the supermarket world (GUI or Headless)
    start_gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    start_gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless'))
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

    # Human 1 Controller Node (Central Aisle - Followed Target)
    human_1_controller_node = Node(
        package='smartcart_human',
        executable='human_controller',
        name='human_controller',
        namespace='human_1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': LaunchConfiguration('human_mode'),
            'entity_name': 'human_1',
            'start_x': 2.0,
            'start_y': 0.0,
            'start_yaw': 0.0,
            'end_x': 10.0,
            'publish_legacy_topic': True,
        }]
    )

    # Human 2 Controller Node (Central Aisle - Bystander)
    human_2_controller_node = Node(
        package='smartcart_human',
        executable='human_controller',
        name='human_controller',
        namespace='human_2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': LaunchConfiguration('human_mode'),
            'entity_name': 'human_2',
            'start_x': 3.8,
            'start_y': 0.4,
            'start_yaw': 0.0,
            'end_x': 9.0,
            'publish_legacy_topic': False,
        }]
    )

    # Human 3 Controller Node (South Aisle C - Shopper)
    human_3_controller_node = Node(
        package='smartcart_human',
        executable='human_controller',
        name='human_controller',
        namespace='human_3',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': LaunchConfiguration('human_mode'),
            'entity_name': 'human_3',
            'start_x': 6.0,
            'start_y': -4.5,
            'start_yaw': 0.0,
            'end_x': 10.0,
            'publish_legacy_topic': False,
        }]
    )

    # Human 4 Controller Node (East Checkout Area - Customer)
    human_4_controller_node = Node(
        package='smartcart_human',
        executable='human_controller',
        name='human_controller',
        namespace='human_4',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': LaunchConfiguration('human_mode'),
            'entity_name': 'human_4',
            'start_x': 13.0,
            'start_y': 3.5,
            'start_yaw': 0.0,
            'end_x': 17.0,
            'publish_legacy_topic': False,
        }]
    )

    # BLE Customer Beacon Simulator Node
    ble_simulator_node = Node(
        package='smartcart_perception',
        executable='ble_simulator',
        name='ble_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'customer_tag_id': 'SMARTCART-CUSTOMER-001',
            'target_human_id': 1
        }]
    )

    # Perception & Target Selection Node (Camera + LiDAR + BLE + Re-ID Fusion)
    target_selector_node = Node(
        package='smartcart_perception',
        executable='target_selector',
        name='target_selector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_human_id': 1,
            'occlusion_timeout': 2.0,
            'reid_match_thresh': 0.65
        }]
    )

    # Follow Controller Node (SmartCart following the Perception /target/pose)
    follow_controller_node = Node(
        package='smartcart_human',
        executable='follow_controller',
        name='follow_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_pose_topic': '/target/pose',
            'target_human_id': 1
        }]
    )

    return LaunchDescription([
        human_mode_arg,
        headless_arg,
        set_gz_resource_path,
        set_gz_file_path,
        set_ign_resource_path,
        start_gazebo_gui,
        start_gazebo_headless,
        robot_state_publisher_node,
        spawn_smartcart_node,
        bridge_node,
        human_1_controller_node,
        human_2_controller_node,
        human_3_controller_node,
        human_4_controller_node,
        ble_simulator_node,
        target_selector_node,
        follow_controller_node
    ])
