from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():

    # Arguments
    urdf_path = PathJoinSubstitution([FindPackageShare('my_urdf_description'), 'urdf', 'my_urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([FindPackageShare('my_urdf_description'), 'rviz', 'urdf_config.rviz'])
    ros2_control_yaml = PathJoinSubstitution([FindPackageShare('my_urdf_description'), 'config', 'ros2_control.yaml'])
    world_path = PathJoinSubstitution([FindPackageShare('my_urdf_description'), 'world', 'rough_terrain.world'])

    # robot_description parameter: run xacro and get URDF
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'world': world_path}.items()
    )


    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_robot'],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    control_node = Node(
        package="controller_manager",
    	executable="ros2_control_node",
    	parameters=[robot_description, ros2_control_yaml],
    	remappings=[("~/robot_description", "/robot_description")],
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Static transform publisher (map -> odom)
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )

    # Execute process to load and activate diff_drive_controller


    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=urdf_path, description='URDF xacro file path'),
        DeclareLaunchArgument('rviz_config_path', default_value=rviz_config_path, description='RViz config file'),
        gazebo_launch,
        spawn_entity,
        control_node,
        robot_state_publisher_node,
        rviz_node,
        static_tf_pub,
    ])

