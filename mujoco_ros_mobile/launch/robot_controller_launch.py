from launch import LaunchDescription
import launch
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    robot_name = launch.substitutions.LaunchConfiguration('robot_name')
    config_file = os.path.join(
        get_package_share_directory('mujoco_ros_mobile'),
        'config',
        'teleop_twist_joy.yaml'
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='false'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='0'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[launch.substitutions.TextSubstitution(text=config_file)]),
        launch.actions.DeclareLaunchArgument('robot_name', default_value='summit_xls'),
        
        Node(
        package='joy', 
        executable='joy_node', 
        name='joy_node',
        parameters=[{
            'device_id': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,}]
        ),

        Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[config_file, {'publish_stamped_twist': publish_stamped_twist}],
        remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
        ),

        Node(
        package='mujoco_ros_mobile',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[{'robot_name': robot_name}]
        ),

    ])
