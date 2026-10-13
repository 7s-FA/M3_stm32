from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    text_ns = ParameterValue(ns, value_type=str)
    transport = ParameterValue(LaunchConfiguration('transport_namespace'), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='burger3'),
        DeclareLaunchArgument('transport_namespace', default_value='/stm32'),
        DeclareLaunchArgument('state_publisher', default_value='true'),
        DeclareLaunchArgument('enable_stamped_cmd_vel', default_value='true'),
        GroupAction([
            PushRosNamespace(ns),
            Node(package='burger_stm32_bridge', executable='stm32_adapter', output='screen',
                 parameters=[{'frame_prefix': text_ns, 'transport_namespace': transport,
                              'enable_stamped_cmd_vel': ParameterValue(
                                  LaunchConfiguration('enable_stamped_cmd_vel'), value_type=bool)}]),
            Node(package='burger_stm32_bridge', executable='stm32_odometry', output='screen',
                 parameters=[{'namespace': text_ns, 'wheels.radius': 0.033,
                              'wheels.separation': 0.160, 'odometry.use_imu': True,
                              'odometry.publish_tf': True, 'odometry.frame_id': 'odom',
                              'odometry.child_frame_id': 'base_footprint'}]),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'), 'launch',
                'turtlebot3_state_publisher.launch.py'])),
                launch_arguments={'namespace': ns, 'use_sim_time': 'false'}.items(),
                condition=IfCondition(LaunchConfiguration('state_publisher'))),
        ])
    ])
