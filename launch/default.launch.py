import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_dir = FindPackageShare('gnss_to_odom')
    yaml_path = PathJoinSubstitution([package_dir, 'config', 'default.yaml'])

    container = ComposableNodeContainer(
        name='gnss_to_odom_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gnss_to_odom',
                plugin='gnss_to_odom::GnssToOdom',
                name='gnss_to_odom',
                parameters=[yaml_path]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager',
                parameters=[{'autostart': True,
                             'node_names': ['/gnss_to_odom']}])
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
