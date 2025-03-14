from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the parameter file
    f1tenth_stack_dir = get_package_share_directory('f1tenth_stack')
    default_param_file = os.path.join(f1tenth_stack_dir, 'config', 'vesc.yaml')

    # Declare the parameter file argument
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Path to the VESC parameter file'
    )

    return LaunchDescription([
        # Add the parameter file argument
        param_file_arg,

        # Node Launcher for sector detection
        Node(
            package='launcher_pkg',
            executable='mk_node_launcher',
            name='mk_node_launcher',
            output='screen'
        ),
        
        # Lane Following Node for camera-based driving
        Node(
            package='camera_basic_pkg',
            executable='lanefollowing',
            name='lane_following_node',
            output='screen'
        ),
        
        # Reactive Follow Gap Node for LiDAR-based driving
        Node(
            package='gap_follow',
            executable='reactive_node',
            name='reactive_node',
            output='screen'
        ),
        
        # VESC to Odom Node with EKF (as a component)
        ComposableNodeContainer(
            name='vesc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='odom_publisher',
                    plugin='vesc_ackermann::VescToOdomWithEKF',
                    name='vesc_to_odom_with_ekf_node',
                    parameters=[LaunchConfiguration('param_file')]
                )
            ],
            output='screen'
        ),
        
        # Odom Navigation Node for position-based driving
        Node(
            package='odom_navigation',
            executable='odom_navigation_node',
            name='odom_navigation_node',
            output='screen'
        )
    ]) 