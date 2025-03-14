from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare the parameter file argument
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(
            FindPackageShare('f1tenth_stack').find('f1tenth_stack'),
            'config',
            'vesc.yaml'
        ),
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