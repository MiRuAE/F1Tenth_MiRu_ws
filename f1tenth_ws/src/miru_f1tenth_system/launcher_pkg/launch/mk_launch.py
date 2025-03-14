from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        
        # VESC to Odom Node with EKF
        Node(
            package='odom_publisher',
            executable='vesc_to_odom_with_ekf',
            name='vesc_to_odom_with_ekf_node',
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