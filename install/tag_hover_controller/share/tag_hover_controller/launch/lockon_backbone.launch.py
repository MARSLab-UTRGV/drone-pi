from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    fcu = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyAMA0:57600'
    )

    mode = DeclareLaunchArgument(
        'mode',
        default_value='SEARCH'
    )

    controller_version = DeclareLaunchArgument(
        'controller_version',
        default_value='v1',
        description='Controller version to use: "v1" (simplified) or "original" (with gating)'
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            {'fcu_url': LaunchConfiguration('fcu_url')}
        ]
    )

    hover_ctrl_v1 = Node(
        package='tag_hover_controller',
        executable='hover_yaw_search_v1',
        name='hover_yaw_search',
        output='screen',
        condition=LaunchConfigurationEquals('controller_version', 'v1'),
        parameters=[
            {'mode': LaunchConfiguration('mode')},
            {'rate_hz': 20.0},
            {'search_yaw': 0.25},               # rad/s - yaw rotation speed in SEARCH mode
            {'lock_k_yaw': 0.1},                # P gain for yaw alignment
            {'lock_k_distance': 0.2},           # P gain for forward/backward (m/s per meter error)
            {'lock_k_lateral': 0.1},            # P gain for left/right
            {'lock_k_vertical': 0.1},           # P gain for up/down
            {'target_distance': 2.0},           # meters - desired standoff from tag
            {'max_forward_vel': 0.5},           # m/s clamp
            {'max_lateral_vel': 0.5},           # m/s clamp
            {'max_yaw_rate': 0.6},              # rad/s clamp
            {'camera_frame': 'camera'},
            {'tag_frame': 'tag36h11:0'},
        ]
    )

    hover_ctrl_orig = Node(
        package='tag_hover_controller',
        executable='hover_yaw_search',
        name='hover_yaw_search',
        output='screen',
        condition=LaunchConfigurationEquals('controller_version', 'original'),
        parameters=[
            {'mode': LaunchConfiguration('mode')},
            {'rate_hz': 20.0},
            {'search_yaw': 0.25},
            {'lock_k_yaw': 0.1},
            {'lock_k_distance': 0.2},
            {'lock_k_lateral': 0.1},
            {'lock_k_vertical': 0.1},
            {'yaw_align_threshold': 0.1},       # radians - only move forward/lateral when yaw aligned
            {'target_distance': 2.0},
            {'max_forward_vel': 0.5},
            {'max_lateral_vel': 0.5},
            {'max_yaw_rate': 0.6},
            {'camera_frame': 'camera'},
            {'body_frame': 'base_link'},        # Original version uses body frame transforms
            {'tag_frame': 'tag36h11:0'},
        ]
    )

    return LaunchDescription([
        fcu,
        mode,
        controller_version,
        mavros,
        hover_ctrl_v1,
        hover_ctrl_orig
    ])

