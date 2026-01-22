# launch/arena.launch.py  (Ignition/Fortress version)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('mam_eurobot_2026')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'arena_world.sdf'])

    return LaunchDescription([
        # Resource paths (set both to be safe across Ignition/Gz)
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', pkg_share),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share),

        DeclareLaunchArgument('world', default_value=world_file),

        # 👉 Start Ignition WITH GUI (remove "-r")
        ExecuteProcess(
            cmd=['ign', 'gazebo', LaunchConfiguration('world')],
            output='screen'
        ),

        # 👉 Use ros_ign_bridge for Ignition, and use ignition.msgs.* types
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='ign_bridge',
            output='screen',
            arguments=[
                # Gazebo(Ignition) -> ROS 2
                '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
                '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/lidar/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                # ROS 2 -> Gazebo (example cmd_vel if you need it)
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            ],
            parameters=[{'use_sim_time': True}],
        ),

        # Optional RViz (driven by sim time)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])

