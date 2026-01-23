# file: launch/arena.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('mam_eurobot_2026')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'arena_world.sdf'])

    return LaunchDescription([
        # Make sure Gazebo can find your worlds/models
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share),

        DeclareLaunchArgument('world', default_value=world_file),

        # Start Gazebo (use one of these; keep the other commented)
        # ExecuteProcess(cmd=['gz', 'sim', '-r', LaunchConfiguration('world')], output='screen'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')], output='screen'),

        # Single bridge with explicit directions
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            arguments=[
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/simple_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            ]
            #arguments=[
                # Gazebo -> ROS
                #'/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock[gz2ros]',
                #'/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image[gz2ros]',
                #'/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                #'/lidar/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                #'/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan[gz2ros]',
                #'/lidar/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked[gz2ros]',
                #'/model/simple_robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry[gz2ros]',
                #'/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry[gz2ros]',
                #'/model/simple_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry[gz2ros]',



                # ROS -> Gazebo
                #'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist[ros2gz]',
                
            #],
            

        ),
        



        # RViz (use sim time so /clock drives it)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            # arguments=['-d', '<path-to-your-rviz-config.rviz>']  # optional
        ),
    ])
