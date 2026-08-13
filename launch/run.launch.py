import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    # Follow /clock from `ros2 bag play --clock` instead of the system clock.
    # Set to false when running against a live sensor.
    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/bag clock published on /clock.')

    # value_type=bool so the string from the CLI arrives as a real boolean;
    # a bare LaunchConfiguration would be declared as a string parameter.
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    # Level for the mapOptimization logger only — 'debug' surfaces the GPS
    # factor diagnostics without the rclcpp internals a process-wide
    # --log-level would pull in.
    log_level_declare = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logger level for lio_sam_mapOptimization (info|debug|warn).')

    map_opt_log_level = ['--ros-args', '--log-level',
                         ['lio_sam_mapOptimization:=',
                          LaunchConfiguration('log_level')]]

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        use_sim_time_declare,
        log_level_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro', ' ', xacro_path]), value_type=str),
                'use_sim_time': use_sim_time,
            }]
        ),
        # ---- Navsat: GPS -> odometry/gps, consumed by mapOptimization ----
        # ekf_gps publishes odometry/filtered (remapped to odometry/navsat),
        # which navsat consumes; navsat publishes odometry/gps, which feeds both
        # back into ekf_gps (odom0) and into LIO-SAM's addGPSFactor().
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_gps',
            respawn=True,
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/navsat')],
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat',
            respawn=True,
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            # ROS 2 subscribes to 'imu', not ROS 1's 'imu/data'.
            remappings=[('imu', 'imu_correct'),
                        ('odometry/filtered', 'odometry/navsat')],
            output='screen'
        ),
        # No name= here: this process constructs TWO nodes (TransformFusion and
        # IMUPreintegration), and Node(name=...) becomes a process-wide
        # `-r __node:=` remap that would rename both to the same thing and
        # collide. They keep the names their ParamServer constructors assign.
        # params.yaml uses the /** wildcard, so parameters still reach both.
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            arguments=map_opt_log_level,
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
