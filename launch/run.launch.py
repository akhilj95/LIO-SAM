import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# platform -> (params file, urdf, navsat IMU topic)
#
# The third entry is the topic navsat_transform_node's 'imu' input is remapped
# to. ROS 2 subscribes to 'imu' where ROS 1 used 'imu/data', and FRUC's
# module_navsat.launch remaps it per robot - so it has to travel with the
# platform rather than be hardcoded.
#
# 'default' reproduces the stock LIO-SAM setup and remains the default, so
# existing usage is unchanged. The rest are ported from
# Forestry-Robotics-UC/fruc_lio_sam.
PLATFORMS = {
    'default':  ('params.yaml',         'robot.urdf.xacro',          'imu_correct'),
    'rslidar':  ('params_rslidar.yaml', 'fruc_rslidar.urdf.xacro',   'imu_data_resampled'),
    'ouster':   ('params_ouster.yaml',  'fruc_ouster.urdf.xacro',    '/imu/data/corrected'),
    'curtmini': ('params_ouster.yaml',  'fruc_curtmini.urdf.xacro',  '/imu/data/corrected'),
    'hesai':    ('params_hesai.yaml',   'fruc_hesai.urdf.xacro',     'imu/data/corrected'),
}


def launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory('lio_sam')

    platform = LaunchConfiguration('platform').perform(context)
    if platform not in PLATFORMS:
        raise RuntimeError(
            "unknown platform '{}'. Choose one of: {}".format(
                platform, ', '.join(sorted(PLATFORMS))))
    default_params, default_urdf, navsat_imu_topic = PLATFORMS[platform]

    # Empty means "use the platform default"; an explicit value wins.
    params_file = (LaunchConfiguration('params_file').perform(context)
                   or os.path.join(share_dir, 'config', default_params))
    xacro_path = (LaunchConfiguration('urdf_file').perform(context)
                  or os.path.join(share_dir, 'config', default_urdf))
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    for path, what in ((params_file, 'params file'), (xacro_path, 'urdf')):
        if not os.path.exists(path):
            raise RuntimeError('{} not found: {}'.format(what, path))

    print('lio_sam platform : {}'.format(platform))
    print('  params         : {}'.format(params_file))
    print('  urdf           : {}'.format(xacro_path))
    print('  navsat imu     : {}'.format(navsat_imu_topic))

    # value_type=bool so the string from the CLI arrives as a real boolean;
    # a bare LaunchConfiguration would be declared as a string parameter.
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    # Level for the mapOptimization logger only - 'debug' surfaces the GPS
    # factor diagnostics without the rclcpp internals a process-wide
    # --log-level would pull in.
    map_opt_log_level = ['--ros-args', '--log-level',
                         ['lio_sam_mapOptimization:=',
                          LaunchConfiguration('log_level')]]

    return [
        # map -> odom. Keep this the ONLY publisher of that transform: FRUC's
        # scripts/static_transforms.py also publishes it, and running both
        # would give odom two parents.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
            ),
        # Publishes the sensor mounts from the URDF. For every FRUC platform
        # lidarFrame != baselinkFrame, so this is what makes TransformFusion's
        # lidar->baselink lookup resolvable.
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
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/navsat')],
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat',
            respawn=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            # ROS 2 subscribes to 'imu', not ROS 1's 'imu/data'.
            remappings=[('imu', navsat_imu_topic),
                        ('odometry/filtered', 'odometry/navsat')],
            output='screen'
        ),
        # No name= here: this process constructs TWO nodes (TransformFusion and
        # IMUPreintegration), and Node(name=...) becomes a process-wide
        # `-r __node:=` remap that would rename both to the same thing and
        # collide. They keep the names their ParamServer constructors assign.
        # The params files use the /** wildcard, so parameters still reach both.
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
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
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'platform',
            default_value='default',
            description='Robot/sensor preset: ' + ' | '.join(sorted(PLATFORMS))),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Override the platform default parameters file.'),
        DeclareLaunchArgument(
            'urdf_file',
            default_value='',
            description='Override the platform default URDF/xacro.'),
        # Follow /clock from `ros2 bag play --clock` instead of the system
        # clock. Set to false when running against live sensors.
        # (unchanged: same default_value='true' as before)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/bag clock published on /clock.'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logger level for lio_sam_mapOptimization (info|debug|warn).'),
        OpaqueFunction(function=launch_setup),
    ])
