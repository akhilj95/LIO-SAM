import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# There is exactly one thing to choose: the parameters file. Everything that
# varies per robot travels inside it, so a config cannot be paired with the
# wrong URDF:
#
#   urdfFile         robot model to publish, relative to config/
#   navsatImuTopic   topic navsat_transform_node's 'imu' input is remapped to.
#                    ROS 2 subscribes to 'imu' where ROS 1 used 'imu/data', and
#                    FRUC remaps it per robot in module_navsat.launch.
#
# Both are optional; omitting them gives the stock LIO-SAM setup. No node
# declares them, so they are ignored as parameters and only read here.
DEFAULT_URDF = 'robot.urdf.xacro'
DEFAULT_NAVSAT_IMU = 'imu_correct'


def resolve(path, config_dir):
    """Bare names resolve against the package config dir; paths are taken as-is."""
    return path if os.path.isabs(path) else os.path.join(config_dir, path)


def launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory('lio_sam')
    config_dir = os.path.join(share_dir, 'config')
    rviz_config_file = os.path.join(config_dir, 'rviz2.rviz')

    params_file = resolve(
        LaunchConfiguration('params_file').perform(context), config_dir)
    if not os.path.exists(params_file):
        raise RuntimeError(
            'params file not found: {}\nAvailable in {}: {}'.format(
                params_file, config_dir,
                ', '.join(sorted(f for f in os.listdir(config_dir)
                                 if f.endswith('.yaml')))))

    with open(params_file) as fh:
        common = (yaml.safe_load(fh) or {}).get('/**', {}).get('ros__parameters', {})

    xacro_path = resolve(common.get('urdfFile', DEFAULT_URDF), config_dir)
    if not os.path.exists(xacro_path):
        raise RuntimeError("urdfFile '{}' from {} does not exist: {}".format(
            common.get('urdfFile'), os.path.basename(params_file), xacro_path))
    navsat_imu_topic = common.get('navsatImuTopic', DEFAULT_NAVSAT_IMU)

    print('lio_sam params : {}'.format(os.path.basename(params_file)))
    print('  urdf         : {}'.format(os.path.basename(xacro_path)))
    print('  navsat imu   : {}'.format(navsat_imu_topic))

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
            'params_file',
            default_value='params.yaml',
            description='Parameters file: a name in config/, or an absolute '
                        'path. It also selects the URDF and the navsat IMU '
                        'topic, via its urdfFile / navsatImuTopic entries.'),
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
