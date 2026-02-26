import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import PushRosNamespace, SetRemap, Node
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
    DeclareLaunchArgument('scan_topic',
                          default_value='',
                          description='Override the default 2D laserscan topic'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.'),  # noqa: E501
    DeclareLaunchArgument('use_lifecycle_manager', default_value='false',
                          choices=['true', 'false'],
                          description='Enable bond connection during node activation'),    
    # DeclareLaunchArgument('use_mocap_localization', default_value='false',
    #                       choices=['true', 'false'],
    #                       description='Enable the Mocap ground truth data as localization source. If this is disable, a localizer or a publisher should provide the transformation from map to odom frame.'),
    # DeclareLaunchArgument('use_mocap_odom', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Enable the Mocap ground truth data as odometry source. Note that the EKF node should be diabled in the robot.yaml file if this is enabled, since the EKF node will fuse the mocap ground truth data and the original odometry data to provide filtered odometry data, which can cause issues if the mocap ground truth data is used as odometry source for localization and navigation at the same time.'),
    DeclareLaunchArgument('is_Local_EKF_running', default_value='true',
                          choices=['true', 'false'],
                          description='Whether the local EKF node is running to provide filtered odometry data. If this is true, the mocap fake EKF node will be disabled since it will fuse the mocap ground truth data and the original odometry data to provide filtered odometry data, which can cause issues if the mocap ground truth data is used as odometry source for localization and navigation at the same time.'),
    DeclareLaunchArgument('is_Localizer_running', default_value='false',
                          choices=['true', 'false'],
                          description='Whether the localizer node is running to provide localization data. If this is true, the mocap fake localizer node will be disabled since it will provide localization data using mocap ground truth data, which can cause issues if the mocap ground truth data is used as localization source for localization and navigation at the same time.'),
    DeclareLaunchArgument('use_mocap_map_frame', default_value='false',
                          choices=['true', 'false'],
                          description=''),

]


def launch_setup(context, *args, **kwargs):
    pkg_clearpath_nav2_demos = get_package_share_directory('mtu32_bringup')
    # pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    setup_path = LaunchConfiguration('setup_path')
    # use_mocap_localization = LaunchConfiguration('use_mocap_localization')
    # use_mocap_odom = LaunchConfiguration('use_mocap_odom')
    is_Local_EKF_running = LaunchConfiguration('is_Local_EKF_running')
    is_Localizer_running = LaunchConfiguration('is_Localizer_running')
    use_mocap_map_frame = LaunchConfiguration('use_mocap_map_frame')


    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    remappings_tf=[
                ('/tf','tf'),
                ('/tf_static','tf_static'),
            ]

    load_nodes = GroupAction(        
        actions=[
            Node(
                package='mocap_fake_localizer', # tf: map -> odom
                executable='mocap_fake_localizer_node',
                name='mocap_fake_localizer_node',
                output='screen',
                namespace=f'/{namespace}',
                parameters=[
                    PathJoinSubstitution(
                        [get_package_share_directory("mtu32_bringup"), "config", f'{platform_model}', "fake_localizer_config.yaml"]
                    )
                ],
                remappings= remappings_tf,        
            ),            
        ],
    )

    return [load_nodes]





def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld