from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, LoadComposableNodes
from nav2_common.launch import RewrittenYaml
from launch.substitutions import NotEqualsSubstitution
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
                          
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),

    DeclareLaunchArgument('use_composition', default_value='False',
                            description="Launch as individual processes"),

    DeclareLaunchArgument('container_name', default_value='nav2_container',
                description='The name of container that nodes will load in if use composition'),
    
    DeclareLaunchArgument('autostart', default_value='True',
                          description = 'whether or not to start the navigation stack'),

    DeclareLaunchArgument('filter', default_value='hospital_keepout_zones.yaml',
                          description='yaml file that points to .png for keepout zone')
]



def generate_launch_description():
    pkg_courier_navigation = get_package_share_directory('courier_navigation')
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    autostart = LaunchConfiguration('autostart')
    keepout_filter = LaunchConfiguration('filter')
    container_name_full = (namespace, '/', container_name)

    keepout_path = PathJoinSubstitution([pkg_courier_navigation, 'map_filters', keepout_filter])

    keepout_param_args = DeclareLaunchArgument(
        'keepout_params',
        default_value=PathJoinSubstitution(
            [pkg_courier_navigation, 'config', 'keepout_params.yaml']),
        description='Keepout_parameters')

    keepout_param = LaunchConfiguration('keepout_params')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': keepout_path
    }

    configured_params = RewrittenYaml(
        source_file = keepout_param,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )


    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[configured_params]),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[configured_params]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_costmap_filters',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]) 
        ]
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            PushRosNamespace(
                condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('namespace'), '')),
                namespace=namespace),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='filter_mask_server',
                        parameters=[configured_params]),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::CostmapFilterInfoServer',
                        name='costmap_filter_info_server',
                        parameters=[configured_params]),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_costmap_filters',
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
                ]
            )
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(keepout_param_args)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    return ld