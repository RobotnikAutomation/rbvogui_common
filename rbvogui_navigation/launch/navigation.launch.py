# Copyright (c) 2022, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from robotnik_common.launch import RewrittenYaml

# Environment variables
#  USE_SIM_TIME: Use simulation (Gazebo) clock if true
#  NAMESPACE: Namespace of the node stack.
#  ROBOT_ID: Frame id of the robot. (e.g. vectornav_link).
#  MAP_FILE: Path to the map file.
#  MAP_FILE_ABS: Absolute path to the map file.

def read_params(ld : launch.LaunchDescription):
    environment = launch.substitutions.LaunchConfiguration('environment')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    nav_config_file = launch.substitutions.LaunchConfiguration('nav_config_file')

    # Declare the launch options
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='environment',
        description='Read params from environment variables.',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        description='Use simulation (Gazebo) clock if true',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='robot_id',
        description='Frame id of the sensor. (e.g. robot).',
        default_value='robot_')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='namespace',
        description='Namespace of the nodes.',
        default_value='robot')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='nav_config_file',
        description='Absolute path to the amcl file.',
        default_value=[get_package_share_directory('rbvogui_navigation'), '/config/nav.yaml'])
    )
    
    # Parse the launch options
    ret = {}

    if environment == 'false':
        ret = {
        'use_sim_time': use_sim_time,
        'namespace': namespace,
        'robot_id': robot_id,
        'nav_config_file': nav_config_file
        }
    
    else:
        if 'USE_SIM_TIME' in os.environ:
            ret['use_sim_time'] =  launch.substitutions.EnvironmentVariable('USE_SIM_TIME')
        else: ret['use_sim_time'] = use_sim_time

        if 'ROBOT_ID' in os.environ:
            ret['robot_id'] = launch.substitutions.EnvironmentVariable('ROBOT_ID')
        else: ret['robot_id'] = robot_id

        if 'NAMESPACE' in os.environ:
            ret['namespace'] = launch.substitutions.EnvironmentVariable('NAMESPACE')
        elif 'ROBOT_ID' in os.environ:
            ret['namespace'] = launch.substitutions.EnvironmentVariable('ROBOT_ID')
        else:  ret['namespace'] = namespace

        if 'NAV_CONFIG_FILE' in os.environ:
            ret['nav_config_file'] = launch.substitutions.EnvironmentVariable('NAV_CONFIG_FILE')
        else: ret['nav_config_file'] = nav_config_file

    return ret


def generate_launch_description():

    ld = launch.LaunchDescription()
    params = read_params(ld)

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    remappings = [
        ('/odom', '/robot/robotnik_base_controller/odom'),
        ('/front_laser/scan', ['/', params['namespace'], '/front_laser/scan']),
        ('/rear_laser/scan',  ['/', params['namespace'], '/rear_laser/scan'])
    ]

    configured_params = RewrittenYaml(
            source_file=params['nav_config_file'],
            root_key=params['namespace'],
            param_rewrites={
                'use_sim_time': params['use_sim_time'],
                'robot_base_frame': [params['robot_id'], 'base_link'],
            },
            convert_types=True)

    stdout_linebuf_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    load_nodes = launch.actions.GroupAction(
        actions=[
            launch_ros.actions.Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    {'use_sim_time': params['use_sim_time']},
                    configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            launch_ros.actions.Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    {'use_sim_time': params['use_sim_time']},
                    configured_params],
                remappings=remappings),
            launch_ros.actions.Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    {'use_sim_time': params['use_sim_time']},
                    configured_params],
                remappings=remappings),
            launch_ros.actions.Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    {'use_sim_time': params['use_sim_time']},
                    configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            launch_ros.actions.Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    {'use_sim_time': params['use_sim_time']},
                    configured_params],
                remappings=remappings),
            launch_ros.actions.Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    {'use_sim_time': params['use_sim_time']},
                    configured_params],
                remappings=remappings),
            launch_ros.actions.Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    {'use_sim_time': params['use_sim_time']},
                    configured_params],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', ['/',params['namespace'], '/robotnik_base_controller/cmd_vel'])]),
            launch_ros.actions.Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': params['use_sim_time'],
                    'autostart': True,
                    'node_names': lifecycle_nodes}]),
        ]
    )

    # Set environment variables
    ld.add_action(launch_ros.actions.PushRosNamespace(namespace=params['namespace']))
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(load_nodes)

    return ld
