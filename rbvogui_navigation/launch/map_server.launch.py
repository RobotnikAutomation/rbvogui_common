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
    map_name = launch.substitutions.LaunchConfiguration('map_name')
    map_file_abs = launch.substitutions.LaunchConfiguration('map_file_abs')
    amcl_file = launch.substitutions.LaunchConfiguration('amcl_file')
    map_frame_id = launch.substitutions.LaunchConfiguration('map_frame_id')

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
        default_value='robot')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='namespace',
        description='Namespace of the nodes.',
        default_value=robot_id)
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='map_name',
        description='Name of the map file.',
        default_value='map')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='map_file_abs',
        description='Absolute path to the map file.',
        default_value=[get_package_share_directory('rbvogui_navigation'), '/maps/', map_name, '.yaml'])
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='amcl_file',
        description='Absolute path to the amcl file.',
        default_value=[get_package_share_directory('rbvogui_navigation'), '/config/amcl.yaml'])
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='map_frame_id',
        description='Frame id of the map.',
        default_value=[robot_id, '_map'])
    )
    
    # Parse the launch options
    ret = {}

    if environment == 'false':
        ret = {
        'use_sim_time': use_sim_time,
        'namespace': namespace,
        'robot_id': robot_id,
        'map_file_abs': map_file_abs,
        'amcl_file': amcl_file,
        'map_frame_id': map_frame_id
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

        if 'MAP_FILE_ABS' in os.environ:
            ret['map_file_abs'] = launch.substitutions.EnvironmentVariable('MAP_FILE_ABS')
        if 'MAP_NAME' in os.environ:
            ret['map_file_abs'] = [get_package_share_directory('rbvogui_navigation'), '/maps/', launch.substitutions.EnvironmentVariable('MAP_NAME'), '.yaml']
        else: ret['map_file_abs'] = map_file_abs

        if 'AMCL_FILE' in os.environ:
            ret['amcl_file'] = launch.substitutions.EnvironmentVariable('AMCL_FILE')
        else: ret['amcl_file'] = amcl_file

        if 'MAP_FRAME_ID' in os.environ:
            ret['map_frame_id'] = launch.substitutions.EnvironmentVariable('MAP_FRAME_ID')
        else: ret['map_frame_id'] = map_frame_id

    return ret


def generate_launch_description():
    
    ld = launch.LaunchDescription()
    params = read_params(ld)

    lifecycle_nodes = ['map_server', 'amcl']

    amcl_rewritten = RewrittenYaml(
        source_file=params['amcl_file'],
        param_rewrites={
            'use_sim_time': params['use_sim_time'],
            'base_frame_id': [params['robot_id'], '_base_footprint'],
            'odom_frame_id': [params['robot_id'], '_odom'],
            'global_frame_id': params['map_frame_id'],
        },
        root_key=[params['robot_id']],
        convert_types=True,
    )

    map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'use_sim_time': params['use_sim_time'],
            'yaml_filename': params['map_file_abs'],
            'topic_name': 'map',
            'frame_id': params['map_frame_id'],
        }],
        output='screen',
        namespace=params['namespace']
        )
    
    amcl_node = launch_ros.actions.Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            {'use_sim_time': params['use_sim_time']},
            amcl_rewritten
        ],
        output='screen',
        namespace=params['namespace'])

    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': params['use_sim_time'],
            'autostart': True,
            'node_names': lifecycle_nodes}])
    
    ld.add_action(map_server)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)

    return ld
