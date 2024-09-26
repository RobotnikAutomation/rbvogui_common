#!/usr/bin/env python3

# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
import launch_ros
from nav2_common.launch import RewrittenYaml

def read_params(ld : launch.LaunchDescription):
    environment = launch.substitutions.LaunchConfiguration('environment')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    image_topic = launch.substitutions.LaunchConfiguration('image_topic')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    # Declare the launch options
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='environment',
        description='Read params from environment variables.',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='namespace',
        description='Namespace of the node.',
        default_value='robot')
    )
    
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='image_topic',
        description='image_topic for aruco detection (default: rear_rgbd_camera/color)',
        default_value='rear_rgbd_camera/color')
    )
    
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        description='Use simulation (Gazebo) clock if true',
        choices=['true', 'false'],
        default_value='true')
    )
    # Parse the launch options
    ret = {}

    if environment == 'false':
        ret = {
        'namespace': namespace,
        'image_topic': image_topic,
        'use_sim_time': use_sim_time,
        }
    
    else:
        if 'NAMESPACE' in os.environ:
            ret['namespace'] = os.environ['NAMESPACE']
        else:  ret['namespace'] = namespace
        if 'ARUCOIMAGETOPIC' in os.environ:
            ret['image_topic'] = os.environ['ARUCOIMAGETOPIC']
        else:  ret['image_topic'] = image_topic

        ret['use_sim_time']= use_sim_time

    return ret

def generate_launch_description():

    ld = launch.LaunchDescription()
    params = read_params(ld)

    rbvogui_dock_params_dir = os.path.join(
        get_package_share_directory('rbvogui_docking'), 'params')

    params_file =os.path.join(rbvogui_dock_params_dir, 'rbvogui_docking.yaml')
    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=params['namespace'],
            param_rewrites={
                'use_sim_time': params['use_sim_time'],
                'base_frame': [params['namespace'], '_base_link'],
                'fixed_frame': [params['namespace'], '_odom'],
                'home_dock.frame':[params['namespace'], '_map'],

            },
            convert_types=True)

    # aruco_node = launch_ros.actions.Node(
    #     package='ros2_aruco',
    #     executable='aruco_node',
    #     name='aruco_node',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': params['use_sim_time']},
    #         {'marker_size': 0.10},
    #         {'aruco_dictionary_id': 'DICT_4X4_100'},
    #         {'image_topic': [params['image_topic'], '/image_raw']},
    #         {'camera_info_topic': [params['image_topic'], '/camera_info']},
    #     ],
    # )

    # dock_pose_publisher = launch_ros.actions.Node(
    #     package='rbvogui_docking',
    #     executable='dock_pose_publisher',
    #     name='dock_pose_publisher',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': params['use_sim_time']},
    #     ]
    # )

    static_dock_pose_publisher = launch_ros.actions.Node(
        package='rbvogui_docking',
        executable='static_dock_pose_publisher',
        name='static_dock_pose_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': params['use_sim_time']},
        ]
    )

    docking_server = launch_ros.actions.Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[
            {'use_sim_time': params['use_sim_time']},
            configured_params
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
    )

    ld.add_action(launch_ros.actions.PushRosNamespace(namespace=params['namespace']))
    ld.add_action(docking_server)
    ld.add_action(lifecycle_manager)
    # ld.add_action(aruco_node)
    ld.add_action(static_dock_pose_publisher)

    return ld