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
#  CONTROLLERS_FILE: Path to the controllers.yaml file.
#  ROBOT_DESCRIPTION: Name of the robot description file.
#  ROBOT_DESCRIPTION_PATH: Path to the robot description file.

def read_params(ld : launch.LaunchDescription):
    environment = launch.substitutions.LaunchConfiguration('environment')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    robot_description = launch.substitutions.LaunchConfiguration('robot_description')
    robot_description_path = launch.substitutions.LaunchConfiguration('robot_description_path')
    controllers_file = launch.substitutions.LaunchConfiguration('controllers_file')
    cart = launch.substitutions.LaunchConfiguration('cart')
    launch_joint = launch.substitutions.LaunchConfiguration('launch_joint')

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
        name='robot_description',
        description='Robot description.',
        default_value='rbvogui_std.urdf.xacro')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='robot_description_path',
        description='Path to the robot description file.',
        default_value=[get_package_share_directory('rbvogui_description'), '/robots/', robot_description])
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='controllers_file',
        description='ROS 2 controller file.',
        default_value=[get_package_share_directory('rbvogui_description'), '/test/empty.yaml'])
    )
    
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='cart',
        description='bool rbvogui with cart',
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='launch_joint',
        description='launch joint_state_publisher_gui',
        default_value='false')
    )

    # Parse the launch options
    ret = {}

    if environment == 'false':
        ret = {
        'use_sim_time': use_sim_time,
        'namespace': namespace,
        'robot_id': robot_id,
        'controllers_file': controllers_file,
        'robot_description_path': robot_description_path,
        'cart': cart,
        'launch_joint': launch_joint
        }
    
    else:
        if 'USE_SIM_TIME' in os.environ:
            ret['use_sim_time'] = os.environ['USE_SIM_TIME']
        else: ret['use_sim_time'] = use_sim_time

        if 'ROBOT_ID' in os.environ:
            ret['robot_id'] = os.environ['ROBOT_ID']
        else: ret['robot_id'] = robot_id

        if 'NAMESPACE' in os.environ:
            ret['namespace'] = os.environ['NAMESPACE']
        elif 'ROBOT_ID' in os.environ:
            ret['namespace'] = os.environ['ROBOT_ID']
        else:  ret['namespace'] = namespace

        if 'CONTROLLERS_FILE' in os.environ:
            ret['controllers_file'] = os.environ['CONTROLLERS_FILE']
        else: ret['controllers_file'] = controllers_file

        if 'ROBOT_DESCRIPTION_PATH' in os.environ:
            ret['robot_description_path'] = os.environ['ROBOT_DESCRIPTION_PATH']
        elif 'ROBOT_DESCRIPTION' in os.environ:
            ret['robot_description_path'] = [get_package_share_directory('rbvogui_description'), '/robots/', os.environ['ROBOT_DESCRIPTION']]
        else: ret['robot_description_path'] = robot_description_path

        if 'CART' in os.environ:
            ret['cart'] = os.environ['CART']
        else:  ret['cart'] = cart

        ret['launch_joint'] = launch_joint

    return ret


def generate_launch_description():

    ld = launch.LaunchDescription()

    params = read_params(ld)
    
    config_file_rewritten = RewrittenYaml(
        source_file=params['controllers_file'],
        param_rewrites={},
        root_key=[params['namespace'],],
        convert_types=True,
    )

    robot_description_content = launch.substitutions.Command(
        [
            launch.substitutions.PathJoinSubstitution(
                [launch.substitutions.FindExecutable(name="xacro")]),
            " ",
            params['robot_description_path'],
            " robot_id:=", params['robot_id'],
            " prefix:=",   params['robot_id'], "_",
            " kinematics:=omni",
            " load_kinematics_file:=false",
            " gpu:=false",
            " publish_bf:=true",
            " hq:=true",
            " launch_arm:=false",
            " arm_manufacturer:=false",
            " arm_model:=false",
            " launch_gripper:=false" ,
            " gripper_manufacturer:=false",
            " gripper_model:=false",
            " launch_lift:=false",
            " lift_manufacturer:=false",
            " lift_model:=false",
            " config_controllers:=", config_file_rewritten,
            " cart:=", params['cart']
        ]
    )

    # Create parameter 
    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': params['use_sim_time'],
            'robot_description': robot_description_param,
            'publish_frequency': 100.0,
            'frame_prefix': ''
        }],
    )

    robot_joint_publisher = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    ld.add_action(launch_ros.actions.PushRosNamespace(namespace=params['namespace']))
    ld.add_action(robot_state_publisher)
    if params['launch_joint']:
        ld.add_action(robot_joint_publisher)

    return ld
