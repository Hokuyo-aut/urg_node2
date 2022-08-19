# Copyright 2022 eSOL Co.,Ltd.
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
import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# 2台LiDAR接続時のurg_node2複数起動

def generate_launch_description():

    # パラメータファイルのパス設定（1台目）
    config_file_path_1st = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_ether.yaml'
    )

    # パラメータファイルのパス設定（2台目）
    config_file_path_2nd = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_ether_2nd.yaml'
    )

    # パラメータファイルのロード（1台目）
    with open(config_file_path_1st, 'r') as file:
        config_params_1st = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # パラメータファイルのロード（2台目）
    with open(config_file_path_2nd, 'r') as file:
        config_params_2nd = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # urg_node2をライフサイクルノードとして起動（1台目）
    lifecycle_node_1st = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name_1st'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name_1st'))],
        parameters=[config_params_1st],
        namespace='',
        output='screen',
    )

    # urg_node2をライフサイクルノードとして起動（2台目）
    lifecycle_node_2nd = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name_2nd'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name_2nd'))],
        parameters=[config_params_2nd],
        namespace='',
        output='screen',
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）（1台目）
    urg_node2_node_1st_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node_1st,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_1st),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）（1台目）
    urg_node2_node_1st_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node_1st,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_1st),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）（2台目）
    urg_node2_node_2nd_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node_2nd,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_2nd),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）（2台目）
    urg_node2_node_2nd_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node_2nd,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_2nd),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # パラメータについて
    # auto_start          : 起動時自動でActive状態まで遷移 (default)true
    # node_name_1st       : 1台目のノード名 (default)"urg_node2_1st"
    # node_name_2nd       : 2台目のノード名 (default)"urg_node2_2nd"
    # scan_topic_name_1st : トピック名 (default)"scan_1st" *マルチエコー非対応*
    # scan_topic_name_2nd : トピック名 (default)"scan_2nd" *マルチエコー非対応*
    # *マルチエコーを使用する場合は上部のnode起動部のremappingsを直接編集してください*
    # *ロードするパラメータファイルは上部のファイル名を直接編集してください*
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('node_name_1st', default_value='urg_node2_1st'),
        DeclareLaunchArgument('node_name_2nd', default_value='urg_node2_2nd'),
        DeclareLaunchArgument('scan_topic_name_1st', default_value='scan_1st'),
        DeclareLaunchArgument('scan_topic_name_2nd', default_value='scan_2nd'),
        lifecycle_node_1st,
        lifecycle_node_2nd,
        urg_node2_node_1st_configure_event_handler,
        urg_node2_node_1st_activate_event_handler,
        urg_node2_node_2nd_configure_event_handler,
        urg_node2_node_2nd_activate_event_handler,
    ])
