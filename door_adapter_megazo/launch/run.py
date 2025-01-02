# Copyright 2024 Kabam Pte Ltd, All Rights Reserved.
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

import yaml
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    config_path = "/door_adapter_megazo_ws/src/door_adapter_megazo/configs/config.yaml"

    # Load config and nav graph yamls
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            config_yaml = yaml.safe_load(f)
    except FileNotFoundError as e:
        raise RuntimeError(f"YAML configuration file not found: {e}")
    except yaml.YAMLError as e:
        raise RuntimeError(f"Error parsing YAML file: {e}")

    enable_mqtt = False
    for door_id, door_data in config_yaml['doors'].items():
        if door_data.get('enable_mqtt_status_polling', False):
            print(f"MQTT Status Polling is enabled for {door_id}." +
                  " Starting up [megazo_mqtt_client]...")
            enable_mqtt = True
            break

    nodes = [
        # Add a delay before launching the second node (3 seconds)
        TimerAction(
            period=5.0,  # Time in seconds to wait
            actions=[
                LogInfo(msg="5 seconds passed, " +
                        "now launching [door_adapter_megazo] to avoid token conflict..."),
                Node(
                    package='door_adapter_megazo',
                    executable='door_adapter',
                    name='door_adapter_megazo',
                    output='screen',
                    arguments=["--config_file", config_path, "--enable_mqtt", str(enable_mqtt)],
                )
            ]
        )
    ]

    if enable_mqtt:
        nodes.append(Node(
            package='door_adapter_megazo',
            executable='mqtt_client',
            name='megazo_mqtt_client',
            output='screen',
            arguments=["--config_file", config_path],
            )
        )

    return LaunchDescription(nodes)
