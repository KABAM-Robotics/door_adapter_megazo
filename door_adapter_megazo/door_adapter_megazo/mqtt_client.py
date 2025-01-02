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

import sys
import yaml
import json
import time
import gmqtt
import rclpy
import asyncio
import argparse
import requests
import threading
from urllib.error import HTTPError
from rclpy.node import Node
from rmf_door_msgs.msg import DoorState, DoorMode

from .utils.mqttevents import MQTT_EVENTTYPE


class Door:
    """A Module that contains RMF Door-specific information."""

    def __init__(self,
                 door_id,
                 door_auto_closes,
                 door_signal_period,
                 continuous_status_polling,
                 enable_mqtt_status_polling):
        self.id = door_id
        self.door_mode = DoorMode.MODE_CLOSED
        self.open_door = False
        self.check_status = None  # set to None if not enabled
        self.door_auto_closes = door_auto_closes
        self.door_signal_period = door_signal_period
        self.enable_mqtt_status_polling = enable_mqtt_status_polling
        if continuous_status_polling:
            self.check_status = False

###############################################################################


class MQTTServer(Node):
    def __init__(self, config):
        super().__init__('megazo_mqtt_client')
        self.config = config
        self.timeout = 5
        self.publisher = self.create_publisher(DoorState, '/door_states', 10)

        # Keep track of doors
        self.doors = {}
        for door_id, door_data in self.config['doors'].items():
            # We support both door_auto_closes and the deprecated
            # door_close_feature for backward compatibility
            auto_close = door_data.get('door_auto_closes', None)
            if auto_close is None:
                if 'door_close_feature' in door_data:
                    auto_close = not door_data['door_close_feature']
            assert auto_close is not None

            self.doors[door_id] = Door(door_id,
                                       auto_close,
                                       door_data['door_signal_period'],
                                       door_data.get('continuous_status_polling', False),
                                       door_data.get('enable_mqtt_status_polling', False))

        self.door_to_projectID = {}
        self.door_to_mqtt_topic = {}
        for door_id in self.doors:
            self.door_to_projectID[door_id] = self.get_iced_list_deviceID(door_id)
            self.door_to_mqtt_topic[door_id] = f"ICEDEvent_{self.door_to_projectID[door_id]}"
            self.get_logger().info(f"Retrieved ProjectID and MQTT Topic for {door_id}...")

        # MQTT settings
        self.get_logger().info("Initializing MQTT Client to Megazo MQTT Broker...")
        self.broker_host = self.config['mqtt_broker']
        self.broker_port = self.config['mqtt_port']
        self.client_id = f"rmf_client_{int(time.time())}"
        self.client = None

        mqtt_thread = threading.Thread(target=self.set_up_mqtt)
        mqtt_thread.start()

    async def connect_mqtt(self):
        """Connect to the MQTT broker and start the loop."""
        # Initialize MQTT client
        self.client = gmqtt.Client(self.client_id)
        self.client.set_auth_credentials(
            username=self.config['header_key'],
            password=self.config['header_value']
            )
        self.client.set_config({
            'reconnect_retries': 10,
            'reconnect_delay': 5
        })
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.client.on_subscribe = self.on_subscribe

        self.get_logger().info('Signing into project...')

        for door_id in self.doors:
            while self.door_to_projectID[door_id] is None:
                self.get_logger().info("Waiting for ProjectID to be retrieved." +
                                       " Checking again in 5 seconds...")
                time.sleep(5)

            while self.door_to_mqtt_topic[door_id] is None:
                self.get_logger().info("Waiting for MQTT TOPIC to be defined." +
                                       " Checking again in 5 seconds...")
                time.sleep(5)

            self.get_logger().info('Retrieved ProjectID for' +
                                   f' {door_id}: {self.door_to_projectID[door_id]}')
            if self.door_to_projectID[door_id] is not None:
                self.project_sign_in(self.token, self.door_to_projectID[door_id])
            else:
                self.get_logger().error(f"Faulty ProjectID retrieved for {door_id}")
                sys.exit(1)

        await self.client.connect(self.broker_host, self.broker_port)
        await asyncio.sleep(2)

        while(True):
            await asyncio.sleep(1)

    def set_up_mqtt(self):
        # Start the MQTT connection in a separate thread
        self.get_logger().info("Connecting to Megazo MQTT Broker...")
        asyncio.run(self.connect_mqtt())

    def get_token(self):
        path = self.config["api_endpoint"] + "/API/System/Login"

        token_req_payload = {
            'data': {
                'UserID': self.config['header_key'],
                'Password': self.config['header_value']
            }
        }

        requestHeaders = {'Content-Type': 'application/json'}

        try:
            r = requests.post(
                path,
                data=json.dumps(token_req_payload),
                headers=requestHeaders,
                timeout=self.timeout
            )
            r.raise_for_status()
            data = r.json()

            if data["IsSuccess"]:
                return data["data"]["Token"]
        except requests.exceptions.ConnectionError as connection_error:
            self.get_logger().warn(f'Connection error: {connection_error}')
            return None
        except HTTPError as http_err:
            self.get_logger().warn(f'HTTP error: {http_err}')
            return None
        return None

    def get_iced_list_deviceID(self, door_id: str):
        path = self.config["api_endpoint"] + "/API/ICED/GetICEDList"

        self.get_logger().info('Retrieving Megazo Token...')
        self.token = self.get_token()

        payload = {
            'UserID': self.config["header_key"],
            'Token': self.token
        }
        requestHeaders = {'Content-Type': 'application/json'}

        try:
            r = requests.post(
                path,
                data=json.dumps(payload),
                headers=requestHeaders,
                timeout=self.timeout
            )
            r.raise_for_status()
            data = r.json()

            if data["IsSuccess"]:
                for device in data['data']:
                    if door_id in device['ICEDName']:
                        return device['ProjectID']
                self.ros_logger.warn(f"Unable to find ICEDName for door {door_id}. "
                                     "Please ensure config.yaml contains the correct door name."
                                     " - Returning None.")
                return None
            else:
                self.ros_logger.warn("\n[get_iced_list_projectID] - [FAIL]")
                return None

        except requests.exceptions.ConnectionError as connection_error:
            self.ros_logger.warn(f'Connection error: {connection_error}')
            return None
        except HTTPError as http_err:
            self.ros_logger.warn(f'HTTP error: {http_err}')
            return None

    def project_sign_in(self, token: str, projectID: str):
        path = self.config["api_endpoint"] + "/API/System/ProjectSignIn"

        payload = {
            'UserID': self.config["header_key"],
            'Token': token,
            'data': {
                'ProjectID': projectID
            }
        }
        requestHeaders = {'Content-Type': 'application/json'}

        try:
            r = requests.post(
                path,
                data=json.dumps(payload),
                headers=requestHeaders,
                timeout=self.timeout
            )
            r.raise_for_status()
            data = r.json()

            if data["IsSuccess"]:
                return data['data']['ProjectID']
            else:
                self.ros_logger.error("\n[signInProject] - [FAIL]")
                return None

        except requests.exceptions.ConnectionError as connection_error:
            self.ros_logger.warn(f'Connection error: {connection_error}')
            return None
        except HTTPError as http_err:
            self.ros_logger.warn(f'HTTP error: {http_err}')
            return None

    def on_connect(self, client, userdata, flags, rc):
        """Execute when connected to MQTT broker."""
        self.get_logger().info(f'Connected to MQTT broker with result code {rc}')
        # Subscribe to a topic if needed, for example:
        mqtt_topic_array = []
        mqtt_topic_filtered_array = []
        for value in self.door_to_mqtt_topic.values():
            mqtt_topic_array.append(value)
        # Remove duplicates
        seen = set()
        for topic in mqtt_topic_array:
            if topic not in seen:
                seen.add(topic)
                mqtt_topic_filtered_array.append(topic)

        for topic in mqtt_topic_filtered_array:
            self.get_logger().info(f'Subcribing to MQTT Topic: {topic}')
            client.subscribe(topic, qos=1)

        self.client = client

    def on_message(self, client, topic, payload, qos, properties):
        """Execute when a new message is received on MQTT topic."""
        try:
            # Attempt to parse the string as JSON
            json_obj = json.loads(payload.decode())
        except json.JSONDecodeError:
            self.get_logger().warn("Input is a string, but not a valid JSON object.")

        if json_obj['doorEvent']['EventType'] != 16:
            self.decode_megazo_mqtt(json_obj)
            # print(f"json_obj = {json_obj}")
            curr_event_type = json_obj['doorEvent']['EventType']
            state_msg = DoorState()
            state_msg.door_time = self.get_clock().now().to_msg()

            for door_id, _ in self.doors.items():
                if door_id in json_obj['doorEvent']['ControllerName']:
                    state_msg.door_name = door_id
                    if curr_event_type in [3, 1020, 1022]:
                        state_msg.current_mode.value = DoorMode.MODE_CLOSED
                    elif curr_event_type in [4, 5, 11, 14, 21, 22]:
                        state_msg.current_mode.value = DoorMode.MODE_OPEN
                    else:
                        state_msg.current_mode.value = DoorMode.MODE_CLOSED
                    self.publisher.publish(state_msg)

    def on_disconnect(self, client, packet, exc=None):
        self.get_logger().info("MQTT Disconnected")

    def on_subscribe(self, client, mid, qos, properties):
        self.get_logger().info(f"Subscribed with message ID {mid}")

    def decode_megazo_mqtt(self, json_obj):
        # Check if input_data is a string and try to parse it as JSON
        try:
            self.get_logger().info("EventType - " +
                                   f"{json_obj['doorEvent']['ControllerName']}:" +
                                   f" {MQTT_EVENTTYPE[str(json_obj['doorEvent']['EventType'])]}")
        except KeyError:
            self.get_logger().warn("Unexpected EventType - " +
                                   f"{json_obj['doorEvent']['ControllerName']}:" +
                                   f" {json_obj['doorEvent']['EventType']}")


def main(argv=sys.argv):
    rclpy.init(args=argv)

    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="mqtt_client",
        description="Configure and spin up door adapter for door ")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file for this door adapter")
    args = parser.parse_args(args_without_ros[1:])
    config_path = args.config_file

    # Load config and nav graph yamls
    with open(config_path, "r", encoding="utf-8") as f:
        config_yaml = yaml.safe_load(f)

    # Instantiate the MQTTServer node
    mqtt_server_node = MQTTServer(config=config_yaml)

    try:
        rclpy.spin(mqtt_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        mqtt_server_node.client.loop_stop()  # Stop the MQTT client loop
        mqtt_server_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
