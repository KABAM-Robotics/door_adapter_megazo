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

"""Module that runs a ROS 2 node for Megazo door to talk to RMF."""

import sys
import time
import argparse
import threading
import yaml

import rclpy
from rclpy.node import Node
from rmf_door_msgs.msg import DoorRequest, DoorState, DoorMode
from door_adapter_megazo.DoorClientAPI import DoorClientAPI


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


class DoorAdapter(Node):
    """A Module that bridges between ROS 2 and Megazo Door API."""

    def __init__(self, config_yaml, is_mqtt_enabled):
        super().__init__('door_adapter_megazo')
        self.get_logger().info('Initialising [door_adapter_megazo]...')

        # Get value from config file
        self.door_state_publish_period = config_yaml['door_publisher']['door_state_publish_period']

        door_pub = config_yaml['door_publisher']
        door_sub = config_yaml['door_subscriber']
        self.mock_adapter = config_yaml.get('mock', False)
        self.enable_mqtt = False
        self.periodic_timer = None

        # Connect to doors
        if not self.mock_adapter:
            self.api = DoorClientAPI(self, config_yaml, self.get_logger())

            assert self.api.connected, "Unable to establish connection with door"

            # Keep track of doors
            self.doors = {}
            for door_id, door_data in config_yaml['doors'].items():
                # We support both door_auto_closes and the deprecated
                # door_close_feature for backward compatibility
                auto_close = door_data.get('door_auto_closes', None)
                if auto_close is None:
                    if 'door_close_feature' in door_data:
                        auto_close = not door_data['door_close_feature']
                assert auto_close is not None

                if door_data.get('enable_mqtt_status_polling', False):
                    self.enable_mqtt = True

                self.doors[door_id] = Door(door_id,
                                           auto_close,
                                           door_data['door_signal_period'],
                                           door_data.get('continuous_status_polling', False),
                                           door_data.get('enable_mqtt_status_polling', False))

        self.door_states_pub = self.create_publisher(
            DoorState, door_pub['topic_name'], 100)

        self.door_request_sub = self.create_subscription(
            DoorRequest, door_sub['request_topic_name'], self.door_request_cb, 100)

        if not self.enable_mqtt:
            self.periodic_timer = self.create_timer(
                self.door_state_publish_period, self.time_cb)

    def door_open_command_request(self, door_data: Door):
        """
        Continuously sends an API request to open a door until it's closed.

        Keeps attempting to open the specified door using the provided data.
        It will wait for a period of time (door_signal_period) before retrying if the
        previous attempt was unsuccessful or successful but not acknowledged by the system.

        Args:
        ----
            door_data (Door): An object containing information about the door to be opened,
            including its ID and signal period.

        """
        while door_data.open_door:
            success = self.api.open_door(door_data.id)
            if success:
                self.get_logger().info(f"Request to open door [{door_data.id}] is successful")
            else:
                self.get_logger().warning(f"Request to open door [{door_data.id}] is unsuccessful")
            time.sleep(door_data.door_signal_period)

    def time_cb(self):
        """
        Periodically updates and publishes the status of all Megazo doors.

        This method is responsible for updating the state of each door by
        querying its current mode.
        It also takes into account whether continuous_status_polling is enabled,
        which affects how often it checks the door's status.
        If enabled, it only updates the door state when there are open or close requests.
        Otherwise, it continuously polls the door's status.

        """
        if self.mock_adapter:
            return
        for door_id, door_data in self.doors.items():

            if door_data.check_status is not None:
                # If continuous_status_polling is enabled, we will only update
                # the door state when there is a door open request. If there is
                # a close door request and the door state is closed, we will
                # assume the door state remains closed until the next door open
                # request. This implementation reduces the number of calls made
                # during state update.
                if door_data.check_status:
                    door_data.door_mode = self.api.get_mode(door_id)
                    if door_data.door_mode == DoorMode.MODE_CLOSED and not door_data.open_door:
                        door_data.check_status = False
            else:
                # If continuous_status_polling is not enabled, we'll just
                # update the door state as it is all the time
                door_data.door_mode = self.api.get_mode(door_id)
            state_msg = DoorState()
            state_msg.door_time = self.get_clock().now().to_msg()

            # publish states of the door
            state_msg.door_name = door_id
            state_msg.current_mode.value = door_data.door_mode
            self.door_states_pub.publish(state_msg)

    def door_request_cb(self, msg: DoorRequest):
        """
        Handle incoming requests to open or close doors.

        This method processes messages of type `DoorRequest` and
        updates the state of the corresponding door accordingly.
        It checks if continuous status polling is enabled per-door basis
        and toggles it as needed when a request is received.
        If running in mock adapter mode, it automatically agrees to every
        request without sending any commands to the API.

        Args:
        ----
            msg (DoorRequest): A message containing information about the
            requested door operation.

        """
        # Agree to every request automatically if this is a mock adapter
        if self.mock_adapter:
            state_msg = DoorState()
            state_msg.door_time = self.get_clock().now().to_msg()
            state_msg.door_name = msg.door_name
            state_msg.current_mode.value = msg.requested_mode.value
            self.door_states_pub.publish(state_msg)
            return

        # Check if this door has been stored in the door adapter. If not, ignore
        door_data = self.doors.get(msg.door_name)
        if door_data is None:
            return

        # When the adapter receives an open request, it will send an open
        # command to API. When the adapter receives a close request, it will
        # stop sending the open command to API
        self.get_logger().info(
            f"[{msg.door_name}] Door mode [{msg.requested_mode.value}]"
            f"requested by {msg.requester_id}"
        )
        if msg.requested_mode.value == DoorMode.MODE_OPEN:
            # open door implementation
            door_data.open_door = True
            if door_data.check_status is not None:
                # If check_status is enabled, we toggle it to true to allow
                # door state updates
                door_data.check_status = True
            if not door_data.door_auto_closes:
                self.api.open_door(msg.door_name)
            else:
                t = threading.Thread(target=self.door_open_command_request,
                                     args=(door_data,))
                t.start()
        elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
            # close door implementation
            door_data.open_door = False
            self.get_logger().info(f'[{msg.door_name}] Command to [CLOSE DOOR] received')
            if not door_data.door_auto_closes:
                self.api.close_door(msg.door_name)
        else:
            self.get_logger().error('Invalid door mode requested. Ignoring...')

###############################################################################


def main(argv=sys.argv):
    """
    Initialize the ROS2 environment.

    It also parses command-line arguments, loads configuration from a
    YAML file, and spins up an instance of DoorAdapter.
    It then runs the adapter until it's shut down by user request or error.

    Args:
    ----
        argv (list): A list containing the command-line arguments.
        Defaults to sys.argv if not provided.

    """
    rclpy.init(args=argv)

    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="door_adapter",
        description="Configure and spin up door adapter for door ")
    parser.add_argument(
        "-c", "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file for this door adapter"
    )
    parser.add_argument(
        '--enable_mqtt',
        type=str,
        default=False,
        help='Enable or disable the MQTT status polling (true/false)'
    )

    args = parser.parse_args(args_without_ros[1:])
    config_path = args.config_file

    print(f"Feature enabled: {args.enable_mqtt}")
    is_mqtt_enabled = False
    if args.enable_mqtt == "True":
        is_mqtt_enabled = True

    # Load config and nav graph yamls
    with open(config_path, "r", encoding="utf-8") as f:
        config_yaml = yaml.safe_load(f)

    door_adapter = DoorAdapter(config_yaml, is_mqtt_enabled)
    rclpy.spin(door_adapter)

    door_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
