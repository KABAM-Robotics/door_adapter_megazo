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

"""Module that authenticates, generates API payloads and sends to Megazo API REST points."""

import time
import json
from urllib.error import HTTPError
from datetime import datetime, timedelta
import requests
from rmf_door_msgs.msg import DoorMode


class DoorClientAPI:
    """A Module that handles communication with Megazo API."""

    def __init__(self, node, config, ros_logger):
        self.timeout = 5  # seconds
        self.debug = False
        self.connected = False
        self.node = node
        self.config = config  # use this config to establish connection
        self.token = None
        self.ros_logger = ros_logger

        count = 0
        self.connected = False
        while not self.check_connection():
            if count >= self.timeout:
                self.ros_logger.info("Unable to connect to door"
                                     " client API.")
                self.connected = False
                break
            self.ros_logger.warn("Unable to connect to door client API. "
                                 "Attempting to reconnect...")
            count += 1
            time.sleep(1)
        self.ros_logger.info("Connected to door client API.")
        self.connected = True

    def get_token(self):
        """
        Retrieve an API token for authentication.

        This method sends a POST request to the configured API endpoint
        with credentials from config and returns the received token if it
        was previously requested.
        If not, ignore this message. Otherwise return the previous token data
        """
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
            self.ros_logger.warn(f'Connection error: {connection_error}')
            return None
        except HTTPError as http_err:
            self.ros_logger.warn(f'HTTP error: {http_err}')
            return None
        return None

    def check_connection(self):
        """Return True if connection to the door API server is successful."""
        self.token = self.get_token()
        return bool(self.token)

    def sign_in_project(self, projectID: str):
        """
        Signs in a user to an API endpoint for Megazo API.

        This method sends a POST request with authentication credentials and
        project ID to the configured API endpoint. If successful, it returns the
        received token data.
        """
        path = self.config["api_endpoint"] + "/API/System/ProjectSignIn"

        payload = {
            'UserID': self.config["header_key"],
            'Token': self.token,
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

    def get_iced_list_deviceID(self, door_id: str):
        """
        Retrieve device ID and project ID from ICED API based on provided door ID.

        Args
        ----
            door_id (str): The ID of the door for which to retrieve device information.

        Returns
        -------
            tuple: A tuple containing the device ID and project ID if found, otherwise None.
            If an error occurs during API request or data parsing, returns None.

        Raises
        ------
            requests.exceptions.ConnectionError: If a connection issue arises while making
            the HTTP POST request.
            HTTPError: If there's an HTTP-related error (e.g., 404 Not Found) when sending
            the request.

        Note
        ----
            This method assumes that the ICED API endpoint and authentication credentials
            are properly configured in the class instance. It also expects the 'data' key
            within the JSON response to contain a list of devices, where each device has an
            'ICEDName' attribute matching the provided door ID.

        """
        path = self.config["api_endpoint"] + "/API/ICED/GetICEDList"

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
                        return device['ID'], device['ProjectID']
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

    def open_door(self, door_id):
        """Return True if the door API server is successful receive open door command."""
        device_id, project_id = self.get_iced_list_deviceID(door_id)

        if device_id is None or project_id is None:
            self.ros_logger.error("Unable to retrieve Device/Project ID. Returning False")
            return False

        project_id_2 = self.sign_in_project(project_id)

        if project_id_2 is None:
            self.ros_logger.error("Unable to sign into Project. Returning False")
            return False

        path = self.config["api_endpoint"] + "/API/Device/ICED/ControlDoor"

        NORMAL_OPEN = 1

        payload = {
            'UserID': self.config["header_key"],
            'Token': self.token,
            'data': {
                'IDs': [device_id],
                'Operate': NORMAL_OPEN
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
                self.ros_logger.info("OPEN DOOR Request Acknowledgement - [SUCCESS]")
                return True
            else:
                self.ros_logger.warn("OPEN DOOR Request Acknowledgement - [FAIL]")
                self.ros_logger.warn(f"Error: {data['errMsg']}")
                return False

        except requests.exceptions.ConnectionError as connection_error:
            self.ros_logger.warn(f'Connection error: {connection_error}')
            return False
        except HTTPError as http_err:
            self.ros_logger.warn(f'HTTP error: {http_err}')
            return False

    def close_door(self, door_id):
        """Return True if the door API server is successful receive open door command."""
        device_id, project_id = self.get_iced_list_deviceID(door_id)

        if device_id is None or project_id is None:
            self.ros_logger.error("Unable to retrieve Device/Project ID. Returning False")
            return False

        project_id_2 = self.sign_in_project(project_id)

        if project_id_2 is None:
            self.ros_logger.error("Unable to sign into Project. Returning False")
            return False

        path = self.config["api_endpoint"] + "/API/Device/ICED/ControlDoor"

        FORCED_SHUTDOWN = 4

        payload = {
            'UserID': self.config["header_key"],
            'Token': self.token,
            'data': {
                'IDs': [device_id],
                'Operate': FORCED_SHUTDOWN
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
                self.ros_logger.info("CLOSE DOOR Request Acknowledgement - [SUCCESS]")
                return True
            else:
                self.ros_logger.warn("CLOSE DOOR Request Acknowledgement - [FAIL]")
                self.ros_logger.warn(f"Error: {data['errMsg']}")
                return False

        except requests.exceptions.ConnectionError as connection_error:
            self.ros_logger.warn(f'Connection error: {connection_error}')
            return False
        except HTTPError as http_err:
            self.ros_logger.warn(f'HTTP error: {http_err}')
            return False

    def get_mode(self, door_id):
        """

        Return the door status with reference rmf_door_msgs.

        Return DoorMode.MODE_CLOSED when door status is closed.
        Return DoorMode.MODE_MOVING when door status is moving.
        Return DoorMode.MODE_OPEN when door status is open.
        Return DoorMode.MODE_OFFLINE when door status is offline.
        Return DoorMode.MODE_UNKNOWN when door status is unknown.
        """
        path = self.config["api_endpoint"] + "/API/ICED/GetICEDList"

        payload = {
            'UserID': self.config['header_key'],
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
                        # Check if door is online recently.
                        last_heartbeat_timestamp = datetime.strptime(
                            data['data'][0]['LastHeartbeatTime'],
                            "%Y-%m-%d %H:%M:%S")
                        current_time = datetime.now()

                        if last_heartbeat_timestamp == '':
                            self.ros_logger.warn(f"Door [{door_id}] HeartBeat null."
                                                 " [MODE_UNKNOWN]")
                            return DoorMode.MODE_UNKNOWN

                        time_difference = current_time - last_heartbeat_timestamp

                        # Check if the difference is more than 15 minutes
                        if time_difference > timedelta(minutes=15):
                            self.ros_logger.warn(f"Door [{door_id}] HeartBeat"
                                                 " expired. [MODE_OFFLINE]")
                            return DoorMode.MODE_OFFLINE
                        else:
                            if data['data'][0]['DoorOpenStatus'] == 0:
                                self.ros_logger.info(f"Door [{door_id}] [MODE_CLOSED]")
                                return DoorMode.MODE_CLOSED
                            else:
                                self.ros_logger.info(f"Door [{door_id}] [MODE_OPENED]")
                                return DoorMode.MODE_OPEN
                self.ros_logger.warn(f"\n[WARN] Unable to find ICEDName for door {door_id}."
                                     " Please ensure config.yaml contains the correct door name."
                                     " - [MODE UNKNOWN]")
            else:
                self.ros_logger.error(f"\nUnable to get Door [{door_id}] status - [MODE_UNKNOWN]")
        except requests.exceptions.HTTPError as http_error:
            self.ros_logger.warn(f'HTTP error: {http_error} - Renewing token...')
            self.token = self.get_token()
        except requests.exceptions.ConnectionError as connection_error:
            self.ros_logger.warn(f'Connection error: {connection_error} - [MODE_UNKNOWN]')
        except HTTPError as http_err:
            self.ros_logger.warn(f'HTTP error: {http_err} - [MODE_UNKNOWN]')

        return DoorMode.MODE_UNKNOWN
