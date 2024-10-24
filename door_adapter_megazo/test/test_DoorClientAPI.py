# Copyright 2015 Open Source Robotics Foundation, Inc.
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

# import sys
import yaml
import rclpy
import unittest
from rclpy.node import Node
from door_adapter_megazo.DoorClientAPI import DoorClientAPI


class TestROS2Node(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Set up a ROS 2 node once for all test cases."""
        rclpy.init()  # Initialize ROS 2
        cls.door_id = 'Lab_D01'
        cls.test_node = Node('test_door_adapter')
        config_path = "config.yaml"
        with open(config_path, "r", encoding="utf-8") as f:
            config_yaml = yaml.safe_load(f)
        cls.api = DoorClientAPI(cls.test_node, config_yaml, cls.test_node.get_logger())

    def test_DoorClientAPI_init(self):
        """Test if the node was created properly."""
        self.assertEqual(self.api.connected, True)
        self.assertEqual(self.api.debug, False)
        self.assertEqual(self.api.timeout, 5)

    def test_DoorClientAPI_get_token(self):
        """Test if the token was generated successfully."""
        self.assertEqual(self.api.get_token() is None, True)

    def test_DoorClientAPI_check_connection(self):
        """Test if the connection to Megazo server was successful."""
        self.assertEqual(self.api.check_connection() is None, False)

    # def test_DoorClientAPI_get_DeviceID_ProjectID(self):
    #     """Test if the ICED Device and Project ID were successfully retrieved."""
    #     device_id, project_id = self.api.get_iced_list_deviceID(self.door_id)
    #     project_id_2 = self.api.sign_in_project(project_id)

    #     self.assertEqual(device_id is None, True)
    #     self.assertEqual(project_id is None, True)
    #     self.assertEqual(project_id_2 is None, True)


if __name__ == '__main__':
    unittest.main()
