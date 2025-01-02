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
import unittest
from rmf_door_msgs.msg import DoorMode
from door_adapter_megazo.door_adapter import Door
from door_adapter_megazo.door_adapter import DoorAdapter


class TestROS2Node(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Set up a ROS 2 node once for all test cases."""
        cls.test_door = Door('Lab_D01',
                             False,
                             3.0,
                             False,
                             False)

        cls.test_door_status = Door('Lab_D01',
                                    False,
                                    3.0,
                                    True,
                                    False)

        config_path = "config.yaml"
        with open(config_path, "r", encoding="utf-8") as f:
            config_yaml = yaml.safe_load(f)
        cls.door_adapter = DoorAdapter(config_yaml, False)

    def test_Door_init(self):
        """Test if the node was created properly."""
        self.assertEqual(self.test_door.id, 'Lab_D01')
        self.assertEqual(self.test_door.door_mode, DoorMode.MODE_CLOSED)
        self.assertEqual(self.test_door.open_door, False)
        self.assertEqual(self.test_door.check_status is None, True)
        self.assertEqual(self.test_door.door_auto_closes, False)
        self.assertEqual(self.test_door.door_signal_period, 3.0)

    def test_Door_init_continuous_status_polling(self):
        """Test if the node was created properly."""
        self.assertEqual(self.test_door_status.id, 'Lab_D01')
        self.assertEqual(self.test_door_status.door_mode, DoorMode.MODE_CLOSED)
        self.assertEqual(self.test_door_status.open_door, False)
        self.assertEqual(self.test_door_status.check_status, False)
        self.assertEqual(self.test_door_status.door_auto_closes, False)
        self.assertEqual(self.test_door_status.door_signal_period, 3.0)

    def test_DoorAdapter_init(self):
        """Test if the node was created properly."""
        self.assertEqual(self.door_adapter.door_state_publish_period is None, False)
        self.assertEqual(self.door_adapter.mock_adapter, False)
        self.assertEqual(self.door_adapter.door_states_pub is None, False)
        self.assertEqual(self.door_adapter.door_request_sub is None, False)
        self.assertEqual(self.door_adapter.periodic_timer is None, False)


if __name__ == '__main__':
    unittest.main()
