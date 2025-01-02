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

MQTT_EVENTTYPE = {
            "3": "Door magnetic closure",
            "4": "Door magnet opens",
            "5": "Remote door opening event",
            "7": "The door is always open at the end of the time period",
            "8": "The door is always open at the beginning of the time period",
            "11": "Keypad door opening event",
            "14": "Door forced open alarm",
            "16": "Power down alarm event",
            "21": "Trigger door opening event",
            "22": "Password door opening event",
            "23": "Swipe card + password to open the door event",
            "1019": "Unlock the door",
            "1020": "Close the door lock",
            "1021": "Unlock the door once",
            "1022": "Locked door",
            "1023": "Door Reset"
        }
MQTT_EVENTRESULT = {
            "0": "NULL",
            "1": "Permission to pass",
            "2": "Illegal cards",
            "3": "Wrong time period",
            "4": "No Passage",
            "5": "Card capacity is full",
            "6": "Repeat card addition",
            "7": "Delete cards that don't exist in the first place",
            "8": "Password error",
            "9": "Work number does not exist",
            "10": "The work number exists, but the use of the password" +
                  " to open the door is prohibited",
            "11": "This door prohibits the use of the password to open the door",
            "12": "Violation of Antipassbac's Normal mode",
            "13": "Multi-card mode, timeout between swipes of two cards",
            "14": "Multi-card mode, whether the card is passable or not is undetermined.",
            "15": "Multi-card mode with more than 20 visitor swipes",
            "16": "Multi-card mode, employees swipe more than 20 cards",
            "17": "This door is in interlock, the door is not open",
            "19": "Repeated PIN's status",
            "24": "User card exceeds expiration date",
            "26": "Denied",
            "1000": "Success",
            "1001": "Failure",
            "1002": "Failure",
            "1003": "Success",
            "1004": "Main control board offline",
            "1005": "Trigger",
            "1006": "Recovery"
        }
