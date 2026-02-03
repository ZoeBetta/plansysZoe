#!/usr/bin/python3

# Copyright 2023 Intelligent Robotics Lab
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

import rclpy
from rclpy.parameter import Parameter, ParameterType

from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from random import random
from plansys2_msgs.msg import ActionExecution
from rclpy.node import Node
from plansys2_msgs.srv import AffectNode, AffectParam, GetProblem, GetDomain
from plansys2_msgs.msg import Node as NodeMsg
from plansys2_msgs.msg import Param
from threading import Event
import time

class CheckstairsAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('checkstairs', 0.5)
        self.progress_ = 0.0
        self.location = None
        self.stair = None
        self.subscription = self.create_subscription(
            ActionExecution,
            '/actions_hub', self.listener_callback, 10)

    def listener_callback(self, msg):
        parameters = msg.arguments
        action_name = msg.action
#        self.get_logger().info(f'Action received with parameters {parameters}')

        # Here, perform any action-specific handling based on action_name and parameters
        if action_name == 'checkstairs':
            if len(parameters) == 3:  # Expected 'move <robot> <location> <stair>'
                self.location = parameters[2]
                self.stair = parameters[1]
#                self.get_logger().info(f'Robot in {self.location}')


    def do_work(self):
        if self.progress_ < 0.3:
            self.progress_ += 0.05
            self.send_feedback(self.progress_, 'Checkstairs running')
        else:
            #self.finish(True, 1.0, 'Search completed');
            self.progress_ = 0.0
            found_person = random() < 0.05
            found_person = True

            self.get_logger().info('Checking stairs:{}'.format(found_person))
            if (found_person):
              self.send_feedback(10.0, 'EmergencyStair ' + self.location + ' ' + self.stair)
              time.sleep(1)
              self.finish(True, 1.0, 'Finish check stairs')
            else:
              self.send_feedback(60.0, 'No emergency recognized')
              time.sleep(1)
              self.finish(True, 1.0, 'Finish check stairs')

        self.get_logger().info('checking stairs ... {}'.format(self.progress_))


def main(args=None):
    rclpy.init(args=args)

    node = CheckstairsAction()
    node.set_parameters([Parameter(name='action_name', value='checkstairs')])

    node.trigger_configure()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
