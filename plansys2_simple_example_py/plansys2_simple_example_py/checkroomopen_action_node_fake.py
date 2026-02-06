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

class CheckroomopenAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('checkroomopen', 0.5)
        self.progress_ = 0.0
        self.locfrom = None
        self.locto = None
        self.subscription = self.create_subscription(
            ActionExecution,
            '/actions_hub', self.listener_callback, 10)

    def listener_callback(self, msg):
        parameters = msg.arguments
        action_name = msg.action
#        self.get_logger().info(f'Action received with parameters {parameters}')

        # Here, perform any action-specific handling based on action_name and parameters
        if action_name == 'checkroomopen':
            if len(parameters) == 2:  # Expected 'checkroomopen <robot> <location>'
                self.loc = parameters[1]
#                self.get_logger().info(f'Robot in {self.location}')


    def do_work(self):
        if self.progress_ < 0.3:
            self.progress_ += 0.05
            self.send_feedback(self.progress_, 'Checkroomopen running')
        else:
            #self.finish(True, 1.0, 'Search completed');
            self.progress_ = 0.0
            found_person = random() < 0.05
            #found_person = True

            self.get_logger().info('Checking room open from {} to {}'.format(self.locfrom, self.locto))
            if (found_person):
              self.send_feedback(10.0, 'EmergencyRoom ' + self.loc)
              time.sleep(1)
              self.finish(True, 1.0, 'Finish check room')
            else:
              self.send_feedback(60.0, 'No emergency recognized')
              time.sleep(1)
              self.finish(True, 1.0, 'Finish check room')

        self.get_logger().info('checking room ... {}'.format(self.progress_))


def main(args=None):
    rclpy.init(args=args)

    node = CheckroomopenAction()
    node.set_parameters([Parameter(name='action_name', value='checkroomopen')])

    node.trigger_configure()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
