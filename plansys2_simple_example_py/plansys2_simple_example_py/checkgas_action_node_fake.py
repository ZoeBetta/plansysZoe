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

class CheckgasAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('checkgas', 0.5)
        self.progress_ = 0.0
        self.location = None


    def do_work(self):
        if self.progress_ < 0.3:
            self.progress_ += 0.05
            self.send_feedback(self.progress_, 'Checkgas running')
        else:
            #self.finish(True, 1.0, 'Search completed');
            self.progress_ = 0.0
            found_person = random() < 0.05
            #found_person = True

            self.get_logger().info('Checking gas:{}'.format(found_person))
            if (found_person):
              self.send_feedback(10.0, 'Emergency recognizedn')
              time.sleep(1)
              self.finish(True, 1.0, 'Finish check gas')
            else:
              self.send_feedback(60.0, 'Good bestemmia')
              time.sleep(1)
              self.finish(True, 1.0, 'Finish check gas')

        self.get_logger().info('checking gas ... {}'.format(self.progress_))


def main(args=None):
    rclpy.init(args=args)

    node = CheckgasAction()
    node.set_parameters([Parameter(name='action_name', value='checkgas')])

    node.trigger_configure()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
