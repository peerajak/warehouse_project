#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from custom_interfaces.srv import GoToLoading
from rclpy.duration import Duration
from rclpy.task import Future
from rclpy.node import Node
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Shelf positions for picking
shelf_positions = {
    "shelf_1": [5.729634686956881, 0.07032379500580269,-0.6874334536853087,0.726247372975826]}

# Shipping destination for picked products
shipping_destinations = {
    "shipping": [2.5069296916199484, 1.3907159489075767,0.756143318789467,0.6544060524246781],
}

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''
class ServiceClient(Node):
  def __init__(self):
    super().__init__('service_client')
    my_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
    self.service_client = self.create_client(
            srv_type=GoToLoading,
            srv_name="/approach_shelf",
            callback_group=my_callback_group)
    self.future: Future = None
    self.final_approach = False

    timer_period: float = 1.0
    self.timer = self.create_timer(timer_period_sec=timer_period,callback=self.timer_callback)
    self.publisher_lift = self.create_publisher(
            msg_type=Empty,
            topic='/elevator_up',
            qos_profile=1)

  def timer_callback(self):  
    while not self.service_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {self.service_client.srv_name} not available, waiting...')
    request = GoToLoading.Request()
    request.attach_to_shelf = self.final_approach
    self.future = self.service_client.call_async(request)
    self.future.add_done_callback(self.response_callback)
    self.timer.cancel()

  def response_callback(self, future: Future):
    response = future.result()
    if response is not None:
        #self.get_logger().info("Some Response happened")
        if(response.complete):
            self.get_logger().info("response from service server: Success!")
            msgs_empty = Empty()
            self.publisher_lift.publish(msgs_empty)
        else:
            self.get_logger().info("response from service server: Failed!")
    else:
        self.get_logger().info("The response is None")





def main():
    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    request_item_location = 'shelf_1'
    request_destination = 'shipping'
    ####################

    rclpy.init()

    navigator = BasicNavigator()

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.0043595783091967205
    initial_pose.pose.position.y = float(-8.493102018902478e-06)
    initial_pose.pose.orientation.z = 0.0020702609325826864
    initial_pose.pose.orientation.w = 0.9999978570075393
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Success')

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    executor = rclpy.executors.Executor()    
    service_client_node = ServiceClient()
    executor.add_node(service_client_node)



    rclpy.spin(service_client_node)

    exit(0)


if __name__ == '__main__':
    main()