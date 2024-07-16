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
import shutil
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from custom_interfaces.srv import GoToLoading
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from rclpy.duration import Duration
from rclpy.task import Future
from rclpy.node import Node
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

nstates = ['ToPreload', 'AttachShelf', 'ToShipping','EndProgramSuccess', 'EndProgramFailure']
nstate = nstates[0]

# Shelf positions for picking
shelf_positions = {
    "shelf_1": [5.779634686956881, 0.07032379500580269,-0.6874334536853087,0.726247372975826]}

# Shipping destination for picked products
shipping_destinations = {
    "shipping": [2.5069296916199484, 1.3907159489075767,0.756143318789467,0.6544060524246781],
}

security_route_return_journey = [
        [5.557730437379715,  -1.1],
        [5.557730437379715,  -0.9],
        [5.557730437379715,  -0.5],
        [4.372317535720752,  -0.01],
        [3.92317535720752,   -0.01],
        #[5.557730437379715,  -0.3],
        #[5.557730437379715,  -0.21633228096287413],
        [2.5069296916199484, 1.3907159489075767]
        ]
'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''
class ServiceClient(Node):
  def __init__(self,navigator,request_item_location,request_destination):
    super().__init__('service_client')
    self.get_logger().info('init service_client')
    self.navigator = navigator
    self.request_item_location = request_item_location
    self.request_destination = request_destination
    my_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    self.service_client = self.create_client(
            srv_type=GoToLoading,
            srv_name="/approach_shelf",
            callback_group=my_callback_group)
    self.service_client2 = self.create_client(
            srv_type=SetParameters,
            srv_name="/global_costmap/global_costmap/set_parameters",
            callback_group=my_callback_group)
    self.service_client3 = self.create_client(
            srv_type=SetParameters,
            srv_name="/local_costmap/local_costmap/set_parameters",
            callback_group=my_callback_group)

    self.future: Future = None
    self.future2: Future = None
    self.future3: Future = None
    self.final_approach = True

    self.robot_with_cart_radius = 0.28
    self.robot_with_cart_footprint = '[ [0.5, 0.28], [0.5, -0.28], [-0.5, -0.28], [-0.5, 0.28] ]'

    timer_period: float = 1.0
    self.timer = self.create_timer(timer_period_sec=timer_period,callback=self.timer_callback)
    self.publisher_lift = self.create_publisher(
            msg_type=String,
            topic='/elevator_up',
            qos_profile=1)
    self.publisher_liftdown = self.create_publisher(
            msg_type=String,
            topic='/elevator_down',
            qos_profile=1)

  def timer_callback(self): 
    self.get_logger().info('timer_callback service_client') 
    while not self.service_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {self.service_client.srv_name} not available, waiting...')
    request = GoToLoading.Request()
    request.attach_to_shelf = self.final_approach
    self.future = self.service_client.call_async(request)
    self.future.add_done_callback(self.response_callback)
    self.timer.cancel()

  def timer2_callback(self):  
    self.get_logger().info('timer_callback lifecycle_service_client')
    while not self.service_client2.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {self.service_client2.srv_name} not available, waiting...')
    request = SetParameters.Request()
    request.parameters = [Parameter(name= 'robot_radius',  
            value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, 
                    double_value= self.robot_with_cart_radius))  ]#0.7071068
    self.future2 = self.service_client2.call_async(request)
    self.future2.add_done_callback(self.response2_callback)
    self.timer2.cancel()

  def timer3_callback(self):  
    self.get_logger().info('timer_callback lifecycle_service_client')
    while not self.service_client3.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {self.service_client3.srv_name} not available, waiting...')
    request = SetParameters.Request()
    request.parameters = [Parameter(name= 'footprint',  
            value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, 
                    string_value= self.robot_with_cart_footprint))]
    self.future3 = self.service_client3.call_async(request)
    self.future3.add_done_callback(self.response3_callback)
    self.timer3.cancel()

  def response_callback(self, future: Future):
    global nstate
    response = future.result()
    if response is not None:
        #self.get_logger().info("Some Response happened")
        if(response.complete):
            self.get_logger().info("response from service server: Success!")
            msgs_empty = String()
            self.publisher_lift.publish(msgs_empty)
            nstate = nstates[2]
            timer_period: float = 1.0
            self.timer2 = self.create_timer(timer_period_sec=timer_period,callback=self.timer2_callback)
        else:
            self.get_logger().info("response from service server: Failed!")
            nstate = nstates[4]

    else:
        self.get_logger().info("The response is None")

  def response2_callback(self, future: Future):
    global nstate
    response = future.result()
    if response is not None:
        #self.get_logger().info("Some Response happened")
        if(response.results[0].successful):
            self.get_logger().info("response from lifecycle service server: Success!"+response.results[0].reason)
            nstate = nstates[2]
            timer_period: float = 1.0
            self.timer3 = self.create_timer(timer_period_sec=timer_period,callback=self.timer3_callback)
        else:
            self.get_logger().info("response from lifecycle service server: Failed!")
            nstate = nstates[4]
    else:
        self.get_logger().info("The response is None")

  def response3_callback(self, future: Future):
    global nstate
    response = future.result()
    if response is not None:
        #self.get_logger().info("Some Response happened")
        if(response.results[0].successful):
            self.get_logger().info("response from lifecycle service server: Success!"+response.results[0].reason)
            nstate = nstates[3]
            self.navigator.waitUntilNav2Active()
            print('Got product from ' + self.request_item_location +
                '! Bringing product to shipping destination (' + self.request_destination + ')...')
            route_poses = []
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.orientation.w = 1.0
            for pt in security_route_return_journey:
                pose.pose.position.x = pt[0]
                pose.pose.position.y = pt[1]
                route_poses.append(deepcopy(pose))
            self.navigator.goThroughPoses(route_poses)
                   # Do something during your route (e.x. AI detection on camera images for anomalies)
        # Print ETA for the demonstration
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

                # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                    print('Navigation has exceeded timeout of 180s, canceling the request.')
                    self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Route complete! Goto shipping destination')
            shipping_destination = PoseStamped()
            shipping_destination.header.frame_id = 'map'
            shipping_destination.header.stamp = self.navigator.get_clock().now().to_msg()
            shipping_destination.pose.position.x = shipping_destinations[self.request_destination][0]
            shipping_destination.pose.position.y = shipping_destinations[self.request_destination][1]
            shipping_destination.pose.orientation.z = shipping_destinations[self.request_destination][2]
            shipping_destination.pose.orientation.w = shipping_destinations[self.request_destination][3]
            self.navigator.goToPose(shipping_destination)
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Arrived shipping destination (' + self.request_destination + ')...')
                msgs_empty = String()
                self.publisher_liftdown.publish(msgs_empty)

            elif result == TaskResult.CANCELED:
                print('Task at ' + request_item_location +
                    ' was canceled. Returning to staging point...')
            elif result == TaskResult.FAILED:
                print('Task at ' + request_item_location + ' failed!')
                exit(-1)

            while not navigator.isTaskComplete():
                pass
        elif result == TaskResult.CANCELED:
            print('Security route was canceled, exiting.')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Security route failed! Restarting from the other side...')

        else:
            self.get_logger().info("response from lifecycle service server: Failed!")
            nstate = nstates[4]            
    else:
        self.get_logger().info("The response is None")
    #print('leaving service node') 
    #rclpy.shutdown()
 


def main():
    global nstate
    rclpy.init()
    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    request_item_location = 'shelf_1'
    request_destination = 'shipping'
    ####################



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
        nstate = nstates[1]

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        nstate = nstates[4]

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        nstate = nstates[4]



    while not navigator.isTaskComplete():
        pass

    executor = rclpy.executors.MultiThreadedExecutor()
    service_client_node = ServiceClient(navigator,request_item_location,request_destination)    
    executor.add_node(service_client_node )
    while rclpy.ok and not(nstate == nstates[3] or nstate == nstates[4]):
        executor.spin_once()
        print(nstate)
    
    










if __name__ == '__main__':

    main()

    exit(0)
    # TODO add new lifecycle_service client
    # - remove service_client_node
    # - create lifecycle_service_client class
    #    - pause lifecycle manager of path planner
    #    - swap config files
    #    - restart lifecycle manager of path planner with new config files
    # - add lifecycle_service_client instance
    ####



