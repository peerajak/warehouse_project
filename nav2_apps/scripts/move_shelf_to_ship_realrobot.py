#! /usr/bin/env python3
# Real Robot

import time
from copy import deepcopy
import shutil
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
from custom_interfaces.srv import GoToLoading
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.task import Future
from rclpy.node import Node
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

nstates = ['ToPreload', 'AttachShelf', 'ToShipping', 'Rotating','EndProgramSuccess', 'EndProgramFailure']
nstate = nstates[0]
is_localized = False
is_odom_initialized = False
localize_threshold = [0.03 ,0.03,0.03]
initial_positions = [ 0,0,0,0 ]

realrobot_move_topic = '/cmd_vel'
simrobot_move_topic = '/diffbot_base_controller/cmd_vel_unstamped'

# Shelf positions for picking
shelf_positions = [ 3.5, -1.7,0.7938911945621069,-0.6080598417892362]
shelf_positions_reverse = [3.5, -1.7, -0.9792658753497765,-0.20257923233993155]
before_shipping = [1.72,-1.7,-0.9230358322434521,-0.3847139877813617]
before_shipping_reverse = [1.32,-1.7,-0.3929562957056833,0.9195571486673721]
shipping_destinations = [1.52,0.8,-0.9230358322434521,-0.3847139877813617]
shipping_destinations_reverse = [1.52,0.8,-0.3929562957056833,0.9195571486673721]
class LocalizeNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        my_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.service_client = self.create_client(
            srv_type=Empty,
            srv_name="/reinitialize_global_localization",
            callback_group=my_callback_group)
        self.amcl_sub = self.create_subscription(
                msg_type=PoseWithCovarianceStamped,
                topic='/amcl_pose',callback=self.amcl_listener_callback, callback_group=my_callback_group, qos_profile=1)
        timer1_period: float = 0.1 #reinitialize_global_localization service
        self.timer1 = self.create_timer(timer_period_sec=timer1_period,callback=self.timer1_callback,callback_group=my_callback_group)
        timer2_period: float = 0.05 #rotating
        self.rotating_counter = 0
        self.rotating_direction = 1
        self.angular_speed = 0.9
        self.timer2 = self.create_timer(timer_period_sec=timer2_period,callback=self.timer2_callback,callback_group=my_callback_group)
        self.publisher_rotate = self.create_publisher(
            msg_type=Twist,
            topic=realrobot_move_topic,qos_profile=1)
        self.future: Future = None

    def amcl_listener_callback(self, msg):
        global is_localized
        self.get_logger().info('Rotating %d,I heard: "%f,%f,%f"' %  
        (self.rotating_counter,msg.pose.covariance[0],msg.pose.covariance[7],
        msg.pose.covariance[-1]))
        self.rotating_counter += 1
        if self.rotating_counter >= 80:
            self.rotating_direction = -1*self.rotating_direction
            self.rotating_counter = 0
        if msg.pose.covariance[0] <localize_threshold[0] and \
           msg.pose.covariance[7] <localize_threshold[1] and \
           msg.pose.covariance[-1]<localize_threshold[2]:
            is_localized = True
        

    def timer2_callback(self): 
        self.get_logger().info('timer2_callback keep rotating')
        ling = Twist()
        ling.angular.z = self.rotating_direction * self.angular_speed
        self.publisher_rotate.publish(ling)         


    def timer1_callback(self): 
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client.srv_name} not available, waiting...') 
        self.get_logger().info('timer1_callback reinitialize_global_localization service')
        request = Empty.Request()
        self.future = self.service_client.call_async(request)
        self.future.add_done_callback(self.response1_callback)
        self.timer1.cancel()

    def response1_callback(self, future: Future):
        global nstate
        response = future.result()
        if response is not None:
            self.get_logger().info("Some Response happened: Success!")
        print('response from reinitialize_global_localization service')


class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscriber_odom = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))


    def odom_callback(self, msg2):
        global initial_positions 
        global is_odom_initialized
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (initial_positions[0], initial_positions[1], posz) = (position.x, position.y, position.z)
        (qx, qy, initial_positions[2], initial_positions[3]) = (orientation.x, orientation.y, orientation.z, orientation.w)
        is_odom_initialized = True



class ServiceClient(Node):
  def __init__(self,navigator):
    super().__init__('service_client')
    self.get_logger().info('init service_client')
    self.navigator = navigator
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
    self.service_client4 = self.create_client(
            srv_type=SetParameters,
            srv_name="/global_costmap/global_costmap/set_parameters",
            callback_group=my_callback_group)
    self.service_client5 = self.create_client(
            srv_type=SetParameters,
            srv_name="/local_costmap/local_costmap/set_parameters",
            callback_group=my_callback_group)

    self.future: Future = None
    self.future2: Future = None
    self.future3: Future = None
    self.future4: Future = None
    self.future5: Future = None
    self.final_approach = True

    self.robot_radius = 0.15
    self.robot_footprint = '[ [0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15] ]'
    self.robot_with_cart_radius = 0.25
    self.robot_with_cart_footprint = '[ [0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25] ]'
    self.cart_inflation_radius = 0.2    

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
    self.get_logger().info('timer2_callback lifecycle_service_client')
    while not self.service_client2.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {self.service_client2.srv_name} not available, waiting...')
    request = SetParameters.Request()
    request.parameters = [Parameter(name= 'robot_radius',  
            value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, 
                    double_value= self.robot_with_cart_radius)), Parameter(name= 'inflation_radius',  
            value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, 
                    double_value= self.cart_inflation_radius))  ]#0.7071068
    self.future2 = self.service_client2.call_async(request)
    self.future2.add_done_callback(self.response2_callback)
    self.timer2.cancel()

  def timer3_callback(self):  
    self.get_logger().info('timer3_callback lifecycle_service_client')
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
        self.get_logger().info("Some Response happened")
        if(response.complete):
            self.get_logger().info("response from service server: Success!")
            msgs_empty = String()
            self.publisher_lift.publish(msgs_empty)
            nstate = nstates[2]
            self.get_logger().info('Lifted product from ')
            just_before_shipping = PoseStamped()
            just_before_shipping.header.frame_id = 'map'
            just_before_shipping.header.stamp = self.navigator.get_clock().now().to_msg()
            just_before_shipping.pose.position.x = before_shipping[0]
            just_before_shipping.pose.position.y = before_shipping[1]
            just_before_shipping.pose.orientation.z = before_shipping[2]
            just_before_shipping.pose.orientation.w = before_shipping[3]
            self.navigator.waitUntilNav2Active()
            self.navigator.goToPose(just_before_shipping)
            while not self.navigator.isTaskComplete():
                self.get_logger().info('waiting nav to complete')
                pass
            print('just before complete!')
            timer_period: float = 1.0
            self.timer2 = self.create_timer(timer_period_sec=timer_period,callback=self.timer2_callback)
        else:
            self.get_logger().info("response from service server: Failed!")
            nstate = nstates[5]

    else:
        self.get_logger().info("The response is None")
  def response2_callback(self, future: Future):
    global nstate
    response = future.result()
    if response is not None:
        self.get_logger().info("Some Response2 happened")
        if(response.results[0].successful):
            self.get_logger().info("response from lifecycle service server: Success!"+response.results[0].reason)
            nstate = nstates[2]
            timer_period: float = 1.0
            self.timer3 = self.create_timer(timer_period_sec=timer_period,callback=self.timer3_callback)
        else:
            self.get_logger().info("response from lifecycle service server: Failed!")
            nstate = nstates[5]
    else:
        self.get_logger().info("The response is None")

  def response4_callback(self, future: Future):
    global nstate
    response = future.result()
    if response is not None:
        self.get_logger().info("Some Response4 happened")
        if(response.results[0].successful):
            self.get_logger().info("response4 from lifecycle service server: Success!"+response.results[0].reason)
            self.is_parameter4_complete = True
        else:
            self.get_logger().info("response4 from lifecycle service server: Failed!")
            self.is_parameter4_complete = True
            nstate = nstates[5]
    else:
        self.get_logger().info("The response is None")
        self.is_parameter4_complete = True

  def response3_callback(self, future: Future):
    global nstate
    response = future.result()
    if response is not None:
        self.get_logger().info("Some Response3 happened %d" % (response.results[0].successful))
        # if(response.results[0].successful):
        #     self.get_logger().info("response from lifecycle service server: Success!"+response.results[0].reason)
            # self.navigator.waitUntilNav2Active()

        # result = self.navigator.getResult()
        #     if result == TaskResult.SUCCEEDED:
        print('Goto shipping destination')
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = self.navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations[0]
        shipping_destination.pose.position.y = shipping_destinations[1]
        shipping_destination.pose.orientation.z = shipping_destinations[2]
        shipping_destination.pose.orientation.w = shipping_destinations[3]
        self.navigator.goToPose(shipping_destination)
        while not self.navigator.isTaskComplete():
            pass
        # result1 = self.navigator.getResult()
        #         if result1 == TaskResult.SUCCEEDED:
        print('Arrived shipping destination (' + self.request_destination + ')...')
        msgs_empty = String()
        self.publisher_liftdown.publish(msgs_empty)
        print('Unloaded complete! Goto initial position')
        timer_period: float = 1.0
        self.timer4 = self.create_timer(timer_period_sec=timer_period,callback=self.timer4_callback)
        self.timer5 = self.create_timer(timer_period_sec=timer_period,callback=self.timer5_callback)
        self.is_parameter4_complete = False
        self.is_parameter5_complete = False
        while (not self.is_parameter4_complete) or (not self.is_parameter5_complete):
            time.sleep(0.1) 
        print('Waiting or footprint and robot_radius reset')
        nstate = nstates[3]
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = self.navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations_reverse[0]
        shipping_destination.pose.position.y = shipping_destinations_reverse[1]
        shipping_destination.pose.orientation.z = shipping_destinations_reverse[3]
        shipping_destination.pose.orientation.w = shipping_destinations_reverse[4]
        self.navigator.goToPose(shipping_destination)
        while not self.navigator.isTaskComplete():
            pass
        result2 = self.navigator.getResult()
        if result2 == TaskResult.SUCCEEDED:
            print('Task at rotate back shipping Success!')
        #self.navigator.waitUntilNav2Active()
        print('Going back to just before shipping_position')
        just_before_shipping = PoseStamped()
        just_before_shipping.header.frame_id = 'map'
        just_before_shipping.header.stamp = self.navigator.get_clock().now().to_msg()
        just_before_shipping.pose.position.x = before_shipping_reverse[0]
        just_before_shipping.pose.position.y = before_shipping_reverse[1]
        just_before_shipping.pose.orientation.z = before_shipping_reverse[2]
        just_before_shipping.pose.orientation.w = before_shipping_reverse[3]
        self.navigator.goToPose(just_before_shipping)
        while not self.navigator.isTaskComplete():
                pass
                    # result3 = self.navigator.getResult()
                    # if result3 == TaskResult.SUCCEEDED:
        print('Going back to initial_position')
        initial_position = PoseStamped()
        initial_position.header.frame_id = 'map'
        initial_position.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_position.pose.position.x = initial_positions["initial_position"][0]
        initial_position.pose.position.y = initial_positions["initial_position"][1]
        initial_position.pose.orientation.z = initial_positions["initial_position"][2]
        initial_position.pose.orientation.w = initial_positions["initial_position"][3]
        self.navigator.goToPose(initial_position)
        while not self.navigator.isTaskComplete():
            pass
            #             result4 = self.navigator.getResult()
            #             if result4 == TaskResult.SUCCEEDED:
            #                 print('Successfully reached initial position. Exit Program')
            #                 nstate = nstates[4] 
            #                 exit(0)
            #             elif result4 == TaskResult.CANCELED:
            #                 print('Security route was canceled, exiting.')
            #                 exit(1)
            #             elif result4 == TaskResult.FAILED:
            #                 print('Security route failed! Restarting from the other side...')
            #             else:
            #                 self.get_logger().info("response from lifecycle service server: Failed!")
            #                 nstate = nstates[5]  
            #         else:                                     
            #             print('Task at return just_before_shipping failed!')
            #             nstate = nstates[4]
            #             exit(-1)
            #     elif result1 == TaskResult.CANCELED:
            #         print('Task at ' + self.request_item_location +
            #             ' was canceled. Returning to staging point...')
            #     elif result1 == TaskResult.FAILED:
            #         print('Task at ' + self.request_item_location + ' failed!')
            #         exit(-1)
            #     while not self.navigator.isTaskComplete():
            #         pass
            # elif result == TaskResult.CANCELED:
            #     print('Security route was canceled, exiting.')
            #     exit(1)
            # elif result == TaskResult.FAILED:
            #     print('Security route failed! Restarting from the other side...')
            # else:
            #     self.get_logger().info("response from lifecycle service server: Failed!")
            #     nstate = nstates[5]           
    else:
        self.get_logger().info("The response is None")
    #print('leaving service node') 
    #rclpy.shutdown()
 
  def response5_callback(self, future: Future):
    global nstate
    response = future.result()
    if response is not None:
        self.get_logger().info("Some Response5 happened")
        if(response.results[0].successful):
            self.get_logger().info("response5 from lifecycle service server: Success!"+response.results[0].reason)  
            self.is_parameter5_complete = True       
    else:
        self.get_logger().info("The response5 is None")
        self.is_parameter5_complete = True
 
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

    odom_subscriber = OdomSubscriber()
    while not is_odom_initialized:
        rclpy.spin_once(odom_subscriber)
        print('waiting odom data..')
        time.sleep(0.1)
    print(initial_positions)
    odom_subscriber.destroy_node()

    localize_node = LocalizeNode()
    while not is_localized:
        rclpy.spin_once(localize_node)
        print('localizing..')
        time.sleep(0.05)
    localize_node.destroy_node()


    navigator = BasicNavigator()
    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x =initial_positions[0]
    initial_pose.pose.position.y = initial_positions[1]
    initial_pose.pose.orientation.z = initial_positions[2]
    initial_pose.pose.orientation.w = initial_positions[3]
    print(initial_positions)
    navigator.goToPose(initial_pose) 


    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x =shelf_positions[0]
    shelf_item_pose.pose.position.y =shelf_positions[1]
    shelf_item_pose.pose.orientation.z =shelf_positions[2]
    shelf_item_pose.pose.orientation.w =shelf_positions[3]
    #print('Received request for item picking at ' + request_item_location + '.')
    print(initial_positions)
    navigator.goToPose(shelf_item_pose)

    # # Do something during your route
    # # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Success')
        nstate = nstates[1]

    elif result == TaskResult.CANCELED:
        print('Task at  was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        nstate = nstates[4]

    elif result == TaskResult.FAILED:
        print('Task at  failed!')
        nstate = nstates[4]

    while not navigator.isTaskComplete():
        pass

    executor = rclpy.executors.MultiThreadedExecutor()
    service_client_node = ServiceClient(navigator)    
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



