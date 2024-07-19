#! /usr/bin/env python3
# Real Robot

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
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

nstates = ['ToPreload', 'AttachShelf', 'ToShipping', 'Rotating','EndProgramSuccess', 'EndProgramFailure']
nstate = nstates[0]
pose_estimation_initial_position = [-0.722, -0.833, -0.275]
is_odom_initialized = False
initial_positions = [ 0,0,0,0 ]
# Shelf positions for picking
shelf_positions = [0,0,0.7938911945621069,-0.6080598417892362]
   

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
        global shelf_positions
        global is_odom_initialized
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (initial_positions[0], initial_positions[1], posz) = (position.x, position.y, position.z)
        shelf_positions[0] = initial_positions[0] + 3.5
        shelf_positions[1] = initial_positions[1] -0.38795086992968963
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
            timer_period: float = 1.0
            #self.timer2 = self.create_timer(timer_period_sec=timer_period,callback=self.timer2_callback)
        else:
            self.get_logger().info("response from service server: Failed!")
            nstate = nstates[5]

    else:
        self.get_logger().info("The response is None")

 
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
    
    print('got odom data at ', initial_positions)

    odom_subscriber.destroy_node()
    navigator = BasicNavigator()
    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = pose_estimation_initial_position[0]
    initial_pose.pose.position.y = pose_estimation_initial_position[1]
    initial_pose.pose.orientation.z = initial_positions[2]
    initial_pose.pose.orientation.w = initial_positions[3]
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[0]
    shelf_item_pose.pose.position.y = shelf_positions[1]
    shelf_item_pose.pose.orientation.z = shelf_positions[2]
    shelf_item_pose.pose.orientation.w = shelf_positions[3]
    print('Received request for item picking at .')
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



