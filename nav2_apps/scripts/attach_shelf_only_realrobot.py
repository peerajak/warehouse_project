#! /usr/bin/env python3
# Real Robot

import time
from copy import deepcopy
import shutil
from enum import Enum
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


class TheState(Enum):
    ToPreload = 0
    AttachShelf = 1
    ToShelfReverse = 2 
    ToBeforeShipping = 3
    ToShipping = 4
    ToShippingReverse = 5
    BackToBeforeShipping = 6
    BackToInitialPosition = 7
    EndProgramSuccess = 8
    EndProgramFailure = 9



nstate = TheState.ToPreload
is_localmap_param_set = False
is_globalmap_param_set = False
is_lifting_done = False
is_rotating_done = False
is_localized = False
is_odom_rotated = False
localize_threshold = [0.03 ,0.03,0.03]
realrobot_move_topic = '/cmd_vel'
simrobot_move_topic = '/diffbot_base_controller/cmd_vel_unstamped'



initial_positions = [-0.9,-0.3,0.1,0.999]

shelf_positions = [3.5, -1.6,0.7638911945621069,-0.6080598417892362]
shelf_positions_reverse = [3.0,-1.7,-0.4,-0.3847139877813617]
shipping_destinations = [1.3,0.5,-0.4,-0.3847139877813617]

shipping_destinations_reverse = [1.3,0.6,0.9,-0.3847139877813617]

before_shipping = [1.02,-1.3,-0.9230358322434521,-0.3847139877813617]

before_shipping_reverse =  [1.02,-1.3,-0.3929562957056833,0.9195571486673721]            



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

    def __init__(self, target_yaw_rad):
        global is_odom_rotated
        is_odom_rotated = False
        self.target_yaw_rad_ = target_yaw_rad
        super().__init__('minimal_subscriber')
        self.subscriber_odom = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.publisher_rotate = self.create_publisher(
            msg_type=Twist,
            topic=realrobot_move_topic,qos_profile=1)

    def check_reached_goal_desire_angle(self, delta_error = 0.08): 
        delta_theta =  abs(self.radian_difference(self.target_yaw_rad_, self.current_yaw_rad_))
        return delta_theta < delta_error   
  
    
    def rotate_at_the_end(self,):
        global is_odom_rotated
        self.get_logger().info( "rotate_at_the_end() target %f, current %f " %(self.target_yaw_rad_, self.current_yaw_rad_))
        ling = Twist()
        while not self.check_reached_goal_desire_angle():
            angular_z_raw = lambda first, second : first - second if abs(first - second) <= 3.14 else (first - second) - 2 * 3.14 
            ling.linear.x = 0.0
            ling.angular.z =  angular_z_raw if angular_z_raw < 1.5 else 0.5 * angular_z_raw
            self.publisher_rotate.publish(ling)   
            self.get_logger().info(
                "Rotating current pos=['%f','%f'] target rad "
                "'%f',current rad %f, angular speed %f" %(
                self.current_pos_.x, self.current_pos_.y, self.target_yaw_rad_, self.current_yaw_rad_,
                ling.angular.z))
            time.sleep(0.1)        
        ling.linear.x = 0.0
        ling.angular.z =0.0
        self.publisher_rotate.publish(ling)
        is_odom_rotated = True  

    def odom_callback(self, msg2):
        global initial_positions 
        global is_odom_initialized
        self.current_pos_  = msg2.pose.pose.position
        self.current_angle_ = msg2.pose.pose.orientation
        # (self.current_yaw_roll_,self.current_yaw_pitch_,self.current_yaw_rad_) = \
        #     euler_from_quaternion ([
        # msg2.pose.pose.orientation.x, msg2.pose.pose.orientation.y,
        # msg2.pose.pose.orientation.z, msg2.pose.pose.orientation.w])




class SetParameterClient(Node):
  def __init__(self,robot_radius,tolerance, footprint, inflation_radius,xy_goal_tolerance):
    super().__init__('setParameterClient')
    global is_localmap_param_set
    global is_globalmap_param_set
    is_localmap_param_set = False
    is_globalmap_param_set = False
    self.get_logger().info('SetParameterClient robot_radius %f, tolerance %f, goal_tolerance %f, inflation_radius %f, footprint '%(robot_radius,tolerance,xy_goal_tolerance,inflation_radius)+footprint)
    my_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    self.service_client2 = self.create_client(
            srv_type=SetParameters,
            srv_name="/global_costmap/global_costmap/set_parameters",
            callback_group=my_callback_group)
    self.service_client3 = self.create_client(
            srv_type=SetParameters,
            srv_name="/local_costmap/local_costmap/set_parameters",
            callback_group=my_callback_group)
    self.robot_radius = robot_radius
    self.tolerance = tolerance
    self.robot_footprint = footprint
    self.inflation_radius = inflation_radius
    self.xy_goal_tolerance = xy_goal_tolerance
    timer_period: float = 0.1
    self.timer2 = self.create_timer(timer_period_sec=timer_period,callback=self.timer2_callback)
    self.timer3 = self.create_timer(timer_period_sec=timer_period,callback=self.timer3_callback)

  def timer2_callback(self):  
    self.get_logger().info('timer2_callback lifecycle_service_client')
    while not self.service_client2.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {self.service_client2.srv_name} not available, waiting...')
    request = SetParameters.Request()
    request.parameters = [
            Parameter(name= 'robot_radius',  
            value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, 
                    double_value= self.robot_radius)), 
            Parameter(name= 'inflation_radius',  
            value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, 
                    double_value= self.inflation_radius)),
            Parameter(name= 'tolerance',  
            value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, 
                    double_value= self.tolerance))  ]
    self.future2 = self.service_client2.call_async(request)
    self.future2.add_done_callback(self.response2_callback)
    self.timer2.cancel()

  def timer3_callback(self):  
    self.get_logger().info('timer3_callback lifecycle_service_client')
    while not self.service_client3.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {self.service_client3.srv_name} not available, waiting...')
    request = SetParameters.Request()
    request.parameters = [
            Parameter(name= 'footprint',  
            value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, 
                    string_value= self.robot_footprint)),
            Parameter(name= 'xy_goal_tolerance',  
            value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, 
                    double_value= self.xy_goal_tolerance))]
    self.future3 = self.service_client3.call_async(request)
    self.future3.add_done_callback(self.response3_callback)
    self.timer3.cancel()
  
  def response2_callback(self, future: Future):
    global nstate
    global is_globalmap_param_set
    response = future.result()
    if response is not None:
        self.get_logger().info("Some Response2 happened")
        if(response.results[0].successful):
            is_globalmap_param_set = True
            self.get_logger().info("response from lifecycle service server: Success!"+response.results[0].reason)
        else:
            is_globalmap_param_set = False
            self.get_logger().info("response from lifecycle service server: Failed!")
            nstate = TheState.EndProgramFailure
    else:
        is_globalmap_param_set = False
        self.get_logger().info("The response is None")

  def response3_callback(self, future: Future):
    global nstate
    global is_localmap_param_set 
    response = future.result()
    if response is not None:
        self.get_logger().info("Some Response2 happened")
        if(response.results[0].successful):
            is_localmap_param_set = True
            self.get_logger().info("response from lifecycle service server: Success!"+response.results[0].reason)
        else:
            is_localmap_param_set = False
            self.get_logger().info("response from lifecycle service server: Failed!")
            nstate = TheState.EndProgramFailure
    else:
        is_localmap_param_set = False
        self.get_logger().info("The response is None")
   
class LiftUpDown(Node):
  def __init__(self, is_lift_up):
    super().__init__('lift_up_down_node')
    self.is_lift_up = is_lift_up
    self.publisher_lift = self.create_publisher(
        msg_type=String,
        topic='/elevator_up',
        qos_profile=1)
    self.publisher_liftdown = self.create_publisher(
            msg_type=String,
            topic='/elevator_down',
            qos_profile=1)
    timer_period: float = 0.01
    self.timer = self.create_timer(timer_period_sec=timer_period,callback=self.timer_callback)

  def timer_callback(self): 
    global is_lifting_done
    self.get_logger().info('timer_callback Lifting') 
    msgs_empty = String()
    if(self.is_lift_up):
        self.get_logger().info("Lift up")
        self.publisher_lift.publish(msgs_empty)
    else:
        self.get_logger().info("Lift down")
        self.publisher_liftdown.publish(msgs_empty)
    is_lifting_done = True
    self.timer.cancel()

class Rotation180(Node):
  def __init__(self):
    super().__init__('lift_up_down_node')

    self.service_client = self.create_client(
            srv_type=Empty,
            srv_name="/rotate180")

    timer_period: float = 0.01
    self.timer = self.create_timer(timer_period_sec=timer_period,callback=self.timer_callback)

  def timer_callback(self): 
    global is_rotating_done
    is_rotating_done = False
    while not self.service_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {self.service_client.srv_name} not available, waiting...') 
    self.get_logger().info('timer_callback Rotate180 service') 
    request = Empty.Request()
    self.future = self.service_client.call_async(request)
    self.future.add_done_callback(self.response_callback)
    self.timer.cancel()

  def response_callback(self, future: Future):
    global is_rotating_done
    response = future.result()
    if response is not None:
        self.get_logger().info("Some Response happened: Success!")
        is_rotating_done = True
    print('response from Rotate180 service')


class ServiceClient(Node):
  def __init__(self):
    super().__init__('service_client')
    self.get_logger().info('init service_client')
    my_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    self.service_client = self.create_client(
            srv_type=GoToLoading,
            srv_name="/approach_shelf",
            callback_group=my_callback_group)
    self.final_approach = False
    self.future: Future = None
    timer_period: float = 0.1
    self.timer = self.create_timer(timer_period_sec=timer_period,callback=self.timer_callback)
    self.publisher_lift = self.create_publisher(
            msg_type=String,
            topic='/elevator_up',
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
            if nstate == TheState.AttachShelf:
                nstate = TheState.ToShelfReverse
            # elif nstate == TheState.ToShippingReverse:
            #     nstate = TheState.BackToBeforeShipping
        else:
            self.get_logger().info("response from service server: Failed!")
            nstate = TheState.EndProgramFailure
    else:
        self.get_logger().info("The response is None")
        nstate = TheState.EndProgramFailure

def setNavigationGoal(dest, navigator):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = dest[0]
    pose.pose.position.y = dest[1]
    pose.pose.orientation.z = dest[2]
    pose.pose.orientation.w = dest[3]
    return pose

def wait_navigation(navigator):
    global nstate
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' +nstate.name +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

def nstate_change_to_navigation_result(result, nstate_success, nstate_failure):
    global nstate
    if result == TaskResult.SUCCEEDED:
        print('Navigation to '+nstate.name + ' Success')
        nstate = nstate_success
        return True
    else:
        print('Task at ' + nstate.name + ' failed!')
        nstate = nstate_failure
        return False




def main():
    global nstate
    global is_localmap_param_set 
    global is_globalmap_param_set 
    rclpy.init()
    robot_radius_large = 0.4
    robot_radius_initial = 0.15
    cart_form_factor = 1.2
    tolerance = 0.5
    xy_goal_tolerance_initial = 0.25
    inflation_radius = 0.25
    # localize_node = LocalizeNode()
    # while not is_localized:
    #     rclpy.spin_once(localize_node)
    #     print('localizing..')
    #     time.sleep(0.05)
    # localize_node.destroy_node()
    # attachShelf state
    service_client_node = ServiceClient()    
    while rclpy.ok and not(nstate == TheState.EndProgramFailure or nstate == TheState.EndProgramSuccess):
        rclpy.spin_once(service_client_node )
        print('at main '+nstate.name)
        if nstate == TheState.ToShelfReverse:
            break
    return

 













if __name__ == '__main__':

    main()

    if(nstate == TheState.EndProgramSuccess):
        exit(0)
    else:
        exit(-1)

