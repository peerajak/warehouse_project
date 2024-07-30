import time
from copy import deepcopy
import shutil
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from custom_interfaces.srv import GoToLoading
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.duration import Duration
from rclpy.task import Future
from rclpy.node import Node
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import ReliabilityPolicy, QoSProfile

initial_positions =  [1.3,0.4,-0.4,-0.3847139877813617]#[0.,0.,0.,0.]
shipping_destinations = [1.3,0.4,-0.4,-0.3847139877813617]
is_localized = False
is_odom_initialized = False
realrobot_move_topic = '/cmd_vel'
simrobot_move_topic = '/diffbot_base_controller/cmd_vel_unstamped'
localize_threshold = [0.03 ,0.03,0.03]

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
        #(initial_positions[0], initial_positions[1], posz) = (position.x, position.y, position.z)
        #(qx, qy, initial_positions[2], initial_positions[3]) = (orientation.x, orientation.y, orientation.z, orientation.w)
        is_odom_initialized = True




def main():
    global nstate
    rclpy.init()
    # odom_subscriber = OdomSubscriber()
    # while not is_odom_initialized:
    #     rclpy.spin_once(odom_subscriber)
    #     print('waiting odom data..')
    #     time.sleep(0.1)
    # odom_subscriber.destroy_node()
    print('initial positions reset to ',initial_positions)


    localize_node = LocalizeNode()
    while not is_localized:
        rclpy.spin_once(localize_node)
        print('localizing..')
        time.sleep(0.05)
    localize_node.destroy_node()


    navigator = BasicNavigator()
    shipping_destination = PoseStamped()
    shipping_destination.header.frame_id = 'map'
    shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
    shipping_destination.pose.position.x = shipping_destinations[0]
    shipping_destination.pose.position.y = shipping_destinations[1]
    shipping_destination.pose.orientation.z = shipping_destinations[2]
    shipping_destination.pose.orientation.w = shipping_destinations[3]
    navigator.goToPose(shipping_destination)
    while not navigator.isTaskComplete():
        pass
    result1 = navigator.getResult()
    if result1 == TaskResult.SUCCEEDED:
        print('success')
    elif result1 == TaskResult.CANCELED:
        print('was canceled. Returning to staging point...')
    elif result1 == TaskResult.FAILED:
        print(' failed!')
        exit(-1)
    while not navigator.isTaskComplete():
        pass

    initial_position_pose = PoseStamped()
    initial_position_pose.header.frame_id = 'map'
    initial_position_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_position_pose.pose.position.x = initial_positions[0]
    initial_position_pose.pose.position.y = initial_positions[1]
    initial_position_pose.pose.orientation.z = initial_positions[2]
    initial_position_pose.pose.orientation.w = initial_positions[3]
    navigator.goToPose(initial_position_pose)
    print('initial positions reset to ',initial_positions)
    while not navigator.isTaskComplete():
        pass
    result1 = navigator.getResult()
    if result1 == TaskResult.SUCCEEDED:
        print('success')
    elif result1 == TaskResult.CANCELED:
        print('was canceled. Returning to staging point...')
    elif result1 == TaskResult.FAILED:
        print(' failed!')
        exit(-1)
    while not navigator.isTaskComplete():
        pass

if __name__ == '__main__':

    main()

    exit(0)