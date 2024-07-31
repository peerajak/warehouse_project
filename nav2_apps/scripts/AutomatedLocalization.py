# Normally Localization of the robot on a map in RVIZ is done by hardcoding the map to odom transformation x,y,theta
# These 3 numbers are usually found by RVIZ "2D Pose Estimate" button.
# However, if we have to do this in real robot everytime, this can be very slow.
# In this file, I want to try automate the localization.
from rclpy.node import Node
import rclpy
import time
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from rclpy.task import Future
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.duration import Duration


is_localized = False
is_odom_initialized = False
localize_threshold = [0.03 ,0.03,0.03]
initial_positions = [ 0,0,0,0 ]

realrobot_move_topic = '/cmd_vel'
simrobot_move_topic = '/diffbot_base_controller/cmd_vel_unstamped'
is_using_sim_robot = False

# Shelf positions for picking
shelf_positions = [ 2.3922234933879425, 0.004731793330113001 , 1.0, 0.00]

class LocalizeNode(Node):
    
    def __init__(self):
        super().__init__('minimal_subscriber')
        global is_using_sim_robot
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
        if is_using_sim_robot:
            self.publisher_rotate = self.create_publisher(
                msg_type=Twist,
                topic=simrobot_move_topic,qos_profile=1)
        else:
            self.publisher_rotate = self.create_publisher(
                msg_type=Twist,
                topic=realrobot_move_topic,qos_profile=1)
        self.future: Future = None

    def amcl_listener_callback(self, msg):
        global is_localized
        global initial_positions 
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
                #position = msg.pose.pose.position
                #orientation = msg.pose.pose.orientation
                #(initial_positions[0], initial_positions[1], posz) = (position.x, position.y, position.z)
                #(qx, qy, initial_positions[2], initial_positions[3]) = (orientation.x, orientation.y, 
                #                                                    orientation.z, orientation.w)

        

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

def wait_backup_or_spin(navigator):
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        print(feedback)


def main():
    rclpy.init()



    localize_node = LocalizeNode()
    while not is_localized:
        rclpy.spin_once(localize_node)
        print('localizing..')
        time.sleep(0.05)
    localize_node.destroy_node()
    print('initial_position = ' ,initial_positions)


    odom_subscriber = OdomSubscriber()
    while not is_odom_initialized:
        rclpy.spin_once(odom_subscriber)
        print('waiting odom data..')
        time.sleep(0.1)
    print(initial_positions)
    odom_subscriber.destroy_node()


    navigator = BasicNavigator()
    # # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x =initial_positions[0]
    initial_pose.pose.position.y = initial_positions[1]
    initial_pose.pose.orientation.z = initial_positions[2]
    initial_pose.pose.orientation.w = initial_positions[3]

    # navigator.setInitialPose(initial_pose)


    # shelf_item_pose = PoseStamped()
    # shelf_item_pose.header.frame_id = 'map'
    # shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    # shelf_item_pose.pose.position.x =shelf_positions[0]
    # shelf_item_pose.pose.position.y =shelf_positions[1]
    # shelf_item_pose.pose.orientation.z =shelf_positions[2]
    # shelf_item_pose.pose.orientation.w =shelf_positions[3]
    # #print('Received request for item picking at ' + request_item_location + '.')
    # print(initial_positions)
    # navigator.goToPose(shelf_item_pose)
    # wait_backup_or_spin(navigator)
    # navigator.spin(spin_dist=1.57)
    # wait_backup_or_spin(navigator)
    # navigator.backup(backup_dist=5.0, backup_speed=2.0)
    # wait_backup_or_spin(navigator)


    
if __name__ == '__main__':

    main()

    exit(0)