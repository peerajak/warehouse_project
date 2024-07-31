import time
from copy import deepcopy
import shutil
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from custom_interfaces.srv import GoToLoading
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.duration import Duration
from rclpy.task import Future
from rclpy.node import Node
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import ReliabilityPolicy, QoSProfile

shipping_destinations =  [1.3,0.4,-0.4,-0.3847139877813617]

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

def wait_backup_or_spin(navigator):
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        print(feedback)



def main():
    global nstate
    rclpy.init()

    navigator = BasicNavigator()
    # shipping_destination = PoseStamped()
    # shipping_destination.header.frame_id = 'map'
    # shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
    # shipping_destination.pose.position.x = shipping_destinations[0]
    # shipping_destination.pose.position.y = shipping_destinations[1]
    # shipping_destination.pose.orientation.z = shipping_destinations[2]
    # shipping_destination.pose.orientation.w = shipping_destinations[3]
    # navigator.goToPose(shipping_destination)
    navigator.spin(spin_dist=3.14, time_allowance=10)
    wait_backup_or_spin(navigator)

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