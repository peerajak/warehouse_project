import time
import shutil
from geometry_msgs.msg import PoseStamped

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():

    rclpy.init()
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
    navigator.waitUntilNav2Active()
    while True:
        time.sleep(1)
        
if __name__ == '__main__':

    main()

    exit(0)