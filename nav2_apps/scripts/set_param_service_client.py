from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rclpy.task import Future
from rclpy.node import Node
import rclpy



class ServiceClient(Node):
  def __init__(self):
    super().__init__('service_client')
    self.get_logger().info('init service_client')
    my_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    self.service_client2 = self.create_client(
            srv_type=SetParameters,
            srv_name="/global_costmap/global_costmap/set_parameters",
            callback_group=my_callback_group)
    self.service_client3 = self.create_client(
            srv_type=SetParameters,
            srv_name="/local_costmap/local_costmap/set_parameters",
            callback_group=my_callback_group)


    self.future2: Future = None
    self.future3: Future = None


    timer_period: float = 1.0
    self.timer2 = self.create_timer(timer_period_sec=timer_period,callback=self.timer2_callback)


  def timer2_callback(self):  
    self.get_logger().info('timer_callback lifecycle_service_client')
    while not self.service_client2.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {self.service_client2.srv_name} not available, waiting...')
    request = SetParameters.Request()
    request.parameters = [Parameter(name= 'robot_radius',  
            value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, 
                    double_value= 0.2))  ]
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
                    string_value= '[ [0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15] ]'))]
    self.future3 = self.service_client3.call_async(request)
    self.future3.add_done_callback(self.response3_callback)
    self.timer3.cancel()


  def response2_callback(self, future: Future):
    global nstate
    response = future.result()
    if response is not None:        
        #self.get_logger().info("Some Response happened")
        if(response.results[0].successful):
            self.get_logger().info("response from lifecycle service server: Success!"+response.results[0].reason)
            timer_period: float = 1.0
            self.timer3 = self.create_timer(timer_period_sec=timer_period,callback=self.timer3_callback)
        else:
            self.get_logger().info("response from lifecycle service server: Failed!")

    else:
        self.get_logger().info("The response is None")

  def response3_callback(self, future: Future):
    response = future.result()
    if response is not None:
        #self.get_logger().info("Some Response happened")
        if(response.results[0].successful):
            self.get_logger().info("response from lifecycle service server: Success!"+response.results[0].reason)
        else:
            self.get_logger().info("response from lifecycle service server: Failed!")           
    else:
        self.get_logger().info("The response is None")
    print('leaving service node') 
    rclpy.shutdown()
 


def main():

    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    service_client_node = ServiceClient()    
    executor.add_node(service_client_node )
    executor.spin()





if __name__ == '__main__':
    main()
    exit(0)


