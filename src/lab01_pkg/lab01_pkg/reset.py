#  node called reset_node that subscribes to /pose. When the distance from the 
# origin of the reference frame is larger than 6.0 m, publish a boolean value 
# std_msgs/msg/Bool (True or False, as you wish) on the topic /reset to take care of this 
# limit condition and reset the node. 

import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

class Reset(Node):

    def __init__(self):
        super().__init__('reset_node')

        #Subscriber to the topic /pose
        self.subscriber = self.create_subscription(
            Pose,
            'pose',
            self.listener_callback,
            10)
        self.subscriber  # prevent unused variable warning

        #Publisher to the topic /reset
        self.publisher_ = self.create_publisher(Bool, 'reset', 10)


    def listener_callback(self, msg):

        reset_msg = Bool() # creates the boolean message
        distance = self.distance_from_origin(msg.position.x, 
                                             msg.position.y)
        if distance > 6.0:
            reset_msg.data = True 
        else: 
            reset_msg.data = False

        self.get_logger().info(f'Distance from origin: {distance:.2f} m, Reset: {reset_msg.data}')
        self.publisher_.publish(reset_msg)           


    def distance_from_origin(self, x: float, y: float):
        '''function to calculate the distance from the origin'''
        d = np.sqrt(x**2 + y**2)
        
        return d

def main(args = None):
    rclpy.init(args=args)

    reset_node = Reset()

    rclpy.spin(reset_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

