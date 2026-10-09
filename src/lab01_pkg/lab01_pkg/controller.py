# Node which  publishes velocity commands on a topic called 
# /cmd_vel of type geometry_msgs/msg/Twist at a frequency of 1 Hz

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) 
        time = 1.0 #seconds
        self.timer = self.create_timer(time, self.timer_callback)
        
        # Number of seconds for each movement
        self.N = 1


        # Current direction
        # 0 = +X
        # 1 = +Y
        # 2 = -X
        # 3 = -Y
        self.direction = 0 

    def timer_callback(self):
        msg = Twist()


        if self.direction == 0:
            msg.linear.x = self.N * 1.0
            direction_name = "X-axis"
            self.publisher_.publish(msg)
            self.get_logger().info(f"{self.N} seconds along the {direction_name}")
            self.direction = 1

        elif self.direction == 1:
            msg.linear.y = self.N * 1.0
            direction_name = "Y-axis"
            self.publisher_.publish(msg)
            self.get_logger().info(f"{self.N} seconds along the {direction_name}")
            self.direction = 2

        elif self.direction == 2:
            msg.linear.x = self.N * -1.0
            direction_name = "-X-axis"
            self.publisher_.publish(msg)
            self.get_logger().info(f"{self.N} seconds along the {direction_name}")
            self.direction = 3

        else:
            msg.linear.y = self.N * -1.0
            direction_name = "-Y-axis"
            self.publisher_.publish(msg)
            self.get_logger().info(f"{self.N} seconds along the {direction_name}")
            self.direction = 0
            self.N += 1


        

        

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


