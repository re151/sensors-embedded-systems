# Node that implements a controller capable of guiding our TB3 through the path 
# delimited by walls to reach the green cube. 
#The starting pose is [x, y, yaw] = (0, 0, 0), the final goal is the green cube. 

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

    #----------------------------------------------------------
    # Publisher to the topic /cmd_vel
    #----------------------------------------------------------
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer = 0.5 #seconds
        self.timer = self.create_timer(timer, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    # ----------------------------------------------------------
    # Subscription to the topic /scan
    # ----------------------------------------------------------
    # Reads the laser scan data and stores it in the variable self.scan_data

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.scan_subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        if self.scan_data[]
        print(f"Scan data: {self.scan_data}")



def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    controller.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()