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

        self.velx = 0.0
        self.ang_velz = 0.0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.velx
        msg.angular.z = self.ang_velz
        self.publisher_.publish(msg)
        print(f"Published velocity command: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")

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

        # Detect obstacles in front of the robot
        front = list(self.scan_data[0:4]) + list(self.scan_data[356:360])

        if any(0.0 < distance < 0.8 for distance in front):
            print("Obstacle detected in front of the robot!")
            self.velx = 0.0

            # Check the left side
            left = self.scan_data[49:57]

            # Check the right side
            right = self.scan_data[13:22]

            if any(0.0 < distance <= 2.0 for distance in left):
                self.ang_velz = 1.5  # Turn left
            elif any(0.0 < distance <= 2.0 for distance in right):
                self.ang_velz = -1.5  # Turn right
            else:
                self.ang_velz = 1.5  # Default turning direction
        else:
            self.velx = 0.22
            self.ang_velz = 0.0

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