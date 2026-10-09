# node called localization, which subscribes to the topic /cmd_vel and 
# estimates the robot's position starting from the axis's origin, considering the topic's period of 1 
# s and the velocity of 1 m/s

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist

class Localization(Node):

    def __init__(self):
        super().__init__('localization')

    # --------------------------------------------------
    # Subscription to the topic /cmd_vel
    # ---------------------------------------------------
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    # --------------------------------------------------
    # Publisher to the topic /pose
    # ---------------------------------------------------
        self.publisher_ = self.create_publisher(Pose, 'pose', 10)
        self.time = 1.0 #seconds -> 1Hz
        self.timer = self.create_timer(self.time, self.timer_callback)

        self.x = 0.0 #Robot's position along the X-axis
        self.y = 0.0 #Robot's position along the Y-axis



    def listener_callback(self, msg):
        # Update the robot's position based on the received velocity command
        # self.get_logger().info(f'I received: vel_x: {msg.linear.x},vel_y: {msg.linear.y}') 

        self.x += msg.linear.x * self.time #Deslocamento = velocidade * tempo, sendo que o tempo é 1 segundo
        self.y += msg.linear.y * self.time 


    def timer_callback(self):
        msg = Pose()
        msg.position.x = self.x
        msg.position.y = self.y

        self.publisher_.publish(msg)

        self.get_logger().info(f'Robot position:{msg.position}')

def main(args = None):
    rclpy.init(args=args)

    localization = Localization()

    rclpy.spin(localization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
