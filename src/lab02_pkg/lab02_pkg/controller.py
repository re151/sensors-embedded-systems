# Node that implements a controller capable of guiding our TB3 through the path 
# delimited by walls to reach the green cube. 
#The starting pose is [x, y, yaw] = (0, 0, 0), the final goal is the green cube. 

import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

    #----------------------------------------------------------
    # Publisher to the topic /cmd_vel
    #----------------------------------------------------------
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer = 0.05 #seconds

        # Publish velocity commands at 10 Hz
        self.timer = self.create_timer(timer, self.timer_callback)

        # Robot velocities
        self.velx = 0.0 # initial linear velocity
        self.ang_velz = 0.0 # initial angular velocity

        # Odometry and target heading
        self.yaw = 0.0 # initial rotation angle
        self.target_yaw = 0.0 # angle to be reached
        self.yaw_tolerance = math.radians(1.0) 

        # Speed limits
        self.max_linear_velocity = 0.3 # m/s
        self.max_angular_velocity = 1.0 # rad/s

        # Obstacle avoidance settings
        self.obstacle_distance = 0.85

        # Controller states
        self.state = 'FORWARD'
        self.direction = None
        self.scan_received = False
        self.odom_received = False

        

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
    # ----------------------------------------------------------
    # Subscription to the topic /odom
    # ----------------------------------------------------------
    # Reads the odometry data and stores it in the variable self.odom_data

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning


    @staticmethod
    def normalize_angle(angle):
        """Normalize an angle in radians to [-pi, pi)."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


    def timer_callback(self):
        # Remain stopped until both sensors have provided data.
        msg = Twist()

        if self.scan_received and self.odom_received:
            msg.linear.x = self.velx
            msg.angular.z = self.ang_velz

        self.publisher_.publish(msg)
       # print(f"Published velocity command: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")


    def find_direction(self,msg):

        """Decides if the robot must rotate to the left or to the right  """

        ranges = msg.ranges 
        n = len(ranges)

        if n == 0:
            return None

        left = np.array(ranges[0:71]) # 0 -> 105°
        right = np.array(ranges[160:230]) # 255° -> 358.5°

        # Convert 'inf' to the higest value 
        left[np.isinf(left)] = 3.5
        right[np.isinf(right)] = 3.5

        # Indices of the three largest values in each vector
        left_idx = np.argsort(left)[-3:]
        right_idx = np.argsort(right)[-3:]

        # Sort the indices and select the middle one
        left_idx = np.sort(left_idx)
        right_idx = np.sort(right_idx)

        left_middle = left_idx[1]
        right_middle = right_idx[1]

        # Avarege distance
        left_sum = sum(left)/len(left)
        right_sum = sum(right)/len(right)

        if left_sum > right_sum:

            return 'Left', left_middle*1.58
        else:
    
            return 'Right', ((right_middle+170)*1.58 - 360)

        

    def scan_callback(self, msg):

        self.scan_received = True
        self.scan_data = msg.ranges

        # Do not change the target while turning.
        if self.state != 'FORWARD':
            return

        front = np.concatenate((self.scan_data[0:10],self.scan_data[220:230])) # ~ +- 16°

        if not(any(d < self.obstacle_distance for d in front)):
            self.velx = self.max_linear_velocity
            self.ang_velz = 0.0
            return

        # Obstacle detected: stop before selecting a direction.
        self.velx = 0.0
        self.ang_velz = 0.0


        self.direction, angle = self.find_direction(msg)

        # Set the target once, using the current odometry yaw.
        self.target_yaw = self.normalize_angle(self.yaw + math.radians(angle))
        self.state = 'Turn'

        self.get_logger().info(f'Obstacle detected. Target yaw: '
                               f'{math.degrees(self.target_yaw):.1f} degrees')


    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        _, _, self.yaw = tf_transformations.euler_from_quaternion(quaternion)

        self.odom_received = True
        
        if self.state != 'Turn':
            return

        # Calculate the shortest angular error.
        yaw_error = self.normalize_angle(self.target_yaw - self.yaw)

        if abs(yaw_error) < self.yaw_tolerance:
            # Turn complete.
            self.ang_velz = 0.0
            self.velx = self.max_linear_velocity
            self.state = 'FORWARD'

            self.get_logger().info('Turn complete. Moving forward.')
        
        else:
            # Rotate in the direction of the target.
            self.velx = 0.0

            if self.direction == 'Left':
                self.ang_velz = self.max_angular_velocity
            else: 
                self.ang_velz = -self.max_angular_velocity

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    controller.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()