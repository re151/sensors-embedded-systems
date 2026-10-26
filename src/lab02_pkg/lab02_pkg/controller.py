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
        self.max_linear_velocity = 0.22 # m/s
        self.max_angular_velocity = 1.0 # rad/s

        # Obstacle avoidance settings
        self.obstacle_distance = 0.80

        # Controller states
        self.state = 'FORWARD'
        self.direction = None
        self.scan_received = False
        self.odom_received = False

        # Goal position
        self.goal_pose = [6.5,3.0,0.0]

        

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

    # ----------------------------------------------------------
    # Subscription to the topic /ground_truth
    # ----------------------------------------------------------
    # Reads the ground_truth data of robot's position and stores it in the variable self.odom_data

        # self.gt_subscription = self.create_subscription(
        #     Odometry,
        #     'ground_truth',
        #     self.gt_callback,
        #     10)
        # self.gt_subscription  # prevent unused variable warning

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

    def goal_reached(self, msg):

        "Detect if the robots is closer enought to the goal (same distance as the used to detect walls)"
        pose = msg.pose.pose.position

        x = pose.x
        y = pose.y

        distance = np.sqrt(((self.goal_pose[0] - x)**2 + (self.goal_pose[1]-y)**2))

        if distance <= 1.0:

            self.velx = 0.0
            self.velz = 0.0
            self.state = 'REACHED'
            self.get_logger().info(f'Goal reached')
        
        return


    def find_direction(self,msg):

        """Decides if the robot must rotate to the left or to the right  """

        ranges = np.array(msg.ranges) 
        n = len(ranges)

        if n == 0:
            return None

        # Calculate the angle of each measurement
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        # Convert angles to degrees in [-180, 180)
        angles_deg = (np.degrees(angles) + 180) % 360 - 180

        # Select left and right sectors
        left_mask = (angles_deg >= 0) & (angles_deg <= 80)
        right_mask = (angles_deg >= -80) & (angles_deg < 0)

        left = ranges[left_mask].copy()
        right = ranges[right_mask].copy()

        # Keep the original LiDAR indices
        left_indices = np.where(left_mask)[0]
        right_indices = np.where(right_mask)[0]

        # Replace positive infinity with maximum range
        left[np.isposinf(left)] = msg.range_max
        right[np.isposinf(right)] = msg.range_max

        # Ignore invalid readings
        left_valid = np.isfinite(left) & (left > 0)
        right_valid = np.isfinite(right) & (right > 0)

        if np.count_nonzero(left_valid) < 3 or np.count_nonzero(right_valid) < 3:
            return None, None

        # Find the indices of the three largest distances
        left_top5 = np.argsort(left_valid * left)[-5:]
        right_top5 = np.argsort(right_valid * right)[-5:]

        # Sort the three indices and select the middle one
        left_middle = np.sort(left_indices[left_top5])[1]
        right_middle = np.sort(right_indices[right_top5])[1]

        # Convert the middle indices into angles
        left_angle = np.degrees(msg.angle_min + left_middle * msg.angle_increment)


        right_angle = np.degrees(msg.angle_min + right_middle * msg.angle_increment)

        # Compare the average distances
        left_mean = np.mean(left[left_valid])
        right_mean = np.mean(right[right_valid])

        if left_mean > right_mean:

            return 'Left', left_angle 
        else:

            return 'Right', right_angle
        

    # left = np.array(ranges[0:71]) # 0 -> 105°
        # right = np.array(ranges[160:230]) # 255° -> 358.5°

        # # Convert 'inf' to the higest value 
        # left[np.isinf(left)] = 3.5
        # right[np.isinf(right)] = 3.5

        # # Indices of the three largest values in each vector
        # left_idx = np.argsort(left)[-3:]
        # right_idx = np.argsort(right)[-3:]

        # # Sort the indices and select the middle one
        # left_idx = np.sort(left_idx)
        # right_idx = np.sort(right_idx)

        # left_middle = left_idx[1]
        # right_middle = right_idx[1]

        # # Avarege distance
        # left_sum = sum(left)/len(left)
        # right_sum = sum(right)/len(right)

        # if left_sum > right_sum:

        #     return 'Left', left_middle*1.58
        # else:
    
        #     return 'Right', ((right_middle+170)*1.58 - 360)


    def scan_callback(self, msg):

        self.scan_received = True
        self.scan_data = np.array(msg.ranges)

        # Do not change the target while turning.
        if self.state == 'Turn':
            return

        elif self.state == 'FORWARD':

            n = len(self.scan_data)
            if n == 0:
                self.velx = 0.0
                self.ang_velz = 0.0
                return
            
            angles = msg.angle_min + np.arange(n) * msg.angle_increment

            # Convert angles to degrees in [-180, 180).
            angles_deg = (np.degrees(angles) + 180) % 360 - 180

            # Select the frontal sector: -16° to +16°.
            front_mask = (angles_deg >= -20) & (angles_deg <= 20)
            front = self.scan_data[front_mask]

            # Detect obstacles only from valid measurements.
            valid_front = np.isfinite(front) & (front > 0)
            obstacle_detected = np.any(
                valid_front & (front < self.obstacle_distance)
            )

            if not obstacle_detected:
                self.velx = self.max_linear_velocity
                self.ang_velz = 0.0
                return
            
            # Obstacle detected: stop before selecting a direction.
            self.velx = 0.0
            self.ang_velz = 0.0


            self.direction, angle = self.find_direction(msg)

            # Check whether the LiDAR data is sufficient.
            if self.direction is None or angle is None:
                return

            # Set the target once, using the current odometry yaw.
            self.target_yaw = self.normalize_angle(self.yaw + math.radians(angle))
            
            self.state = 'Turn'

            self.get_logger().info(f'Obstacle detected. Target yaw: '
                                f'{math.degrees(self.target_yaw):.1f} degrees')
        else:
            return


    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        _, _, self.yaw = tf_transformations.euler_from_quaternion(quaternion)

        self.odom_received = True
        self.goal_reached(msg)
        
        if self.state == 'FORWARD':
            return
        
        elif self.state == 'Turn':
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
        else: 
            return

# def gt_callback(self, msg):
    #     q = msg.pose.pose.orientation
    #     quaternion = [q.x, q.y, q.z, q.w]

    #     _, _, self.yaw = tf_transformations.euler_from_quaternion(quaternion)

    #     self.odom_received = True
    #     self.goal_reached(msg)
        
    #     if self.state == 'FORWARD':
    #         return
        
    #     elif self.state == 'Turn':
    #         # Calculate the shortest angular error.
    #         yaw_error = self.normalize_angle(self.target_yaw - self.yaw)

    #         if abs(yaw_error) < self.yaw_tolerance:
    #             # Turn complete.
    #             self.ang_velz = 0.0
    #             self.velx = self.max_linear_velocity
    #             self.state = 'FORWARD'

    #             self.get_logger().info('Turn complete. Moving forward.')
            
    #         else:
    #             # Rotate in the direction of the target.
    #             self.velx = 0.0

    #             if self.direction == 'Left':
    #                 self.ang_velz = self.max_angular_velocity
    #             else: 
    #                 self.ang_velz = -self.max_angular_velocity
    #     else: 
    #         return

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    controller.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()