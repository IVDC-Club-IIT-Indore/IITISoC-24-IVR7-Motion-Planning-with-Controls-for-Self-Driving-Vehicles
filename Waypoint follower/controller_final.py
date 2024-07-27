import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import tf_transformations
import numpy as np
import math
import csv
        
kp_linear = 0.5  # Proportional gain for linear velocity
kd_linear = 0
ki_linear = 0
         
kp_angular = 2.0  # Proportional gain for angular velocity
kd_angular = 0
ki_angular = 0

#One pid loop which will be accessed whenever necessary 
def compute(self, error, sample_time, kp, kd, ki):
    self.integral += error * sample_time
    derivative = (error - self.previous_error) / sample_time
    output = kp * error + ki * self.integral + kd * derivative
    self.previous_error = error
    return output


def get_steering_direction(self, v1, v2):
    corss_prod = v1[0]*v2[1] - v1[1]*v2[0]
    if corss_prod >= 0:
        return -1
    return 1        
    
#Get crosstrack error at the instant        
def get_crosstrack_error(self, i, x, y, waypoints):
    P = np.asarray([x, y])
  
    if i == 0:
        A = np.asarray([waypoints[i][0], waypoints[i][1]])
        B = np.asarray([waypoints[i+1][0], waypoints[i+1][1]])
    else:
        A = np.asarray([waypoints[i-1][0], waypoints[i-1][1]])
        B = np.asarray([waypoints[i][0], waypoints[i][1]])
    n = B-A
    m = P-A
    dirxn = self.get_steering_direction(n, m)
    crosstrack_error = dirxn*(np.abs(((B[0]-A[0])*(A[1]-P[1]))-((A[0]-P[0])*(B[1]-A[1])))/np.sqrt((B[0]-A[0])**2+(B[1]-A[1])**2))
    return crosstrack_error
        
def normalize_angle(self, angle):
    if angle > math.pi:
        angle -= 2.0 * math.pi
    elif angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
    
#Stops the bot once endpoint is reached    
def stop_robot(self):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    self.cmd_vel_publisher.publish(twist)


#Actual algorithm to move the bot along the path        
def move_to_waypoints(self, x, y):
    if self.current_position is None or self.current_orientation is None:
        return

    if self.current_waypoint_index >= len(self.waypoints):
        self.get_logger().info('All waypoints reached!')
        self.stop_robot()
        return
    cte = get_crosstrack_error(i, current_x, current_y, waypoints)
    waypoint = self.waypoints[self.current_waypoint_index]
    target_x, target_y = waypoint
    angle_to_goal = math.atan2(target_y - y, target_x - x)
    err_ang = self.normalize_angle(angle_to_goal - self.current_yaw)

    if distance < self.waypoint_threshold:
        self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached. Moving to next waypoint.')
        self.current_waypoint_index += 1
        return

    angular_velocity = compute(cte, 0.1, kp_linear, kd_linear, ki_linear)
    linear_velocity = compute(err_ang, 0.1, kp_angular, kd_angular, ki_angular)

    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity
    self.cmd_vel_publisher.publish(twist)
        
        


            
class DiffDriveControllerWithIMU(Node):

    def __init__(self):
        super().__init__('diff_drive_controller_with_imu')
        
        self.waypoint_threshold = 0.2  # Distance to waypoint before switching to the next

        # Subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu_plugin/out',
            self.imu_callback,
            10)
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.x = None
        self.y = None
        self.current_orientation = None
        self.current_yaw = None
        self.angular_velocity = None
        self.current_waypoint_index = 0
        


    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q)
        
        self.move_to_waypoints(x, y)

# Process IMU data
    def imu_callback(self, msg):
        self.angular_velocity = msg.angular_velocity.z

# Convert quaternion to euler angles
    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z





def main(args=None):
    rclpy.init(args=args)
    diff_drive_controller_with_imu = DiffDriveControllerWithIMU()
    rclpy.spin(diff_drive_controller_with_imu)
    diff_drive_controller_with_imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
