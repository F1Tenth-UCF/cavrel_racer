import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # PID parameters (tuned for stable driving)
        self.kp = 0.4
        self.kd = 0.1
        self.ki = 0.0005

        self.integral = 0.0
        self.prev_error = 0.0

        # Speed parameter
        self.min_speed = 0.5                #0.5 - spielberg
        self.max_speed = 5.0                #7.0 - spielberg map

        # Desired distance to left-side wall
        self.desired_distance = 0.7         #1.6 - spielberg map

        # Look into the future distance
        self.lookahead_distance = 1.5       #1.5 - spielberg map

    def get_range(self, data, angle_deg):
        angle_rad = np.radians(angle_deg)
        index = int((angle_rad - data.angle_min) / data.angle_increment)
        index = np.clip(index, 0, len(data.ranges)-1)

        distance = data.ranges[index]
        if np.isnan(distance) or np.isinf(distance):
            distance = 10.0

        return distance

    def get_error(self, data):
        # Angles defined for left-side wall
        theta = 40  # front-left
        dist_a = self.get_range(data, theta)
        dist_b = self.get_range(data, 90)

        alpha = np.arctan2(dist_a * np.cos(np.radians(theta)) - dist_b,
                           dist_a * np.sin(np.radians(theta)))

        # Current perpendicular distance to left-side wall
        current_dist = dist_b * np.cos(alpha)

        # Predict future position
        future_dist = current_dist + self.lookahead_distance * np.sin(alpha)

        error = self.desired_distance - future_dist
        return error

    def pid_control(self, error, velocity):
        derivative = error - self.prev_error
        self.integral += error
        self.integral = np.clip(self.integral, -0.3, 0.3)

        steering_angle = -(self.kp * error + self.kd * derivative + self.ki * self.integral)
        steering_angle = np.clip(steering_angle, -0.70, 0.70)

        self.prev_error = error

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

        # Log speed, steering angle, and error
        self.get_logger().info(
            f"PID Output -> Speed: {velocity:.2f} m/s, Steering Angle: {steering_angle:.3f} rad, Error: {error:.3f} m"
        )

    def scan_callback(self, data):
        error = self.get_error(data)
        velocity = max(self.min_speed, self.max_speed - (10 * abs(error)))
        self.pid_control(error, velocity)

def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
