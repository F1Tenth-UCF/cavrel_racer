#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# ---------------------------- Tuning Parameters ----------------------------

kMaxRange = 2.0         #2.0 - spielberg map
kVehicleRadius = 0.3
kMinGapAngle = math.radians(3.0)
g_fovAngleMax = math.pi / 2 + math.pi / 16
g_goal_angle = 0.0
kGapWeightCoefficient = 100.0

# Speed tuning
MIN_SPEED = 0.5         #0.5 - spielberg map
MAX_SPEED = 7.0         #7.0 - spielberg map
STEERING_SCALE = 3.5  # Scale how much steering angle impacts speed

# ---------------------------- Exceptions ----------------------------

class NoGapFoundException(Exception): pass

# ---------------------------- Data Structures ----------------------------

class Obstacle:
    def __init__(self, distance, angle, radius=0.0):
        self.distance = distance
        self.angle = angle
        self.radius = radius
        self.x = distance * math.cos(angle)
        self.y = distance * math.sin(angle)

    def overlaps(self, other):
        return abs(self.angle - other.angle) < math.radians(2.0)

    def distance_between_centers(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)

class Gap:
    def __init__(self, obstacle_left, obstacle_right):
        self.obstacle_left = obstacle_left
        self.obstacle_right = obstacle_right
        self.angle_left = obstacle_left.angle
        self.angle_right = obstacle_right.angle
        self.gap_size = abs(self.angle_left - self.angle_right)

    def is_wide_enough(self):
        width = self.obstacle_left.distance_between_centers(self.obstacle_right)
        return width > 2 * kVehicleRadius

class LidarData:
    def __init__(self, range_min, range_max, angle_min, angle_max, angle_increment, ranges):
        self.range_min = range_min
        self.range_max = range_max
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.ranges = ranges

# ---------------------------- Core Logic ----------------------------

def filter_obstacles(obstacles_in):
    return [o for o in obstacles_in if abs(o.angle) <= g_fovAngleMax and o.distance <= kMaxRange]

def find_gaps_angle(obstacles):
    gaps = []
    if not obstacles:
        return gaps
    obstacles.sort(key=lambda o: o.angle)
    for i in range(1, len(obstacles)):
        left_obs = obstacles[i - 1]
        right_obs = obstacles[i]
        if not left_obs.overlaps(right_obs):
            gap = Gap(left_obs, right_obs)
            if gap.gap_size > kMinGapAngle and gap.is_wide_enough():
                gaps.append(gap)
    return gaps

def calculate_gap_center_angle_basic(gap):
    return 0.5 * (gap.obstacle_left.angle + gap.obstacle_right.angle)

def calculate_final_heading_angle(theta_goal, theta_c, d_min, alpha):
    d_min = max(d_min, 0.001)
    return ((alpha / d_min) * theta_c + theta_goal) / ((alpha / d_min) + 1.0)

def find_nearest_obstacle(obstacles):
    return min(obstacles, key=lambda o: o.distance)

def follow_the_gap_method(obstacles, lidar_data):
    gaps = find_gaps_angle(obstacles)
    if not gaps:
        # Fallback: outermost gap
        leftmost = max(obstacles, key=lambda o: o.angle)
        rightmost = min(obstacles, key=lambda o: o.angle)
        fallback_gap = Gap(leftmost, rightmost)
        if fallback_gap.gap_size > math.radians(5.0) and fallback_gap.is_wide_enough():
            gap_center_angle = calculate_gap_center_angle_basic(fallback_gap)
            d_min = find_nearest_obstacle(obstacles).distance
            return calculate_final_heading_angle(g_goal_angle, gap_center_angle, d_min, kGapWeightCoefficient)
        else:
            raise NoGapFoundException("No valid wide-enough gaps")
    largest_gap = max(gaps, key=lambda g: g.gap_size)
    gap_center_angle = calculate_gap_center_angle_basic(largest_gap)
    d_min = find_nearest_obstacle(obstacles).distance
    return calculate_final_heading_angle(g_goal_angle, gap_center_angle, d_min, kGapWeightCoefficient)

def run_callback(obstacles_in, lidar_data):
    obs = filter_obstacles(obstacles_in)
    if not obs:
        raise NoGapFoundException("No valid obstacles after filtering")
    return follow_the_gap_method(obs, lidar_data)

# ---------------------------- ROS 2 Node ----------------------------

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info("🚗 FollowTheGapNode initialized.")

    def scan_callback(self, msg: LaserScan):
        angle = msg.angle_min
        obstacles = []
        for r in msg.ranges:
            if r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue
            obstacles.append(Obstacle(r, angle))
            angle += msg.angle_increment

        lidar_data = LidarData(msg.range_min, msg.range_max, msg.angle_min,
                               msg.angle_max, msg.angle_increment, msg.ranges)

        try:
            final_angle = run_callback(obstacles, lidar_data)

            # Dynamic speed adjustment based on steering angle
            speed_factor = max(0.0, 1.0 - STEERING_SCALE * abs(final_angle))
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * speed_factor
            speed = min(max(speed, MIN_SPEED), MAX_SPEED)

            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = final_angle
            drive_msg.drive.speed = speed
            self.drive_pub.publish(drive_msg)

            self.get_logger().info(f"Steering: {final_angle:.3f} rad | Speed: {speed:.2f} m/s")
        except NoGapFoundException as e:
            self.get_logger().warn(f"No gap found: {e}")
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)

# ---------------------------- Main Entry ----------------------------

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
