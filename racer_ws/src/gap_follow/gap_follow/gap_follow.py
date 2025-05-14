#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from argparse import Namespace


class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')
        
        # Parameters
        self.declare_parameter('bubble_radius', 30)
        self.declare_parameter('preprocess_conv_size', 3)
        self.declare_parameter('max_lidar_dist', 6.0)
        self.declare_parameter('safe_threshold', 15)
        self.declare_parameter('best_point_conv_size', 400)
        self.declare_parameter('max_steer', 0.6981) # 40 degree
        self.declare_parameter('straights_steering_angle', 0.1396)  # 8 degree
        self.declare_parameter('fast_steering_angle', 0.0698)   # 4 degree
        self.declare_parameter('corners_speed', 1.5)
        self.declare_parameter('straights_speed', 3.0)
        self.declare_parameter('fast_speed', 5.0)
        
        # Load parameters
        self.params = self.get_parameters_as_namespace()
        
        # Publishers and subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
        
        self.get_logger().info('Follow the Gap node has been initialized')

    def get_parameters_as_namespace(self):
        """Convert ROS 2 parameters to a Namespace object similar to the example code"""
        params_dict = {
            'bubble_radius': self.get_parameter('bubble_radius').value,
            'preprocess_conv_size': self.get_parameter('preprocess_conv_size').value,
            'max_lidar_dist': self.get_parameter('max_lidar_dist').value,
            'safe_threshold': self.get_parameter('safe_threshold').value,
            'best_point_conv_size': self.get_parameter('best_point_conv_size').value,
            'max_steer': self.get_parameter('max_steer').value,
            'straights_steering_angle': self.get_parameter('straights_steering_angle').value,
            'fast_steering_angle': self.get_parameter('fast_steering_angle').value,
            'corners_speed': self.get_parameter('corners_speed').value,
            'straights_speed': self.get_parameter('straights_speed').value,
            'fast_speed': self.get_parameter('fast_speed').value,
        }
        return Namespace(**params_dict)

    def scan_callback(self, scan_msg):
        """Process LiDAR scan and publish drive commands"""
        # Convert scan to numpy array
        ranges = np.array(scan_msg.ranges)
        
        # Replace inf values with max_lidar_dist
        ranges = np.clip(ranges, 0, self.params.max_lidar_dist)
        
        # Process scan data
        proc_ranges = self.preprocess_lidar(ranges)
        
        # Find closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.params.bubble_radius
        max_index = closest + self.params.bubble_radius
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        
        # If no valid gap found, try to find any non-zero points
        if gap_start is None or gap_end is None:
            non_zero_indices = np.nonzero(proc_ranges)[0]
            if len(non_zero_indices) > 0:
                # If there are some non-zero points, pick the farthest one
                best_idx = np.argmax(proc_ranges)
            else:
                # If everything is zero, just go straight
                best_idx = len(proc_ranges) // 2
        else:
            # Find the best point in the gap
            best_idx = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Calculate steering angle
        steering_angle = self.get_angle(best_idx, len(proc_ranges))
        
        # Determine speed based on steering angle
        if abs(steering_angle) > self.params.straights_steering_angle:
            speed = self.params.corners_speed
        elif abs(steering_angle) > self.params.fast_steering_angle:
            speed = self.params.straights_speed
        else:
            speed = self.params.fast_speed
        
        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        
        self.drive_pub.publish(drive_msg)
        
        self.get_logger().debug(f"Published: steering={steering_angle:.2f}, speed={speed:.2f}")

    def preprocess_lidar(self, ranges):
        """
        Preprocess the LiDAR scan array:
        1. Use only the forward-facing scan points
        2. Set each value to the mean over some window
        3. Reject high values
        """
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        
        # Use only forward-facing points (similar to original code)
        proc_ranges = np.array(ranges[135:-135])
        
        # Apply moving average
        proc_ranges = np.convolve(proc_ranges, 
                                 np.ones(self.params.preprocess_conv_size), 
                                 'same') / self.params.preprocess_conv_size
        
        # Clip values to max distance
        proc_ranges = np.clip(proc_ranges, 0, self.params.max_lidar_dist)
        
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """
        Return the start index & end index of the max gap in free_space_ranges
        free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """
        # Mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        
        # Get slices for each contiguous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        
        if slices is None or len(slices) == 0:
            return None, None
        
        # Find longest slice that's above the safety threshold
        max_len = 0
        chosen_slice = None
        
        for sl in slices:
            sl_len = sl.stop - sl.start
            if sl_len > max_len and sl_len > self.params.safe_threshold:
                max_len = sl_len
                chosen_slice = sl
        
        if chosen_slice is not None:
            return chosen_slice.start, chosen_slice.stop
        elif len(slices) > 0:
            # If no slice meets the threshold, return the largest one
            max_slice = max(slices, key=lambda s: s.stop - s.start)
            return max_slice.start, max_slice.stop
        else:
            return None, None

    def find_best_point(self, start_i, end_i, ranges):
        """
        Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Uses sliding window average over the data in the max gap
        """
        # Handle edge case where the gap is too small for convolution
        if end_i - start_i < self.params.best_point_conv_size:
            # Just return the middle of the gap
            return start_i + (end_i - start_i) // 2
        
        # Do a sliding window average over the data in the max gap
        averaged_max_gap = np.convolve(
            ranges[start_i:end_i],
            np.ones(self.params.best_point_conv_size), 
            'same') / self.params.best_point_conv_size
        
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """
        Get the angle of a particular element in the lidar data and 
        transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        
        # Clip the steering angle to the maximum steering angle
        steering_angle = np.clip(steering_angle, -self.params.max_steer, self.params.max_steer)
        
        return steering_angle


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()