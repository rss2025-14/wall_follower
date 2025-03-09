#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/input/navigation")
        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.0)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)
  
        # TODO: Initialize your publishers and subscribers here
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.DRIVE_TOPIC,
            10
        )

        # PD Parameters
        self.Kp = 0.3 # 0.3
        self.Kd = 0.21 # 0.21
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # Thresholds for corner detection
        self.max_slope_threshold = 5 
        self.max_std_dev_threshold = 0.2 

        # OPTIONAL: A publisher for debugging lines in RViz
        self.wall_marker_pub = self.create_publisher(Marker, "/wall_points", 1) # Plots the wall points being followed

        self.get_logger().info("Wall Follower node initialized.")

    def scan_callback(self, scan_msg: LaserScan):
        """
        - We follow a wall on one side (left or right).
        - If the robot is too close to the wall, we back up and steer away.
        - Otherwise, we do normal PD wall-following plus corner handling.
        """

        # 1. Convert LaserScan to arrays
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # 2. Side wedge
        if self.SIDE == 1:  # left
            side_min_angle = np.deg2rad(30)
            side_max_angle = np.deg2rad(170)
        else:               # right
            side_min_angle = np.deg2rad(-170)
            side_max_angle = np.deg2rad(-30)

        side_mask = (angles >= side_min_angle) & (angles <= side_max_angle)
        valid_mask = np.isfinite(ranges)
        distance_mask = (ranges < 6.0 * self.DESIRED_DISTANCE)
        use_mask = side_mask & valid_mask & distance_mask

        side_ranges = ranges[use_mask]
        side_angles = angles[use_mask]

        if len(side_ranges) < 1:
            self.publish_drive(0.0, 0.0)
            return

        # Remove outliers
        r_mean = np.mean(side_ranges)
        r_std = np.std(side_ranges)
        inlier_mask = np.abs(side_ranges - r_mean) < 1.5 * r_std
        side_ranges = side_ranges[inlier_mask]
        side_angles = side_angles[inlier_mask]

        if len(side_ranges) < 1:
            self.publish_drive(0.0, 0.0)
            return

        # Convert to (x,y)
        x_vals = side_ranges * np.cos(side_angles)
        y_vals = side_ranges * np.sin(side_angles)

        # Fit line => get distance, slope, std_dev
        wall_dist, slope_side, std_side = self.fit_line_and_compute_distance(x_vals, y_vals)

        # 3. If too close => BACK UP & STEER AWAY
        TOO_CLOSE_THRESHOLD = 0.6 * self.DESIRED_DISTANCE
        if wall_dist < TOO_CLOSE_THRESHOLD:
            # Robot is dangerously close to the wall
            reverse_speed = -2.0   # negative => backing up
            # Steer away from the wall:
            #   If SIDE=1 (left), we want negative steering => turn right
            #   If SIDE=-1 (right), we want positive steering => turn left
            reverse_steer = self.SIDE * 0.3

            # Publish
            max_steering = 0.34
            reverse_steer = max(-max_steering, min(max_steering, reverse_steer))

            self.publish_drive(reverse_speed, reverse_steer)
            self.get_logger().info("TOO CLOSE => backing up and steering away!")
            return

        # 4. (Optional) Corner wedge logic
        if self.SIDE == 1:  # left
            corner_min_angle = 0.0
            corner_max_angle = np.deg2rad(30)
        else:               # right
            corner_min_angle = np.deg2rad(-30)
            corner_max_angle = 0.0

        corner_mask = (angles >= corner_min_angle) & (angles <= corner_max_angle) & np.isfinite(ranges)
        corner_ranges = ranges[corner_mask]
        corner_dist = float('inf')
        slope_corner = slope_side

        if len(corner_ranges) > 3:
            corner_dist = np.mean(corner_ranges)
            x_corner = corner_ranges * np.cos(angles[corner_mask])
            y_corner = corner_ranges * np.sin(angles[corner_mask])
            _, slope_corner, _ = self.fit_line_and_compute_distance(x_corner, y_corner)
        else:
            corner_dist = float('inf')

        corner_threshold = 1.3 * self.DESIRED_DISTANCE
        slope_diff = abs(slope_side - slope_corner)
        corner_detected = (corner_dist < corner_threshold) or (slope_diff > 2.0)

        # 5. Normal PD control if not backing up
        error = self.DESIRED_DISTANCE - wall_dist
        now_time = self.get_clock().now()
        dt = (now_time - self.prev_time).nanoseconds * 1e-9
        if dt == 0:
            dt = 1e-6
        derivative = (error - self.prev_error) / dt
        steering_angle = -self.SIDE * (self.Kp * error + self.Kd * derivative)
        self.prev_error = error
        self.prev_time = now_time

        max_steering = 0.34
        steering_angle = max(-max_steering, min(max_steering, steering_angle))

        speed_cmd = self.VELOCITY

        if corner_detected:
            # If corner => slow down + add small turn away
            speed_cmd = 0.75 * self.VELOCITY
            corner_turn_bias = 0.2
            steering_angle += -self.SIDE * corner_turn_bias
            steering_angle = max(-max_steering, min(max_steering, steering_angle))
            self.get_logger().info("Corner detected => slowing + turning away")

        # 6. Publish final drive
        self.publish_drive(speed_cmd, steering_angle)
        self.get_logger().info(f"Steering angle: {steering_angle:.3f}, Speed: {speed_cmd:.3f}")

        # Visualization
        VisualizationTools.plot_line(x_vals, y_vals, self.wall_marker_pub,
                                    color=(1.0, 0.0, 0.0), frame="/base_link")

    
    def fit_line_and_compute_distance(self, x, y):
        """
        Fits y = m*x + b by least squares and returns:
          distance: the perpendicular distance from the robot base_link origin (0,0) to that line,
          slope (m), 
          intercept (b),
          std_dev: standard deviation of residuals (for corner detection heuristics)
        """
        # Case of no data or insufficient data
        N = len(x)
        if N < 2:
            return 0.0, 0.0, 0.0, 999.0  # no meaningful fit; set std_dev large

        # Compute slope m and intercept b via standard formulas
        x_mean = np.mean(x)
        y_mean = np.mean(y)

        denom = np.sum((x - x_mean)**2)
        if abs(denom) < 1e-9:
            # This would mean all x are nearly the same => near-vertical line
            # Return something big slope, so corner detection triggers
            return 0.0, 999.0, 0.0, 999.0

        m = np.sum((x - x_mean)*(y - y_mean)) / denom
        b = y_mean - m*x_mean

        # Distance from origin to line y=mx+b is:
        #    dist = |b| / sqrt(m^2 + 1)
        # The sign of 'b' indicates whether the line is above or below the origin
        dist = abs(b) / np.sqrt(m*m + 1.0)

        # Residuals => measure how well the points fit
        y_pred = m*x + b
        residuals = y - y_pred
        std_dev = np.std(residuals)

        return dist, m, std_dev

    def publish_drive(self, speed, steering_angle):
        """
        Helper function to publish to the Ackermann drive topic.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(drive_msg)

    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    