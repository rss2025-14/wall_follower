import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import numpy as np

# Import the visualization tools from the provided file.
from wall_follower.visualization_tools import VisualizationTools

class SafetyController(Node):

    def __init__(self):
        super().__init__('safety_controller')

        # Initialize the last received drive command (updated via ackermann callback)
        self.last_cmd = None

        # Initialize default velocity (will be updated via ackermann command subscription)
        self.velocity = 1.0

        # Safety flag that indicates whether to stop the car.
        self.should_stop = False

        # Publisher for safety commands on the low-level safety topic.
        safety_topic = "/vesc/low_level/input/safety"
        self.get_logger().info(f"Publishing safety commands to {safety_topic}")
        self.safety_publisher = self.create_publisher(AckermannDriveStamped, safety_topic, 10)

        # Publisher for visualization markers (for displaying laser points in RViz)
        self.wall_marker_pub = self.create_publisher(Marker, "/front_points", 1)

        # Subscribe to the low-level ackermann command to update the drive command and velocity.
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            '/vesc/low_level/ackermann_cmd',
            self.ackermann_callback,
            10)
        # Subscribe to the laser scan topic.
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

    def ackermann_callback(self, cmd_msg: AckermannDriveStamped):
        """
        Update the last received drive command and extract the current velocity.
        """
        self.last_cmd = cmd_msg
        self.velocity = cmd_msg.drive.speed
        self.get_logger().debug(f"Received ackermann command with speed: {self.velocity:.2f}")

    def laser_callback(self, scan_msg: LaserScan):
        """
        Process the LaserScan message, perform the safety check, and publish the safety command:
          1. Convert ranges to a numpy array and generate corresponding angles.
          2. Filter points within a wedge from -20° to 20°.
          3. Visualize these points.
          4. Determine if any point is closer than the safety distance.
          5. Publish the appropriate drive command.
        """
        # Convert ranges and generate corresponding angles.
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Define the wedge: only keep points between -20° and 20°.
        wedge_mask = (angles >= np.deg2rad(-20)) & (angles <= np.deg2rad(20))
        valid_mask = np.isfinite(ranges)
        use_mask = wedge_mask & valid_mask

        wedge_ranges = ranges[use_mask]
        wedge_angles = angles[use_mask]

        # Convert polar coordinates to Cartesian (robot frame: x is forward).
        x_points = wedge_ranges * np.cos(wedge_angles)
        y_points = wedge_ranges * np.sin(wedge_angles)

        # Publish the visualization (for RViz).
        VisualizationTools.plot_line(x_points, y_points, self.wall_marker_pub,
                                       color=(0.0, 0.0, 1.0), frame="/base_link")

        # Compute safety distance: always at least 1.0, or 1.0 * velocity.
        safety_distance = max(1.0, 1.0 * self.velocity)

        # Check if any measured point is closer than the safety distance.
        if wedge_ranges.size > 0:
            min_range = np.min(wedge_ranges)
            if min_range < safety_distance:
                self.should_stop = True
                self.get_logger().info(
                    f"Object detected at {min_range:.2f}m (< safety {safety_distance:.2f}m): Stopping!")
            else:
                self.should_stop = False
        else:
            self.should_stop = False

        # Publish the safety command based on the laser scan check.
        if self.last_cmd is None:
            # If no command has been received yet, nothing to publish.
            return

        if self.should_stop:
            # Override speed to zero but maintain the last turning command.
            safety_cmd = AckermannDriveStamped()
            safety_cmd.header.stamp = self.get_clock().now().to_msg()
            safety_cmd.drive.speed = 0.0
            safety_cmd.drive.steering_angle = self.last_cmd.drive.steering_angle
            self.get_logger().info("SafetyController: Stopping vehicle due to obstacle.")
            self.safety_publisher.publish(safety_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

