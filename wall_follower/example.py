import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import numpy as np

# Import the visualization tools from the provided file.
from wall_follower.visualization_tools import VisualizationTools

class ExampleNode(Node):

    def __init__(self):
        super().__init__('example_node')

        # HIGH LEVEL TOPIC - you may switch between nav_0, nav_1, or nav_2.
        topic_name = "/vesc/high_level/input/nav_2"
        self.get_logger().info(f"Using topic {topic_name}")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, topic_name, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Publisher for visualization markers (for displaying laser points in RViz)
        self.wall_marker_pub = self.create_publisher(Marker, "/front_points", 1)

        # Subscribe to the laser scan topic
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

    def timer_callback(self):
        """
        Callback to publish drive commands.
        """
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Example drive command: go straight
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 2.0
        self.publisher_.publish(msg)

    def laser_callback(self, scan_msg: LaserScan):
        """
        Process the LaserScan message using NumPy:
          1. Convert ranges to a numpy array.
          2. Create an array of corresponding angles.
          3. Select only the points within a wedge from -20° to 20°.
          4. Convert these polar coordinates to Cartesian (x, y).
          5. Publish these points via the visualization tool.
        """
        # Convert ranges and generate corresponding angles
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Define the wedge: only keep points between -20° and 20° (converted to radians)
        wedge_mask = (angles >= np.deg2rad(-20)) & (angles <= np.deg2rad(20))
        valid_mask = np.isfinite(ranges)
        use_mask = wedge_mask & valid_mask

        wedge_ranges = ranges[use_mask]
        wedge_angles = angles[use_mask]

        # Convert polar to Cartesian coordinates (robot frame: x is forward)
        x_points = wedge_ranges * np.cos(wedge_angles)
        y_points = wedge_ranges * np.sin(wedge_angles)

        # Publish the visualization if there are points in the wedge
        VisualizationTools.plot_line(x_points, y_points, self.wall_marker_pub,
                                         color=(0.0, 0.0, 1.0), frame="/base_link")

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

