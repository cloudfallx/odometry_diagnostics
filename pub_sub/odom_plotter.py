#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import math

class OdomDualFramePlotter(Node):
    def __init__(self):
        super().__init__('odom_dual_frame_plotter')
        self.window_size = 10.0
        self.time_data = deque()
        self.euclid_odom = deque()
        self.euclid_map = deque()

        self.odom_frame = "odom"
        self.map_frame = "map"
        self.base_frame = "base_footprint"

        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], "r-", label="Distance from odom origin")
        self.line2, = self.ax.plot([], [], "b-", label="Distance from map origin")
        self.ax.set_xlabel("ROS Time [s]")
        self.ax.set_ylabel("Euclidean Distance [m]")
        self.ax.set_title("Euclidean Distance: odom vs map Frame")
        self.ax.legend()
        self.ax.grid(True)
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=200)
        self.time_zero = None

    def odom_callback(self, msg):
        time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.time_zero is None:
            self.time_zero = time_sec
        rel_time = time_sec - self.time_zero

        # 1. Distance from odom origin (local, no TF)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        d_odom = math.sqrt(x ** 2 + y ** 2)

        # 2. Distance from map frame (TF lookup)
        d_map = float('nan')
        try:
            from rclpy.time import Time
            msg_time = Time(seconds=time_sec)
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                msg_time
            )
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            d_map = math.sqrt(tx ** 2 + ty ** 2)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed (map): {str(e)}")

        self.time_data.append(rel_time)
        self.euclid_odom.append(d_odom)
        self.euclid_map.append(d_map)

        # Sliding window
        while self.time_data and (rel_time - self.time_data[0]) > self.window_size:
            self.time_data.popleft()
            self.euclid_odom.popleft()
            self.euclid_map.popleft()

    def update_plot(self, frame):
        self.line1.set_data(self.time_data, self.euclid_odom)
        # Only plot valid map TF data
        map_y = [v for v in self.euclid_map if not math.isnan(v)]
        map_x = [self.time_data[i] for i, v in enumerate(self.euclid_map) if not math.isnan(v)]
        self.line2.set_data(map_x, map_y)
        if self.time_data:
            self.ax.set_xlim(max(0, self.time_data[-1] - self.window_size),
                             self.time_data[-1] + 0.1)
        combined = list(self.euclid_odom) + map_y
        if combined:
            self.ax.set_ylim(min(combined) - 0.5, max(combined) + 0.5)
        return self.line1, self.line2

def main(args=None):
    rclpy.init(args=args)
    plotter = OdomDualFramePlotter()
    try:
        plt.ion()
        plt.show()
        while rclpy.ok():
            rclpy.spin_once(plotter, timeout_sec=0.1)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
