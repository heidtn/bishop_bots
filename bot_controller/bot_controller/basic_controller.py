import rclpy
from rclpy import executors
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Wrench, PoseStamped
from bishop_messages.action import FollowPath
from rclpy.action import ActionServer

import numpy as np


class ADCSNode(Node):
    """
    Attitude determination and control system node.  Can control in velocity (twist) or position mode by
    sending wrench commands
    """

    def __init__(self):
        super().__init__('bot_controller_node')
        self.get_logger().info(f"Starting bot with namespace {self.get_namespace()}")
        message_group = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10,
            callback_group=message_group)

        self.effort_publisher = self.create_publisher(Wrench, 'gazebo_ros_force', 10, callback_group=message_group)
        server_group = MutuallyExclusiveCallbackGroup()
        self._path_server = ActionServer(self, FollowPath, "follow_path", self.follow_path_callback, callback_group=server_group)
        

        self.goal_position = None
        self.Kp = 0.1
        self.Kd = 0.05
        self.last_position = None
        self.last_time = None

    def follow_path_callback(self, goal_handle):
        path: Path = goal_handle.request.path
        start_point = path.poses[0]
        for point in path.poses[1:]:
            self.goto_point(start_point, point)
            start_point = point
        result = FollowPath.Result()
        result.completed = True
        return result

    def stamp_to_seconds(self, stamp):
        return stamp.sec + stamp.nanosec / 10e9

    def time_between_headers(self, header1: Header, header2: Header):
        t1 = self.stamp_to_seconds(header1.stamp)
        t2 = self.stamp_to_seconds(header2.stamp)
        return t2 - t1

    def goto_point(self, point1: PoseStamped, point2: PoseStamped):
        total_time = self.time_between_headers(point1.header, point2.header)
        while self.stamp_to_seconds(self.get_clock().now().to_msg()) < self.stamp_to_seconds(point2.header.stamp):
            curtime = self.get_clock().now()
            dummy_header = Header()
            dummy_header.stamp = curtime.to_msg()
            time_since_start = self.time_between_headers(point1.header, dummy_header)
            scale_factor = time_since_start / total_time
            p1 = point1.pose.position
            p1 = np.array([p1.x, p1.y, p1.z])
            p2 = point2.pose.position
            p2 = np.array([p2.x, p2.y, p2.z])
            self.goal_position = (p2 - p1)*scale_factor + p1

    def set_goal_position(self, goal):
        self.goal_position = goal

    def control(self, current_position, time):
        err = self.goal_position - current_position
        deriv = (self.last_position - current_position) / (time - self.last_time)

        self.last_position = current_position
        self.last_time = time
        effort = err*self.Kp + deriv*self.Kd

        msg = Wrench()
        msg.force.x = effort[0]
        msg.force.y = effort[1]
        msg.force.z = effort[2]
        self.effort_publisher.publish(msg)

    def listener_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        time = msg.header.stamp.sec + msg.header.stamp.nanosec/10e9
        pos = np.array([p.x, p.y, p.z])
        if self.goal_position is None:
            self.goal_position = pos 
            return
        if self.last_position is None:
            self.last_position = pos
            self.last_time = time
            return
        self.control(pos, time)


def main(args=None):
    rclpy.init(args=args)

    node = ADCSNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
