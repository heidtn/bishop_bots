
from numpy.core.numeric import _roll_dispatcher
import rclpy
from rclpy import executors
import message_filters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Wrench, PoseStamped, Pose
from bishop_messages.action import FollowPath
from rclpy.action import ActionClient, server
from collections import OrderedDict
import numpy as np

NBOTS = 10

class BotPositionManager(Node):
    def __init__(self):
        super().__init__('bot_position_manager')
        self.bot_positions = OrderedDict()
        self.bot_position_subscriptions = OrderedDict()
        self.get_logger().info("Creating bot position manager")
        for i in range(NBOTS):
            sub = message_filters.Subscriber(
                self,
                Odometry,
                f"/bot_{i}/odom",
                )
            self.bot_position_subscriptions[f"bot_{i}"] = sub
        ts = message_filters.ApproximateTimeSynchronizer(self.bot_position_subscriptions.values(), 10, 1.0)
        ts.registerCallback(self.position_callback)
        self.get_logger().info("started bot position callback")
        self.positions = []
        self.new_position = False

    def position_callback(self, *msg):
        self.new_position = True
        self.positions = [m.pose.pose for m in msg]

    def wait_for_positions(self):
        self.new_position = False
        while not self.new_position:
            pass
        return list(self.positions)

class BotManagerTest(Node):
    """
    Attitude determination and control system node.  Can control in velocity (twist) or position mode by
    sending wrench commands
    """

    def __init__(self, position_manager: BotPositionManager):
        super().__init__('minimal_subscriber')
        self.position_manager = position_manager

        self.bot_action_clients = {}
        for i in range(NBOTS):
            action_client = ActionClient(
                self, FollowPath, f'/bot_{i}/follow_path')
            self.bot_action_clients[f"bot_{i}"] = action_client

        self.timer = self.create_timer(1.0, self.start)

    def start(self):
        self.timer.reset()
        positions = self.position_manager.wait_for_positions()
        self.get_logger().info(f"Got new positions {positions}")

    def send_goal(self):
        goal_msg = FollowPath.Goal()
        curtime = self.get_clock().now()
        goal_msg.path.header.stamp = curtime.to_msg()
        goal_msg.path.header.frame_id = 'world'
        points = [np.array([0., 0., 15.0]),
                  np.array([0., 5., 15.0]),
                  np.array([5., 5., 15.0]),
                  np.array([5., 0., 15.0])]

        time_delta = 5.0
        for point in points:
            dur = Duration(seconds=time_delta)
            next_time = dur + curtime
            curtime = next_time
            pose = PoseStamped()
            pose.header.stamp = curtime.to_msg()
            pose.header.frame_id = 'world'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            goal_msg.path.poses.append(pose)

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    bot_position_manager = BotPositionManager()
    bot_manager = BotManagerTest(bot_position_manager)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bot_manager)
    executor.add_node(bot_position_manager)
    executor.spin()
    executor.shutdown()


if __name__ == '__main__':
    main()
