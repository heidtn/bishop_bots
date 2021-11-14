
from numpy.core.numeric import _roll_dispatcher
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Wrench, PoseStamped, Pose
from bishop_messages.action import FollowPath
from rclpy.action import ActionClient

import numpy as np


class BotManagerTest(Node):
    """
    Attitude determination and control system node.  Can control in velocity (twist) or position mode by
    sending wrench commands
    """

    def __init__(self):
        super().__init__('minimal_subscriber')
        self._action_client = ActionClient(self, FollowPath, '/bot_0/follow_path')
    
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
    action_client = BotManagerTest()
    future = action_client.send_goal()
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()

