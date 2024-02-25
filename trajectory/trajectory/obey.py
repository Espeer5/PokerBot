"""
This file contains the code for the Control node, which is used to force the robot 
to obey the brain node. The brain node is primary planner for the Poker Robot,
and the obey node is the primary controller which translates commands from the
brain to the hardware.
"""

import numpy as np
import rclpy

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from utils.constants import CONTROL_NODE, GET_CHAIN, nan
from trajectory.spline_q import JointSplineQueue


class Trajectory():
    """
    This class is used to generate trajectories for the Poker Robot based on the
    goal commands given to the Controller node from the brain node. The
    trajectory class interacts with the control node in 2 ways. The control node
    receives messages from the brain and passes them to the trajectory class by 
    adding them to the goal queue. The trajectory class then generates the
    aproppriate trajectory when the control node requests the next joint
    positions and velocities on each time step through the evaluate method.
    """
    def __init__(self, node, q0):
        """
        Initialize the trajectory class with the initial joint positions of the
        robot.
        """
        self.node = node
        self.chain = GET_CHAIN(node)
        self.q0 = np.array(q0).reshape(5, 1)
        self.queue = JointSplineQueue()
        self.goal_joints = None
        self.q = q0
        self.suction = True
        self.pubbed = False

    def evaluate(self, t, _):
        """
        Return the appropriate joint positions and velocities at the given time
        based on the trajectories in the queue.
        """
        if t < 4.0:
            self.queue.t0 = t
            self.q = np.array(self.node.actpos).reshape(5, 1)
            return [nan, nan, nan, nan, nan], [nan, nan, nan, nan, nan]
        q, qdot = self.queue.evaluate(t)
        if q is not None:
            self.q = q
            return q.flatten().tolist(), qdot.flatten().tolist()
        # If no queued trajectory spline, simply remain in place
        else:
            # Notify the detector to look for a new card
            if not self.pubbed:
                self.node.get_logger().info("sending detect")
                self.node.detect_pub.publish(String(data="detect"))
                self.pubbed = True
            return self.q.flatten().tolist(), [0.0, 0.0, 0.0, 0.0, 0.0]
    

class ControlNode(CONTROL_NODE):
    """
    The robot controller node which receives goal commands from the brain node 
    and sends joint commands to the robot hardware. The control node uses the 
    trajectory class to generate the appropriate intermediary joint positions
    and velocities to navigate from the current positions to the goal positions
    """
    def __init__(self, name):
        """
        Initialize the control node.
        """
        super().__init__(name)
        self.get_logger().info("ControlNode()")
        self.traj = Trajectory(self, self.actpos)

        # Create a subscriber for goal messages, but do not begin listeining yet
        self.goal_sub = self.create_subscription(JointState, '/goal', self.cb_goal,
                                                 10)
        self.detect_pub = self.create_publisher(String, '/detect_now', 3)
        # Wait for the detector to sub to the detect_now topic
        while self.detect_pub.get_subscription_count() == 0:
            pass
    
    def cb_goal(self, msg):
        """
        Callback function for receiving goal messages from the brain node. The
        goal messages are added to the queue of goal commands in the trajectory
        class.
        """
        self.get_logger().info("Received goal message")
        joint_pos = np.array(msg.position).reshape(5, 1)
        joint_vel = np.array(msg.velocity).reshape(5, 1)
        # Slightly hacky way to do this, but oh well
        T = msg.effort[0]
        self.traj.pubbed = False
        
        if self.traj.queue.empty():
            self.traj.queue.enqueue(np.array(self.actpos).reshape(5, 1),
                                    joint_pos, np.zeros((5, 1)), joint_vel, T,
                                    endAction=msg.name[0])
            self.traj.queue.t0 = self.t
        else:
            prev_goal = self.traj.queue.peek_back()
            self.traj.queue.enqueue(prev_goal.qf, joint_pos, prev_goal.qdotf,
                                    joint_vel, T, endAction=msg.name[0])


def main(args=None):
    """
    Run the control node.
    """
    # Initialize ROS.
    rclpy.init(args=args)

    # Create the control node.
    node = ControlNode('controller')
    node.get_logger().info("Control node is running")

    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Clean up the node when it is destroyed.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
