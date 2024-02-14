"""
Demo code of a very simple grab and release sequence of an object at a pre-defined
position. The robot will grab the object, wait for 5 seconds, and release the
object.
"""

import numpy as np
import rclpy
import math
from pb_utils.pump_util import send_pwm
from point.TrajectoryUtils    import *
from point.KinematicChain     import *

from rclpy.node         import Node
from sensor_msgs.msg    import JointState

# Constants
RATE = 100.0 # Hz
jointnames = ["base", "shoulder", "elbow", "tip"]


class Trajectory():
    def __init__(self, demo_node, q0):
        self.chain = KinematicChain(demo_node, "world", "cup", jointnames)
        self.q0 = np.array(q0).reshape(4, 1)
        self.demo = demo_node
        self.q = self.q0
        self.waitQ = np.array([-np.pi/2, np.pi/2, np.pi/2, 0]).reshape(4, 1)
        self.target = np.array([-1.53, 0.74, 1.95, -0.4]).reshape(4, 1)
        self.suckin = False

    def evaluate(self, t, dt):
        """
        Evaluate the trajectory at time t
        """
        if t < 10:
            q, qdot = goto5(t, 10, self.q0, self.waitQ)
            self.q = q
            return q.flatten().tolist(), qdot.flatten().tolist()
        elif t < 20:
            q, qdot = goto5(t-10, 10, self.waitQ, self.target)
            self.q = q
            return q.flatten().tolist(), qdot.flatten().tolist()
        elif t < 22:
            if not self.suckin:
                send_pwm(220)
                self.suckin = True
            return self.target.flatten().tolist(), np.zeros((4, 1)).flatten().tolist()
        elif t < 32:
            q, qdot = goto5(t-22, 10, self.target, self.waitQ)
            self.q = q
            return q.flatten().tolist(), qdot.flatten().tolist()
        elif t < 42:
            q, qdot = goto5(t-32, 10, self.waitQ, self.q0)
            self.q = q
            return q.flatten().tolist(), qdot.flatten().tolist()
        elif t < 44:
            if self.suckin:
                send_pwm(0)
                self.suckin = False
            return self.q0.flatten().tolist(), np.zeros((4, 1)).flatten().tolist()
        elif t < 54:
            q, qdot = goto5(t-44, 10, self.q0, self.waitQ)
            self.q = q
            return q.flatten().tolist(), qdot.flatten().tolist()
        else:
            return self.demo.actpos, np.zeros((4, 1)).flatten().tolist()

class DemoNode(Node):
    """
    Defines the ROS node which runs the periodic point demo
    """

    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Gravity constants
        self.A_el = 0.0 # sin
        self.B_el = -1.6 # cos
        self.A_sh = 0.5
        self.B_sh = 5.0

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        self.actpos = None
        self.statessub = self.create_subscription(JointState, '/joint_states', 
                                                  self.cb_states, 1)
        while self.actpos is None:
            rclpy.spin_once(self)
            self.get_logger().info("Initial positions: %r" % self.actpos)

        # Trajectory object
        self.trajectory = Trajectory(self, self.actpos)

        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.timer = self.create_timer(1/rate, self.sendcmd)
        self.dt        = self.timer.timer_period_ns * 1e-9
        self.t         = - self.dt
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                            (self.timer.timer_period_ns * 1e-9, rate))
        
    def cb_states(self, msg):
        # Save the actual position.
        self.actpos = msg.position
        
    def gravity(self, pos):
        tau_elbow = self.A_el * math.sin(-pos[1] + pos[2]) + self.B_el * math.cos(-pos[1] + pos[2])
        tau_shoulder = -tau_elbow + self.A_sh * math.sin(pos[1]) + self.B_sh * math.cos(pos[1])
        return (0.0, tau_shoulder, tau_elbow, 0.0)

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    def sendcmd(self):

        # To avoid any time jitter enforce a constant time step in
        # integrate to get the current time.
        self.t += self.dt

        # Compute the desired joint positions and velocities for this time.
        q, qdot = self.trajectory.evaluate(self.t, self.dt)
        # Build up the message and publish.
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['base', 'shoulder', 'elbow', 'tip']
        self.cmdmsg.position     = q
        self.cmdmsg.velocity     = qdot
        self.cmdmsg.effort       = self.gravity(self.actpos)
        self.cmdpub.publish(self.cmdmsg)

def main(args=None):
    """
    Send gravity compensating torques to the robot.
    """
        # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = DemoNode('demo')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
