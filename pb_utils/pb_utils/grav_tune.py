"""
Code used to run simple gravity compensation in order to tune the poker bot 
gravity compensation parameters.
"""

import numpy as np
import rclpy
import math

from rclpy.node         import Node
from sensor_msgs.msg    import JointState

# Constants
RATE = 100.0 # Hz
jointnames = ["base", "shoulder", "elbow", "tip"]

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
        nan = float('nan')
        # Build up the message and publish.
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['base', 'shoulder', 'elbow', 'tip']
        self.cmdmsg.position     = (nan, nan, nan, nan)
        self.cmdmsg.velocity     = (nan, nan, nan, nan)
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
