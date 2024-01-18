#!/usr/bin/env python3
"""
A simple demo of the 3DOF robot in ME/CS 134. Causes the arm to execute a simple 
periodic waving motion.
"""

import numpy as np
import rclpy

from rclpy.node         import Node
from sensor_msgs.msg    import JointState
from point.TrajectoryUtils    import *
from point.KinematicChain     import *
from point.newton_raphson import newton_raphson

# Constants
RATE = 100.0 # Hz

jointnames = ["base", "shoulder", "elbow"]


class Trajectory():
    """
    Computes the positions and velocities of each joint based on the current 
    time to be sent via the ROS node to the 3DOF robot.
    """

    def __init__(self, demo_node, q0):
        self.chain = KinematicChain(demo_node, "world", "tip", jointnames)

        self.q0 = np.array(q0).reshape(3, 1)
        # self.q0b = np.array([-1/2, -1/2, 1/2]).reshape(3,1)
        self.waitQ = np.array([np.pi/2, 0, -np.pi/2]).reshape(3, 1)
        self.p_wait, _, _, _ = self.chain.fkin(self.waitQ)
        self.lam = 20
        self.leftPos = np.array([-0.4, 0.4, 0.1]).reshape(3, 1)
        self.rightPos = np.array([0.4, 0.4, 0.0]).reshape(3, 1)
        self.q = self.q0

        #self.leftQ = newton_raphson(self.chain, self.leftPos, self.q0)
        #self.rightQ = newton_raphson(self.chain, self.rightPos, self.q0)

    def evaluate(self, t, dt):
        if t < 5:
            q, qdot = goto(t, 5, self.q0, np.array([self.q0[0], self.waitQ[1], self.q0[2]]).reshape(3, 1))
            self.q = q
        elif t < 10:
            q, qdot = goto(t - 5, 5, np.array([self.q0[0], self.waitQ[1], self.q0[2]]).reshape(3, 1), self.waitQ)
            self.q = q
        elif t < 15:
            pd, vd = goto(t - 10, 5, self.p_wait, self.leftPos)

            # Compute the old fkin
            p, _, Jv, _ = self.chain.fkin(self.q)
            print("p: ",p)
            # Compute the errors
            epd = ep(pd, p)

            # Compute each q_dot
            qdot = np.linalg.pinv(Jv) @ (vd + self.lam * epd)

            # Integrate to get the new q
            q = self.q + qdot * dt

            # Store the new q
            self.q = q
            print("q: ",q)
        
        else:
            q, qdot = self.q, np.zeros((3, 1))
            self.q = q

        return q.flatten().tolist(), qdot.flatten().tolist()
        

class DemoNode(Node):
    """
    Defines the ROS node which runs the periodic point demo
    """

    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.position0)

        # Create the Trajectory object
        self.trajectory = Trajectory(self, self.position0)

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.timer = self.create_timer(1/rate, self.sendcmd)
        self.dt        = self.timer.timer_period_ns * 1e-9
        self.t         = - self.dt
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    # Grab a single feedback - do not call this repeatedly.
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return self.grabpos

    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        # Just print the position (for now).
        #print(list(fbkmsg.position))
        pass

        # Send a command - called repeatedly by the timer.
    def sendcmd(self):

        # To avoid any time jitter enforce a constant time step in
        # integrate to get the current time.
        self.t += self.dt

        # Compute the desired joint positions and velocities for this time.
        q, qdot = self.trajectory.evaluate(self.t, self.dt)

        # Build up the message and publish.
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['base', 'shoulder', 'elbow']
        self.cmdmsg.position     = q
        self.cmdmsg.velocity     = qdot
        self.cmdmsg.effort       = [0.0, 0.0, 0.0]
        self.cmdpub.publish(self.cmdmsg)


def main(args=None):
    """
    Main method which instantiates the node and runs the tranjectory.
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
