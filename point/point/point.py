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
import math
from enum import Enum

# Constants
RATE = 100.0 # Hz
jointnames = ["base", "shoulder", "elbow"]


class States(Enum):
    INIT = 1
    WAIT = 2
    GOTO = 3
    RETURN = 4


def ikin(t, T, start, goal, prev_q, chain, lambd, dt):
    """
    Iterative ikin using kinematic chain
    """
    pd, vd = goto(t, T, start, goal)
    # Compute the old fkin
    p, _, Jv, _ = chain.fkin(prev_q)

    # Compute the errors
    epd = ep(pd, p)

    # Compute each q_dot
    JT =  np.transpose(Jv)
    gamma = 0.1
    J_winv = JT @ np.linalg.pinv((Jv @ JT + gamma**2 * np.eye(3)))
    qdot = J_winv @ (vd + lambd * epd)

    # Integrate to get the new q
    q = prev_q + qdot * dt

    return q, qdot


class Trajectory():
    """
    Computes the positions and velocities of each joint based on the current 
    time to be sent via the ROS node to the 3DOF robot.
    """

    def __init__(self, demo_node, q0):
        self.chain = KinematicChain(demo_node, "world", "tip", jointnames)

        self.q0 = np.array(q0).reshape(3, 1)
        self.waitQ = np.array([np.pi/2, 0, -np.pi/2]).reshape(3, 1)
        self.p_wait, _, _, _ = self.chain.fkin(self.waitQ)
        self.lam = 20
        self.state = States.INIT
        self.goals = []
        self.curr_goal = None
        self.start_time = 0
        self.q = self.q0

    def add_goal(self, goal):
        self.goals.append(goal)

    def evaluate(self, t, dt):
        match self.state:
            case States.INIT:
                if t < 5:
                    q, qdot = goto(t, 5, self.q0, 
                                   np.array([self.q0[0], 
                                             self.waitQ[1], 
                                             self.q0[2]]).reshape(3, 1))
                    self.q = q
                elif t < 10:
                    q, qdot = goto(t - 5, 5, 
                                   np.array([self.q0[0], 
                                             self.waitQ[1], 
                                             self.q0[2]]).reshape(3, 1), 
                                             self.waitQ)
                    self.q = q
                else:
                    q, qdot = self.q, np.zeros((3, 1))
                    self.state = States.WAIT
            case States.WAIT:
                q, qdot = self.q, np.zeros((3, 1))
                if not len(self.goals) == 0:
                    self.start_time = t
                    self.curr_goal = self.goals.pop()
                    self.state = States.GOTO
            case States.GOTO:
                q, qdot = ikin(t - self.start_time, 5, self.p_wait,
                               self.curr_goal, self.q, self.chain, self.lam, dt)
                # Store the new q
                self.q = q
                if t - self.start_time >= 5:
                    self.start_time = t
                    self.curr_goal = q
                    self.state = States.RETURN
            case States.RETURN:
                q, qdot = goto(t - self.start_time, 5, self.curr_goal, self.waitQ)
                # Store the new q
                self.q = q
                if t - self.start_time >= 5:
                    self.state = States.WAIT

            
        return q.flatten().tolist(), qdot.flatten().tolist()
        

class DemoNode(Node):
    """
    Defines the ROS node which runs the periodic point demo
    """

    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Gravity constants
        self.A = -1.8
        self.B = -0.25

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
        
        # Create a subscriber to receive point messages.
        self.fbksub = self.create_subscription(
            Point, '/point', self.recvpoint, 10)

        # Report.
        self.get_logger().info("Running point command receiver")

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
        tau_shoulder = self.A * math.sin(pos[1]) + self.B * math.cos(pos[1])
        return (0.0, tau_shoulder, 0.0)

    # Receive a point message - called by incoming messages.
    def recvpoint(self, pointmsg):
        # Extract the data.
        x = pointmsg.x
        y = pointmsg.y
        z = pointmsg.z

        self.trajectory.add_goal(np.array([x, y, z]).reshape(3, 1))
        
        # Report.
        self.get_logger().info("Running point %r, %r, %r" % (x,y,z))
    
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
        self.cmdmsg.effort       = self.gravity(self.actpos)
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
