"""
This module contains all of the constants used in the PokerBot project, 
including both constant values and constant functions needed by all nodes in 
the project.
"""

import os
import math
import rclpy
import numpy as np
from ament_index_python.packages import get_package_share_directory as pkgdir

from sensor_msgs.msg import JointState
from rclpy.node import Node
from launch.actions import Shutdown
import launch_ros.actions

from utils.KinematicChain import KinematicChain
from utils.pump_util import send_pwm

# JOINTS AND MOTORS
# A list of the joint names as listed in the URDF file, and the corresponding
# list of motors which drive these joints. The motors are listed as IDs
# corresponding to the individual HEBI motors in the lab.
JOINTS = ["base", "shoulder", "elbow", "tip", "tipturn"]
MOTORS = ["8.3", "8.5", "8.4", "8.1", "8.2"]

# Get a kinematic chain corresponding to these joints:
GET_CHAIN = lambda node: KinematicChain(node, "world", "end", JOINTS)

# TRAJECTORY CONSTANTS
LAMBDA = 0.1 # Correction factor for position in simulated ikin
SIM_T = 10 # simulated time for finding joint angles (greater -> more precise)
FLIP_PHI = np.pi / 2 + 0.2 # The angle of the end affector to the table surface for card flip

# GRAVITY MODEL
# The gravity model is a simple sinusoidal model of the form:
#   tau = A*sin(q) + B*cos(q)
# where tau is the torque, q is the joint position, and A and B are constants.
# The arm needs resistive torques to counteract gravity on only the shoulder and
# elbow joints. The tip and tipturn joints are not affected greatly by gravity.

A_EL = 0.15 # Elbow sin coefficient
B_EL = -2.9 # Elbow cos coefficient
A_SH = 0.6 # Shoulder sin coefficient
B_SH = 7.0 # Shoulder cos coefficient

# ACTION MAP
# Maps strings of actions to functions which execute those actions on the vacuum 
# gripper.
ACTION_MAP = {
    "GB_CARD": lambda: send_pwm(200),
    "GB_CHIP": lambda: send_pwm(220),
    "DROP": lambda: send_pwm(0),
    "FLIP": lambda: send_pwm(0),
    "NONE": lambda: None
}

# NODE CONSTANTS

# All nodes in the PokerBot project run at the same rate of 100 Hz.
RATE = 100

# If no position or velocity is to be sent to the robot, should send nan values.
nan = float('nan')

# BASE CONTROLLER NODE
# Every robot control node in the project uses the same basic structure and
# shares a common set of publishers and receivers. This class may be used to 
# create a new node by simply inheriting from it and defining the additional 
# functionality on top of it
class CONTROL_NODE(Node):
    """
    A class which defines the basic structure of a robot control node. This 
    class is intended to be inherited from, and the inheriting class should
    define the additional functionality of the node.

    Attributes:
        cmdmsg: A JointState message which is used to send joint commands to the
                robot.
        cmdpub: A publisher which sends the joint commands to the robot.
        actpos: A list of the actual joint positions from the hardware.
        statessub: A subscriber to the joint states topic to receive the actual
                   joint positions from the hardware.
        timer: A timer to keep calculating/sending commands.
        dt: The time step of the control loop.
        t: The current time of the control loop.
    """
    def __init__(self, name):
        """
        Initialize a robot controller node.
        """
        # Intialize the node, naming it as specified
        super().__init__(name)

        # Create a message and publisher to send joint commands to the robot.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Create a subscriber to receive the actual joint positions from the
        # hardware.
        self.actpos = None
        self.statessub = self.create_subscription(JointState, '/joint_states', 
                                                  self.cb_states, 1)
        
        # Wait for the first message to be received, ensuring connection to the
        # hardware.
        while self.actpos is None:
            rclpy.spin_once(self)

        # Wait for the hardware to subscribe to the joint commands topic. (i.e,
        # ensure the robot is listening)
        while(not self.count_subscribers('/joint_commands')):
            pass

        # Create a timer to keep calculating/sending commands.
        self.timer = self.create_timer(1/RATE, self.sendcmd)
        self.dt = self.timer.timer_period_ns * 1e-9
        self.t = -self.dt
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, RATE))
    
    def cb_states(self, msg):
        """
        Callback funtion for the joint states subscriber. Simply saves the joint 
        positions from the hardware to be used in the control algorithm. May be
        overridden by the inheriting class if more advanced functionality is
        required.
        """
        self.actpos = msg.position

    def shutdown(self):
        """
        Shutdown the ROS node on exit from the program.
        """
        send_pwm(0)
        self.destroy_node()

    def grav_model(self):
        """
        Compute the gravity resisting torques for the shoulder and elbow motors
        of the PokerBot dependent on the last known joint positions. Should be used 
        to compute the torques sent to the PokerBot on each time step of the robot
        control node. The gravity model needs to be faded in at the start of the 
        control loop to avoid a step change in the torques send to the robot.
        """
        pre_factor = 1.0 if self.t > 4.0 else -math.exp(-2 * self.t) + 1
        tau_elbow = (A_EL * math.sin(-self.actpos[1] + self.actpos[2]) +
                        B_EL * math.cos(-self.actpos[1] + self.actpos[2]))
        tau_shoulder = (-tau_elbow + A_SH * math.sin(self.actpos[1]) +
                        B_SH * math.cos(self.actpos[1]))
        return (nan, pre_factor * tau_shoulder, pre_factor * tau_elbow, nan, nan)

    def sendcmd(self):
        """
        Send the joint commands to the robot. If the node class in question has 
        a trajectory class "self.traj" which may be used to generate the joint
        positions and velocities to be sent to the robot, then `traj.evaluate`
        is used to do so. If no trajectory is provded, the function will simply
        send the gravity resisting torques to the robot with nan position and
        velocity commands.
        """
        # Update the current time.
        self.t += self.dt
        # Compute the desired joint positions and velocities for this time.
        q, qdot = (self.traj.evaluate(self.t, self.dt) if hasattr(self, 'traj') and self.t > 2
                   else ((nan, nan, nan, nan, nan), (nan, nan, nan, nan, nan)))
        # Build up the joint command message to send to the motors and publish.
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = JOINTS
        self.cmdmsg.position     = q
        self.cmdmsg.velocity     = qdot
        self.cmdmsg.effort       = self.grav_model()
        self.cmdpub.publish(self.cmdmsg)

# LAUNCH NODES
# We use launch files to start several nodes concurrently in a particular
# configuration. Here we define some utility nodes which are almost always
# needed in a launch file as constants to avoid repeating their definitions.

RVIZCONFIG = os.path.join(pkgdir('utils'), 'rviz/viewurdf.rviz')
URDF = os.path.join(pkgdir('utils'), 'urdf/pb1.urdf')
with open(URDF, 'r') as file:
    ROBOT_DESCRIPTION = file.read()

# Node to publish the robot's URDF to the robot state publisher.
ACTUAL_RSP = launch_ros.actions.Node(
    name       = 'robot_state_publisher', 
    package    = 'robot_state_publisher',
    executable = 'robot_state_publisher',
    output     = 'screen',
    parameters = [{'robot_description': ROBOT_DESCRIPTION}])

# Node to publish the robot's URDF to RVIZ for visualization.
NODE_RVIZ = launch_ros.actions.Node(
    name       = 'rviz', 
    package    = 'rviz2',
    executable = 'rviz2',
    output     = 'screen',
    arguments  = ['-d', RVIZCONFIG],
    on_exit    = Shutdown())

# Node to communicate with the HEBI motors.
NODE_HEBI = launch_ros.actions.Node(
    name       = 'hebi', 
    package    = 'hebiros',
    executable = 'hebinode',
    output     = 'screen',
    parameters = [{'family':   'robotlab'},
                  {'motors':   MOTORS},
                  {'joints':   JOINTS}],
    on_exit    = Shutdown())

# Camera setup node
NODE_USBCAM = launch_ros.actions.Node(
    name       = 'usb_cam', 
    package    = 'usb_cam',
    executable = 'usb_cam_node_exe',
    namespace  = 'usb_cam',
    output     = 'screen',
    parameters = [{'camera_name':         'logitech'},
                    {'video_device':        '/dev/video0'},
                    {'pixel_format':        'yuyv2rgb'},
                    {'image_width':         800},
                    {'image_height':        600},
                    {'framerate':           15.0},
                    {'brightness':          -1},
                    {'contrast':            -1},
                    {'saturation':          -1},
                    {'sharpness':           -1},
                    {'gain':                30},
                    {'auto_white_balance':  False},
                    {'white_balance':       3200},
                    {'autoexposure':        False},
                    {'exposure':            250},
                    {'autofocus':           True},
                    {'focus':               -1}])
