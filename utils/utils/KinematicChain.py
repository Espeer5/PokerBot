'''KinematicChainSol.py

   This is the solution code for Kinematic Chains (HW5 Problem 5).

   chain = KinematicChain(node, basefame, tipframe, expectedjointnames)

      Initialize the kinematic chain, reading from the URDF message on
      the topic '/robot_description', sent by the robot_state_publisher.
      Determine the kinematic steps walking from the baseframe to the
      tipframe.  This expects the active joints to match the given names.

   (ptip, Rtip, Jv, Jw) = chain.fkin(q)

      Compute the forward kinematics and report the results.


   Node:        /kintest or as given
   Subscribe:   /robot_description      std_msgs/String

'''

import enum
import rclpy
import numpy as np

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from std_msgs.msg               import String
from urdf_parser_py.urdf        import Robot

# Grab the utilities
from pb_utils.TransformHelpers   import *


#
#   Single Kinematic Step
#
#   This captures a single step from one frame to the next.  It be of type:
#
#     FIXED     Just a fixed T-matrix shift, nothing moving, not a DOF.
#     REVOLUTE  A fixed T-matrix shift, followed by a rotation about an axis.
#     LINEAR    A fixed T-matrix shift, followed by a transation along an axis.
#
#   A step contains several pieces of permanent data (coming from the URDF):
#
#     Tshift    Fixed shift: Transform of this frame w.r.t. previous
#     elocal    Joint axis (if applicable) in the local frame
#     type      One of the above
#     name      String showing the name
#     dof       If an active dof (not FIXED), the dof number
#
#   A step also contains several pieces of transient data, changing
#   over time or, more precisely, as the joint positions (angles) change:
#
#     T         Transform of this frame w.r.t. the world frame
#     p         Position  of this frame w.r.t. the world frame
#     R         Rotation  of this frame w.r.t. the world frame
#     e         Joint axis vector       w.r.t. the world frame
#
#   The latter information is reset and recomputed during each walk up
#   the chain.
#

# Define the joint types.
class Joint(enum.Enum):
    FIXED    = 0
    REVOLUTE = 1
    LINEAR   = 2

# Define a single step in the kinematic chain.
class KinematicStep():
    def __init__(self, Tshift, elocal, type, name):
        # Store the permanent/fixed/URDF data.
        self.Tshift = Tshift    # Transform w.r.t. previous frame
        self.elocal = elocal    # Joint axis in the local frame
        self.type   = type      # Joint type
        self.name   = name      # Joint name
        self.dof    = None      # Joint DOF number (or None if FIXED)

        # Clear the information to be updated every walk up the chain.
        self.clear()

    def clear(self):
        self.T = None           # Transform of frame w.r.t. world
        self.p = None           # Position  of frame w.r.t. world
        self.R = None           # Rotation  of frame w.r.t. world
        self.e = None           # Axis vector        w.r.t. world

    @classmethod
    def FromRevoluteJoint(cls, joint):
        return KinematicStep(T_from_URDF_origin(joint.origin),
                             e_from_URDF_axis(joint.axis),
                             Joint.REVOLUTE, joint.name)
    @classmethod
    def FromLinearJoint(cls, joint):
        return KinematicStep(T_from_URDF_origin(joint.origin),
                             e_from_URDF_axis(joint.axis),
                             Joint.LINEAR, joint.name)
    @classmethod
    def FromFixedJoint(cls, joint):
        return KinematicStep(T_from_URDF_origin(joint.origin),
                             np.zeros((3,1)),
                             Joint.FIXED, joint.name)


#
#   Kinematic Chain Object
#
#   This stores the information provided by the URDF in the form of
#   steps (see above).  In particular, see the fkin() function, as it
#   walks up the chain to determine the transforms.
#

# Define the full kinematic chain
class KinematicChain():
    # Helper functions for printing info and errors.
    def info(self, string):
        self.node.get_logger().info("KinematicChain: " + string)
    def error(self, string):
        self.node.get_logger().error("KinematicChain: " + string)
        raise Exception(string)

    # Initialization.
    def __init__(self, node, baseframe, tipframe, expectedjointnames):
        # Store the node (for the helper functions).
        self.node = node

        # Prepare the information.
        self.steps = []
        self.dofs  = 0

        # Grab the info from the URDF!
        self.load(baseframe, tipframe, expectedjointnames)

    # Load the info from the URDF.
    def load(self, baseframe, tipframe, expectedjointnames):
        # Create a temporary subscriber to receive the URDF.  We use
        # the TRANSIENT_LOCAL durability, so that we see the last
        # message already published (if any).
        self.info("Waiting for the URDF to be published...")
        self.urdf = None
        def cb(msg):
            self.urdf = msg.data
        topic   = '/robot_description'
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        sub = self.node.create_subscription(String, topic, cb, quality)
        while self.urdf is None:
            rclpy.spin_once(self.node)
        self.node.destroy_subscription(sub)

        # Convert the URDF string into a Robot object and report.
        robot = Robot.from_xml_string(self.urdf)
        self.info("Proccessing URDF for robot '%s'" % robot.name)

        # Parse the Robot object into a list of kinematic steps from
        # the base frame to the tip frame.  Search backwards, as the
        # robot could be a tree structure: while a parent may have
        # multiple children, every child has only one parent.  The
        # resulting chain of steps is unique.
        frame = tipframe
        while (frame != baseframe):
            # Look for the URDF joint to the parent frame.
            joint = next((j for j in robot.joints if j.child == frame), None)
            if (joint is None):
                self.error("Unable find joint connecting to '%s'" % frame)
            if (joint.parent == frame):
                self.error("Joint '%s' connects '%s' to itself" %
                           (joint.name, frame))
            frame = joint.parent

            # Convert the URDF joint into a simple step.
            if joint.type == 'revolute' or joint.type == 'continuous':
                self.steps.insert(0, KinematicStep.FromRevoluteJoint(joint))
            elif joint.type == 'prismatic':
                self.steps.insert(0, KinematicStep.FromLinearJoint(joint))
            elif joint.type == 'fixed':
                self.steps.insert(0, KinematicStep.FromFixedJoint(joint))
            else:
                self.error("Joint '%s' has unknown type '%s'" %
                           (joint.name, joint.type))

        # Set the active DOF numbers walking up the steps.
        dof = 0
        for s in self.steps:
            if s.type is not Joint.FIXED:
                s.dof = dof
                dof += 1
        self.dofs = dof
        self.info("URDF has %d steps, %d active DOFs:" %
                  (len(self.steps), self.dofs))

        # Report we found.
        for (step, s) in enumerate(self.steps):
            string = "Step #%d %-8s " % (step, s.type.name)
            string += "      " if s.dof is None else "DOF #%d" % s.dof
            string += " '%s'" % s.name
            self.info(string)

        # Confirm the active joint names matches the expectation
        jointnames = [s.name for s in self.steps if s.dof is not None]
        if jointnames != list(expectedjointnames):
            self.error("Chain does not match the expected names: " +
                  str(expectedjointnames))


    # Compute the forward kinematics!
    def fkin(self, q):
        # Check the number of joints
        if (len(q) != self.dofs):
            self.error("Number of joint angles (%d) does not chain (%d)",
                       len(q), self.dofs)

        # Clear any data from past invocations (just to be safe).
        for s in self.steps:
            s.clear()

        # Initialize the T matrix to walk up the chain, w.r.t. world frame!
        T = np.eye(4)

        # Walk the chain, one step at a time.  Record the T transform
        # w.r.t. world for each step.
        for s in self.steps:
            # Always apply the shift.
            T = T @ s.Tshift

            # For active joints, also apply the joint movement.
            if s.type is Joint.REVOLUTE:
                # Revolute is a rotation:
                T = T @ T_from_Rp(Rote(s.elocal, q[s.dof]), pzero())
            elif s.type is Joint.LINEAR:
                # Linear is a translation:
                T = T @ T_from_Rp(Reye(), s.elocal * q[s.dof])

            # Store the info (w.r.t. world frame) into the step.
            s.T = T
            s.p = p_from_T(T)
            s.R = R_from_T(T)
            s.e = R_from_T(T) @ s.elocal

        # Collect the tip information.
        ptip = p_from_T(T)
        Rtip = R_from_T(T)

        # Re-walk up the chain to fill in the Jacobians.
        Jv = np.zeros((3,self.dofs))
        Jw = np.zeros((3,self.dofs))
        for s in self.steps:
            if s.type is Joint.REVOLUTE:
                # Revolute is a rotation:
                Jv[:,s.dof:s.dof+1] = cross(s.e, ptip - s.p)
                Jw[:,s.dof:s.dof+1] = s.e
            elif s.type is Joint.LINEAR:
                # Linear is a translation:
                Jv[:,s.dof:s.dof+1] = s.e
                Jw[:,s.dof:s.dof+1] = np.zeros((3,1))

        # Return the info
        return (ptip, Rtip, Jv, Jw)


#
#   Main Code
#
#   This simply tests the kinematic chain and associated calculations!
#
def main(args=None):
    # Set the print options to something we can read.
    np.set_printoptions(precision=3, suppress=True)

    # Initialize ROS and the node.
    rclpy.init(args=args)
    node = Node('kintest')

    # Set up the kinematic chain object, assuming the 3 DOF.
    jointnames = ['theta1', 'theta2', 'theta3']
    baseframe  = 'world'
    tipframe   = 'tip'
    chain = KinematicChain(node, baseframe, tipframe, jointnames)

    # Define the test.
    def test(q):
        (ptip, Rtip, Jv, Jw) = chain.fkin(q)
        print('q:\n',       q)
        print('ptip(q):\n', ptip)
        print('Rtip(q):\n', Rtip)
        print('Jv(q):\n',   Jv)
        print('Jw(q):\n',   Jw)
        print('----------------------------------------');

    # Run the tests.
    test(np.radians(np.array([  20.0,   40.0,  -30.0])).reshape(3,1))
    test(np.radians(np.array([  30.0,   30.0,   60.0])).reshape(3,1))
    test(np.radians(np.array([ -45.0,   75.0,  120.0])).reshape(3,1))

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
