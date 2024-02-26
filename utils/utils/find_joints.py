"""
This module contains a method for solving the ikin for the poker bot 
to find the joints angles for a particular position. This ikin is solved
repeatedly until the desired position is reached, and then the corresponding
joints are returned. These joint angles may then be used in a joint spline to 
send the bot to a particular location.
"""

import numpy as np
from utils.KinematicChain     import *
from utils.TransformHelpers   import *
from utils.TrajectoryUtils      import *
from utils.constants import LAMBDA, SIM_T

def find_joints(chain: KinematicChain, goalp: np.ndarray, 
                goal_th: float, goal_phi: float = 0.0) -> np.ndarray:
    """
    Iteratively solve fkin using the Jacobian until the desired position is
    reached. The theta for the angle of the end affector to the table surface is 
    always 0, and the angle in the plane of the table is passed in along with
    the desired position and solved for. Since the angle of the end affector the
    the table is 0, the goal theta is controlled fully by the angle of the tip 
    rotation joint so that it can be controlled independently.

    The goal theta should be specified as the local angle at a point -> i.e 0
    degrees at ANY point is always at the same angle to the table.
    """
    # Intialize the joint angles
    q = np.array([-1.5708, 1.5708, 1.5708, 0, 0]).reshape(5, 1)
    # Initialize the starting position
    p, _, Jv, _ = chain.fkin(q)
    initialp = p
    initial_phi = q[1] - q[2] + q[3]
    # Create fake spline timing
    t = 0
    dt = 0.1

    # Iterate until the distance between the current position and the goal
    # position is less than 0.001
    while t < SIM_T:
        path_p, path_v = goto5(t, SIM_T, 0, 1)
        pd = initialp + path_p * (goalp - initialp)
        vd = path_v * (goalp - initialp) / SIM_T
        phi_d = initial_phi + path_p * (goal_phi - initial_phi)
        # Compute the errors
        epd = ep(pd, p)
        eth = phi_d - (q[1] - q[2] + q[3])
        e_s = np.vstack((epd, eth))
        J = np.vstack((Jv, np.array([0, 1, -1, 1, 0])))
        v_s = np.vstack((vd, eth))
        JT = np.transpose(J)
        # Compute the q_dot
        gamma = 0.1
        J_winv = JT @ np.linalg.pinv((J @ JT + gamma**2 * np.eye(4)))
        qdot = J_winv @ (v_s + LAMBDA * e_s)
        # Integrate to get the new q
        q = q + qdot
        # Compute the new position
        p, _, Jv, _ = chain.fkin(q)
        t += dt
    tip_th = goal_th + q[0][0]
    return np.array([q[0][0], q[1][0], q[2][0], q[3][0], tip_th]).reshape(5, 1)
