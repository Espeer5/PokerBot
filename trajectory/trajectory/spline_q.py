"""
This module contains the code for a class to represent a joint spline, as well
as a queue of joint splines. These are used by the Control node to receive goal
commands from the brain node and generate and follow the appropriate joint 
trajectories to reach the goal commands. The joint splines are used to generate
the joint positions and velocities at each time step to reach the goal commands
with the appropriate final velocity.
"""

import numpy as np

from utils.TrajectoryUtils import spline5


class JointSpline():
    """
    A class used to represent a joint spline between 2 joint states. The spline
    is general in that it may be given any set of positions and initial/final 
    velocities, and will generate the appropraiate intermediate joint positions
    and velocities. The spline is generated with a time duration set by the 
    brain node such that the brain can control the maximum velocity of the robot
    """
    def __init__(self, q0, qf, qdot0, qdotf, T):
        self.q0 = q0
        self.qf = qf
        self.qdot0 = qdot0
        self.qdotf = qdotf
        self.T = T

    def evaluate(self, t):
        """
        Return the appropriate joint positions and velocities at the given time 
        t in seconds from the start of the spline.
        """
        return spline5(t, self.T, self.q0, self.qf, self.qdot0, self.qdotf,
                          np.zeros((5, 1)), np.zeros((5, 1)))
        
    def complete(self, t):
        """
        Return True if the spline has been completed, and False otherwise.
        """
        return t >= self.T


class JointSplineQueue():
    """
    A queue of joint spline objects. This class is used by the Poker Bot control
    node to store the commanded locations and velocities from the brain node and
    generate the appropriate joint positions and velocities at each time step. 
    Splines are dequeued as the robot successfully reaches commanded joint
    locations.
    """
    def __init__(self):
        self.t0 = 0
        self.splines = []

    def enqueue(self, q0, qf, qdot0, qdotf, T):
        """
        Add a joint spline to the queue with the given initial and final joint
        positions, initial and final joint velocities, and time duration.
        """
        self.splines.append(JointSpline(q0, qf, qdot0, qdotf, T))
    
    def dequeue(self):
        """
        Remove and return the first joint spline from the queue.
        """
        return self.splines.pop(0)
    
    def evaluate(self, t):
        """
        Return the appropriate joint positions and velocities at the given time 
        t in seconds from the start of the first spline in the queue. If the
        firsts spline in the queue is completed, it is dequeued and the next 
        spline begins evaluation with self.t0 reset to the start time of the new
        spline. if the queue is emtpy, the funtion returns None, None to allow
        the control node trajectory object to decide what to do next. In the 
        case of the empty queue, the starting time of the next spline is set to
        the current time so that when the next spline is enqueued, the start time
        of that spline will be the current time.
        """
        if len(self.splines) > 0:
            if self.splines[0].complete(t - self.t0):
                self.dequeue()
                self.t0 = t
                # In this case, we need to recheck if the queue is now empty
                return self.evaluate(t)
            return self.splines[0].evaluate(t - self.t0)
        else:
            self.t0 = t
            return None, None
        
    def empty(self):
        """
        Return True if the queue is empty, and False otherwise.
        """
        return len(self.splines) == 0
    
    def peek_back(self):
        """
        Return the last joint spline in the queue without removing.
        """
        return self.splines[-1]
