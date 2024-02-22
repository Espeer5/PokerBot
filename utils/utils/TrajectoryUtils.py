'''TrajUtils.py

   Trajectory (Spline) Utility Functions

   from TrajUtils import hold, interpolate, goto, spline, goto5, spline5

   The functions

      (p,v) = hold(             p0)                         Constant

      (p,v) = interpolate(t, T, p0, pf)                     Linear

      (p,v) = goto(       t, T, p0, pf)                     Cubic
      (p,v) = spline(     t, T, p0, pf, v0, vf)             Cubic

      (p,v) = goto5(      t, T, p0, pf)                     Quintic
      (p,v) = spline5(    t, T, p0, pf, v0, vf, a0, af)     Quintic

   each compute the position and velocity of the variable as a
   function of time.  They use a constant/linear.cubic/quintic
   polynomial with the given boundary conditions.

   NOTE TIME IS RELATIVE, so each assume the time advances from t=0
   through t=T!

   The (p0,pf,v0,vf,a0,af) may be NumPy arrays and the (p,v) are
   returned with the appropriate dimensions.

'''
import numpy as np


#
#   Constant Helpers
#
#   This is really only included for completeness and to zero the
#   velocity (with the same dimension).
#
def hold(p0):
    # Compute the current (p,v).
    p = p0
    v = 0*p0
    return (p,v)


#
#   Linear Helpers
#
#   Linearly interpolate between an initial/final position of the time T.
#
def interpolate(t, T, p0, pf):
    # Compute the current (p,v).
    p = p0 + (pf-p0)/T * t
    v =    + (pf-p0)/T
    return (p,v)


#
#   Cubic Spline Helpers
#
#   Compute a cubic spline position/velocity as it moves from (p0, v0)
#   to (pf, vf) over time T.
#
def goto(t, T, p0, pf):
    # Compute the current (p,v).
    p = p0 + (pf-p0)   * (3*(t/T)**2 - 2*(t/T)**3)
    v =    + (pf-p0)/T * (6*(t/T)    - 6*(t/T)**2)
    return (p,v)

def spline(t, T, p0, pf, v0, vf):
    # Compute the parameters.
    a = p0
    b = v0
    c =   3*(pf-p0)/T**2 - vf/T    - 2*v0/T
    d = - 2*(pf-p0)/T**3 + vf/T**2 +   v0/T**2
    # Compute the current (p,v).
    p = a + b * t +   c * t**2 +   d * t**3
    v =     b     + 2*c * t    + 3*d * t**2
    return (p,v)


#
#   Quintic Spline Helpers
#
#   Compute a quintic spline position/velocity as it moves from
#   (p0,v0,a0) to (pf,vf,af) over time T.
#

def goto5(t, T, p0, pf):
    # Compute the current (p,v).
    p = p0 + (pf-p0)   * (10*(t/T)**3 - 15*(t/T)**4 +  6*(t/T)**5)
    v =    + (pf-p0)/T * (30*(t/T)**2 - 60*(t/T)**3 + 30*(t/T)**4)
    return (p,v)

def spline5(t, T, p0, pf, v0, vf, a0, af):
    # Compute the parameters.
    a = p0
    b = v0
    c = a0
    d = + 10*(pf-p0)/T**3 - 6*v0/T**2 - 3*a0/T    - 4*vf/T**2 + 0.5*af/T
    e = - 15*(pf-p0)/T**4 + 8*v0/T**3 + 3*a0/T**2 + 7*vf/T**3 -     af/T**2
    f = +  6*(pf-p0)/T**5 - 3*v0/T**4 -   a0/T**3 - 3*vf/T**4 + 0.5*af/T**3
    # Compute the current (p,v).
    p = a + b * t +   c * t**2 +   d * t**3 +   e * t**4 +   f * t**5
    v =     b     + 2*c * t    + 3*d * t**2 + 4*e * t**3 + 5*f * t**4
    return (p,v)


def calculate_desired_exit_velocity(point0, point1, desired_dt):
    g = -9.81

    x0, y0, z0 = point0
    x1, y1, z1 = point1

    delta_x = x1 - x0
    delta_y = y1 - y0
    delta_z = z1 - z0

    vx = delta_x / desired_dt
    vy = delta_y / desired_dt
    vz = (delta_z - (1/2 * g * desired_dt ** 2)) / desired_dt

    return np.array([[vx], [vy], [vz]]).reshape(3, 1)


def calculate_kicking_velocity(point0, point1, desired_dt, elasticity, vball):
    # desired_exit_velocity = (kicking_velocity + ball_velocity) * elasticity
    d_exit_velocity = calculate_desired_exit_velocity(point0, point1, desired_dt)

    norm = d_exit_velocity / np.linalg.norm(d_exit_velocity)
    ball_velo_on_axis = (norm * vball) * norm

    # kicking_velocity = (desired_exit_velocity / elasticity) - ball_velocity
    kicking_velocity = (d_exit_velocity / elasticity) - ball_velo_on_axis
    return kicking_velocity.reshape(3, 1)






