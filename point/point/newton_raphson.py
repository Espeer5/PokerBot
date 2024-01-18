import math
import numpy as np

def unwrapped(q):
    return q - np.round(q / (2*np.pi)) * (2*np.pi)

#
#  Newton Raphson
#
def newton_raphson(chain, xgoal, q0):
    # Set the initial joint value guess.
    # q = np.array([0.0, np.pi/2, -np.pi/2]).reshape(3,1)
    q = q0

    # IMPLEMENT THE NEWTON-RAPHSON ALGORITHM!
    e = 1
    counter = 0
    print("entered newton raphson")
    while e > math.pow(10, -12):
        counter += 1
        p, _, Jv, _ = chain.fkin(q)
        Jac = Jv
        jac_inverse = np.linalg.inv(Jac)
        delta_x = xgoal - p
        step = jac_inverse @ delta_x
        next_q = q + step
        
        p, _, _, _ = chain.fkin(next_q)
        e = np.linalg.norm(xgoal - p)
        q = next_q
    q = unwrapped(q)
    print("q: ",q)
    print("p: ",p)
    return q