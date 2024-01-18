import math
import numpy as np

#
#  Newton Raphson
#
def newton_raphson(chain, xgoal, q0):
    # Collect the distance to goal and change in q every step!
    xdistance = []
    qstepsize = []

    # Set the initial joint value guess.
    # q = np.array([0.0, np.pi/2, -np.pi/2]).reshape(3,1)
    q = q0
    print("qsize=", len(q))

    # IMPLEMENT THE NEWTON-RAPHSON ALGORITHM!
    e = 1
    counter = 0
    while e > math.pow(10, -12):
        if counter > 20:
            break
        counter += 1
        p, _, Jv, Jw = chain.fkin(q)
        Jac = np.stack([Jv, Jw])

        jac_inverse = np.linalg.pinv(Jac @ q)
        delta_x = xgoal - p
        next_q = q + jac_inverse @ delta_x

        p, _, _, _ = chain.fkin(next_q)
        e = np.linalg.norm(xgoal - p)
        xdistance.append(e)
        qstepsize.append(np.linalg.norm(next_q - q))
        q = next_q

    print("q.size=", len(q))
    return q