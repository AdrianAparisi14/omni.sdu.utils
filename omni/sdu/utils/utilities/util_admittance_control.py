from omni.sdu.utils.utilities.math_util import skew_symmetric
import numpy as np
import sys
import termios
import atexit
from select import select

class CollisionDetector:
    """
    Method that looks at the force projected
    on to the direction of movement.
    """
    def __init__(self, threshold=-100):
        self.threshold = threshold

    def in_collision(self, vel, F, R, r):

        omega = vel[3:6]
        vel_trans = vel[0:3] + skew_symmetric(omega) @ R @ r
        force = F[:3]
        if np.linalg.norm(vel_trans) > 0.001:
            F_proj = np.dot(force, (vel_trans / np.linalg.norm(vel_trans)))
        else:
            F_proj = 0
        return F_proj < self.threshold


def get_param_as_matrix(val, dim=3):
    if isinstance(val, list):
        if len(val) == dim:
            return np.diag(val)
        else:
            raise TypeError("Wrong list size specified expected length to be "+str(dim)+" got: "+str(len(val)))
    elif isinstance(val, np.ndarray):
        if val.shape == (dim,):
            return np.diag(val)
        elif val.shape == (dim, dim):
            return val
        else:
            raise TypeError("Wrong input shape specified, expected ("+str(dim)+","+str(dim)+") or ("+str(dim)+",) got: "+str(val.shape))
    else:
        raise TypeError("Wrong input type specified, expected list, numpy array or numpy matrix")


class KBHit:

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
        atexit.register(self.set_normal_term)

    def set_normal_term(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    def getch(self):
        """ Returns a keyboard character after kbhit() has been called.
        """
        return sys.stdin.read(1)

    def kbhit(self):
        """ Returns True if keyboard character was hit, False otherwise.
        """
        dr, dw, de = select([sys.stdin], [], [], 0)
        return dr != []
