import quaternion as qt
import numpy as np

def q2eul(q):
    q0, q1, q2, q3 = qt.as_float_array(q)
    roll = np.arctan2(2*(q0*q1 + q2*q3), 1-2*(q1**2 + q2**2))
    pitch = np.arcsin(2*(q0*q2 - q3*q1))
    yaw = np.arctan2(2*(q0*q3 + q1*q2), 1- 2*(q2**2 + q3**2))

    return np.array([roll, pitch, yaw])

def eul2q(roll, pitch, yaw): #pitch, roll, yaw):

    qroll = qt.quaternion(np.cos(roll/2), np.sin(roll/2), 0, 0)
    qpitch = qt.quaternion(np.cos(pitch/2), 0, np.sin(pitch/2), 0)
    qyaw = qt.quaternion(np.cos(yaw/2), 0, 0, np.sin(yaw/2))

    return qyaw*qpitch*qroll



def q2rotmat(q):

    M = np.empty((3,3))
    M[:, 0] = (q*qt.quaternion(0,1,0,0)*q.conjugate()).vec
    M[:, 1] = (q*qt.quaternion(0,0,1,0)*q.conjugate()).vec
    M[:, 2] = (q*qt.quaternion(0,0,0,1)*q.conjugate()).vec

    return M
