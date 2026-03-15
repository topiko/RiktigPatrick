import numpy as np


class Quaternion:
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def from_array(arr):
        return Quaternion(arr[0], arr[1], arr[2], arr[3])

    def as_array(self):
        return np.array([self.w, self.x, self.y, self.z])

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
            x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
            y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
            z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
            return Quaternion(w, x, y, z)
        else:
            return Quaternion(self.w * other, self.x * other, self.y * other, self.z * other)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __add__(self, other):
        return Quaternion(self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z)

    def __truediv__(self, other):
        return Quaternion(self.w / other, self.x / other, self.y / other, self.z / other)

    @property
    def vec(self):
        return np.array([self.x, self.y, self.z])

    @vec.setter
    def vec(self, value):
        self.x, self.y, self.z = value

    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def norm(self):
        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalize(self):
        n = self.norm()
        if n > 0:
            return self / n
        return self


def q2eul(q) -> np.ndarray:
    if isinstance(q, Quaternion):
        w, x, y, z = q.w, q.x, q.y, q.z
    else:
        w, x, y, z = q

    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    return np.array([roll, pitch, yaw])


def eul2q(roll: float, pitch: float, yaw: float):
    cr = np.cos(roll / 2)
    sr = np.sin(roll / 2)
    cp = np.cos(pitch / 2)
    sp = np.sin(pitch / 2)
    cy = np.cos(yaw / 2)
    sy = np.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return Quaternion(w, x, y, z)


def as_rotation_matrix(q) -> np.ndarray:
    if not isinstance(q, Quaternion):
        q = Quaternion.from_array(q)

    w, x, y, z = q.w, q.x, q.y, q.z

    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def q2rotmat(q) -> np.ndarray:
    return as_rotation_matrix(q)
