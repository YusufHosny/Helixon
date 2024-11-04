import numpy as np

# rotate vector with quaternion
def rot_vect(vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    q = quaternion
    v = np.array([0, *vector])
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    v_rot = quat_mult(quat_mult(q, v), q_conj)
    return v_rot[1:]  # Return only the x, y, z part of the quaternion product

# multiply quaternions
def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

def slerp(q1: np.ndarray, q2: np.ndarray, t: int) -> np.ndarray:
        
        # if no q1 then identity (unit quaternion)
        if q1 is None:
            q1 = np.array([1, 0, 0, 0])

        # norm the quaternions to unit quat
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        # if dot product is negative, negate one quaternion to take the double cover shorter path
        dot_product = np.dot(q1, q2)
        if dot_product < 0.0:
            q2 = -q2
            dot_product = -dot_product

        # clamp the dot product to be valid
        dot_product = np.clip(dot_product, -1.0, 1.0)

        # get angle between the quaternions
        theta = np.arccos(dot_product)

        # if small angle, lerp to avoid division by zero explosion
        if theta < 1e-6:
            return (1 - t) * q1 + t * q2

        # SLERP otherwise
        sin_theta = np.sin(theta)
        w1 = np.sin((1 - t) * theta) / sin_theta
        w2 = np.sin(t * theta) / sin_theta
        q_interpolated = w1 * q1 + w2 * q2

        return q_interpolated
