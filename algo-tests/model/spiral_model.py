import numpy as np
from typing import Self
import numpy.linalg


class Spiral:

    def __init__(self: Self, pitch: float, radius: float, pathwidth: float):
        self.pitch = pitch
        self.r = radius
        self.pathwidth = pathwidth

    ### Finds the closest point from a random point to the helix
    def closest_point_to(self: Self, random_pt: np.ndarray) -> np.ndarray:

        # Extract random point coordinates
        x_pt, y_pt, z_pt = random_pt
        
        # Transform point to cylindrical coordinates
        theta_pt = np.arctan2(y_pt, x_pt)  
        z_offset = self.pitch * (z_pt // self.pitch)  
        normalized_z_pt = z_pt % self.pitch      

        # Derive possible solutions for z_sp
        threshold = (np.pi + self.pitch) * theta_pt / (2 * np.pi)
        normalized_z_sp_1 = (self.pitch * theta_pt + np.pi * self.pitch) / (2 * np.pi) if normalized_z_pt > threshold else (self.pitch * theta_pt) / (2 * np.pi)
        normalized_z_sp_2 = (self.pitch * theta_pt - np.pi * self.pitch) / (2 * np.pi)
        normalized_z_sp_3 = normalized_z_pt

        z_sp_candidates = np.array([normalized_z_sp_1, normalized_z_sp_2, normalized_z_sp_3]) + z_offset

        # Compute distances and find the closest z_sp
        distances = [
            np.sqrt(
                (self.r * np.cos(2 * np.pi * z_sp / self.pitch) - x_pt) ** 2 +
                (self.r * np.sin(2 * np.pi * z_sp / self.pitch) - y_pt) ** 2 +
                (z_sp - z_pt) ** 2
            )
            for z_sp in z_sp_candidates
        ]

        z_sp = z_sp_candidates[np.argmin(distances)]
        theta_sp = 2 * np.pi * z_sp / self.pitch

        # Transform back to Cartesian coordinates
        x_sp = self.r * np.cos(theta_sp)
        y_sp = self.r * np.sin(theta_sp)

        return np.array([x_sp, y_sp, z_sp])


    def point_at_z(self: Self, z: float) -> np.ndarray:
        x = self.r * np.cos(2*np.pi*z/self.pitch)
        y = self.r * np.sin(2*np.pi*z/self.pitch)
        
        return np.array([x, y, z])



    ### Checks if a random point is within the helix's "thickness"
    def is_point_on(self: Self, random_pt: np.ndarray) -> bool:

        closest_point = closest_point(self.pitch, self.r, random_pt)
        distance = numpy.linalg.norm(closest_point-random_pt)

        return distance <= self.pathwidth


