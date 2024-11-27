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
        theta = 2 * np.pi * (z / self.pitch)  
        x = self.r * (np.cos(theta))      
        y = self.r * np.sin(theta)            
        
        return np.array([x, y, z])



    ### Checks if a random point is within the helix's "thickness"
    def is_point_on(self: Self, random_pt: np.ndarray) -> bool:

        closest_point = closest_point(self.pitch, self.r, random_pt)
        distance = numpy.linalg.norm(closest_point-random_pt)

        return distance <= self.pathwidth
    

    ### Centers spiral to x = 0 and y = 0
    def center_spiral(self: Self, spiral: np.array):

        # Find x, y bounds for predicting center
        x_max_spiral = np.max(spiral[:, 0])
        x_min_spiral = np.min(spiral[:, 0])
        y_max_spiral = np.max(spiral[:, 1])
        y_min_spiral = np.min(spiral[:, 1])

        # Compute centroids
        center_xy_spiral = [(x_max_spiral + x_min_spiral) / 2, (y_max_spiral + y_min_spiral) / 2]


        spiral[:,0] -= center_xy_spiral[0]

        spiral[:,1] -= center_xy_spiral[1]

        return spiral
    

    ### Rotates first spiral so that it matches the second one
    def rotate(self: Self, rotating_spiral, reference_spiral):

        rotating_spiral = np.array(rotating_spiral, dtype=float).reshape(-1, 3)
        reference_spiral = np.array(reference_spiral, dtype=float).reshape(-1, 3)
        
        # Get the starting points of the predicted and groundtruth spirals
        predicted_start = rotating_spiral[0, :2]  # XY of predicted
        gt_start = reference_spiral[0, :2]  # XY of groundtruth
        
        # Angle between the two vectors in the XY plane
        dot_product = np.dot(predicted_start, gt_start)
        norm_red = np.linalg.norm(predicted_start)
        norm_blue = np.linalg.norm(gt_start)
        angle = np.arccos(dot_product / (norm_red * norm_blue))
        
        # 2D rotation matrix for rotation around the z-axis
        rotation_matrix = np.array([
            [np.cos(-angle), -np.sin(-angle), 0],
            [np.sin(-angle),  np.cos(-angle), 0],
            [0,               0,              1]
        ])
        
        # Apply rotation to all points in the red (predicted) spiral
        rotated_positions = np.dot(rotating_spiral, rotation_matrix.T)
        
        return rotated_positions


def point_from_RSSI(self: Self, RSSI: float):

    pi = np.pi
    r_sp = self.radius
    pitch = self.pitch

    A = -40 
    n = 4
    distance = 10 ** ((A - RSSI) / (10 * n))

    theta_sp = (2*pi*z_sp) / pitch
    theta_pt = 0

    x = theta_sp - theta_pt

    # Numerator
    numerator = 16 * (x + pi / 2) * (pi / 2 - x)
    
    # Denominator
    denominator = 5 * pi**2 - 4 * (x + pi / 2) * (pi / 2 - x)
    
    # Function value
    cos = numerator / denominator

    distance = np.sqrt( np.square(r_sp) + np.square(r_pt) - 2*r_sp*r_pt*cos + (z_pt - z_sp)**2 )

    return 0