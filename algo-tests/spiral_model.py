import numpy as np
import matplotlib.pyplot as plt
import numpy.linalg

### Finds the closest point from a random point to the helix
def closest_point(pitch, r_sp, random_pt):

    # Extract random point coordinates
    x_pt, y_pt, z_pt = random_pt
    
    # Transform point to cylindrical coordinates
    theta_pt = np.arctan2(y_pt, x_pt)  
    z_offset = pitch * (z_pt // pitch)  
    normalized_z_pt = z_pt % pitch      

    # Derive possible solutions for z_sp
    threshold = (np.pi + pitch) * theta_pt / (2 * np.pi)
    normalized_z_sp_1 = (pitch * theta_pt + np.pi * pitch) / (2 * np.pi) if normalized_z_pt > threshold else (pitch * theta_pt) / (2 * np.pi)
    normalized_z_sp_2 = (pitch * theta_pt - np.pi * pitch) / (2 * np.pi)
    normalized_z_sp_3 = normalized_z_pt

    z_sp_candidates = np.array([normalized_z_sp_1, normalized_z_sp_2, normalized_z_sp_3]) + z_offset

    # Compute distances and find the closest z_sp
    distances = [
        np.sqrt(
            (r_sp * np.cos(2 * np.pi * z_sp / pitch) - x_pt) ** 2 +
            (r_sp * np.sin(2 * np.pi * z_sp / pitch) - y_pt) ** 2 +
            (z_sp - z_pt) ** 2
        )
        for z_sp in z_sp_candidates
    ]

    z_sp = z_sp_candidates[np.argmin(distances)]
    theta_sp = 2 * np.pi * z_sp / pitch

    # Transform back to Cartesian coordinates
    x_sp = r_sp * np.cos(theta_sp)
    y_sp = r_sp * np.sin(theta_sp)

    return [x_sp, y_sp, z_sp]



### Checks if a random point is within the helix's "thickness"
def is_in_spiral(pitch, r_sp, thickness_sp, random_pt):

    closest_point = closest_point(pitch, r_sp, random_pt)
    distance = numpy.linalg.norm(closest_point-random_pt)

    return distance <= thickness_sp


