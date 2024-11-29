import numpy as np
from typing import Self
import numpy.linalg
from scipy.optimize  import fsolve
import time
import matplotlib.pyplot as plt


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
        y = - self.r * np.sin(theta)            
        
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


    def point_from_RSSI(self:Self, RSSI: float, router_point: np.array, current_z: float):

        # Parameters from the Spiral class
        r_sp = self.r
        pitch = self.pitch

        # RSSI parameters
        rssi_measured_power = -40 #dBm
        rssi_path_loss_exponent = 2

        # Distance from RSSI
        distance = 10 ** ((rssi_measured_power - RSSI) / (10 * rssi_path_loss_exponent))

        # Router point to cylindrical coords
        r_pt = np.sqrt(router_point[0]**2 + router_point[1]**2)
        theta_pt = np.arctan2(router_point[1], router_point[0]) 
        z_pt = router_point[2]
        
        # Distance equation, solving for z_sp iteratively
        def equation(z_sp):
            return (r_sp**2 + r_pt**2 - 2 * r_sp * r_pt * np.cos(((2 * np.pi * z_sp) / pitch) - theta_pt) + (z_pt - z_sp)**2) - distance**2

        # Max number of iterations for faster computation
        options = {'maxfev': 40}

        # Since there can be many solutions to the equation, we want the one closest to the current location of the person
        initial_guess = current_z  
        z_sp = fsolve(equation, initial_guess,full_output=True, **options)

        return self.point_at_z(z_sp[0][0])
        

################################################################# TESTING ############################################################
TESTING = False

if (TESTING):
    # Wifi data from one router
    rssi_measured_power = -40 # typical 1-meter-RSSI for WiFi routers is between -40 and -60 dBm
    rssi_path_loss_exponent = 2 # typical indoor path loss exponent is between 4 and 6

    X, Y, Z, MAC = 0, 1, 2, 3 # for convenience

    wifi_router = [-0.9229859736350821, -0.8181204089944636, 8.208368772600491, '45:84:bd:7b:45:8f']

    # For testing, we generate the RSSI of the router depending on the person's location
    def get_RSSI_MAC(position):
        wifi_data = []

        distance = np.sqrt(
                (wifi_router[X] - position[X]) ** 2 +
                (wifi_router[Y] - position[Y]) ** 2 +
                (wifi_router[Z] - position[Z]) ** 2
            )

        mac = wifi_router[MAC]
        wifi_data.append(mac)
        
        rssi = rssi_measured_power - 10 * rssi_path_loss_exponent * np.log10(distance)
        wifi_data.append(rssi)

        return wifi_data


    spiral = Spiral(4, 8, 2.4)

    router = np.array([wifi_router[0],wifi_router[1],wifi_router[2]])

    # Randomly chosen a current location of the person and calculated the distance between them and the router
    test_point = spiral.point_at_z(3)
    print("test point, ", test_point)

    test_distance = distance = np.sqrt(
                (wifi_router[X] - test_point[X]) ** 2 +
                (wifi_router[Y] - test_point[Y]) ** 2 +
                (wifi_router[Z] - test_point[Z]) ** 2
            )
    print("test distance", test_distance)

    # Retrieved the RSSI depending on the person's location and the router
    RSSI_test = get_RSSI_MAC(test_point)
    print("RSSI test", RSSI_test)

    # We use the retrieved RSSI to extrapolate the person's location and verify the distance between the router and the extr. point
    point_sp = spiral.point_from_RSSI(RSSI_test[1],router)
    print("extrapoalted point", point_sp)


    distance = np.sqrt(
                (wifi_router[X] - point_sp[X]) ** 2 +
                (wifi_router[Y] - point_sp[Y]) ** 2 +
                (wifi_router[Z] - point_sp[Z]) ** 2
            )
    print("extrapolated distance", distance)

    ### PLOTTING ###
    
    # Spiral from "raw heights" for better visualization
    spirall = []
    for height in np.linspace(0,13,1000):
        spirall.append(spiral.point_at_z(height))
    spirall = np.array(spirall)

    # Plotting the spiral, the current location and the extrapolated location (from RSSI)

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(spirall[:,0], spirall[:,1], spirall[:,2], 'red', label='Predicted Positions (Kalman)')
    ax.scatter(point_sp[0], point_sp[1], point_sp[2], color='red', s=50, label='Point from RSSI')
    ax.scatter(router[0], router[1], router[2], color='blue', s=50, label='Point')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()  