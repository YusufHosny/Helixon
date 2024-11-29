import os
import numpy as np
import h5py
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize

def calculate_average_tangent(data, num_points=100):
    """Calculate the average tangent vector from the first `num_points`."""
    tangents = np.diff(data[:num_points, :3], axis=0)
    avg_tangent = np.mean(tangents, axis=0)
    return avg_tangent / np.linalg.norm(avg_tangent)

def align_starting_point(data, ref_start):
    """Translate the dataset so that it starts at the reference point."""
    start = data[0, :3]
    translation = ref_start - start
    data[:, :3] += translation
    return data

def rotate_to_match_tangent(data, ref_tangent):
    """Rotate data around the z-axis to match the reference tangent."""
    def tangent_difference(angle):
        """Calculate the difference between tangents after rotation."""
        rotation = R.from_rotvec(angle * np.array([0, 0, 1]))
        rotated_data = rotation.apply(data[:, :3])
        avg_tangent = calculate_average_tangent(rotated_data)
        return np.linalg.norm(avg_tangent[:2] - ref_tangent[:2])  # Only consider x-y projection

    # Optimize the rotation angle
    result = minimize(tangent_difference, x0=0, bounds=[(-np.pi, np.pi)], tol=1e-6)
    optimal_angle = result.x[0]

    # Apply the optimal rotation
    final_rotation = R.from_rotvec(optimal_angle * np.array([0, 0, 1]))
    data[:, :3] = final_rotation.apply(data[:, :3])

    return data

# Directory paths
input_dir = "data/"  # Replace with actual input directory
output_dir = "data/aligned/"  # Replace with desired output directory
os.makedirs(output_dir, exist_ok=True)

# Process each .h5 file
file_names = [
    "NormalUDP1.h5", "NormalUDP2.h5", "RandomUDP1.h5",
    "RandomUDP2.h5", "RandomUDP3.h5", "RandomUDP4.h5",
    "RandomUDP5.h5", "RandomUDP6.h5"
]

# Reference tangent and start
reference_tangent = None
reference_start = None

def copy_group(input_group, output_group, is_reference=False):
    global reference_tangent, reference_start  # Declare global reference tangent and start
    for key, item in input_group.items():
        if isinstance(item, h5py.Dataset):
            if input_group.name == "/GT_DATA" and key == "POSITION":
                # Align only the POSITION dataset in GT_DATA
                data = item[:]
                if is_reference:
                    # Use the first file to calculate reference tangent and start point
                    reference_tangent = calculate_average_tangent(data)
                    reference_start = data[0, :3].copy()
                else:
                    # Align starting point
                    data = align_starting_point(data, reference_start)
                    # Rotate to match the reference tangent
                    data = rotate_to_match_tangent(data, reference_tangent)
                output_group.create_dataset(key, data=data)
            else:
                # Copy datasets without modification
                output_group.create_dataset(key, data=item[:])
        elif isinstance(item, h5py.Group):
            # Recursively copy groups
            new_group = output_group.create_group(key)
            copy_group(item, new_group, is_reference=is_reference)

for i, file_name in enumerate(file_names):
    input_path = os.path.join(input_dir, file_name)
    output_path = os.path.join(output_dir, file_name)
    
    # Open the input and output HDF5 files
    with h5py.File(input_path, 'r') as input_file, h5py.File(output_path, 'w') as output_file:
        is_reference = i == 0  # The first file is the reference
        copy_group(input_file, output_file, is_reference=is_reference)
