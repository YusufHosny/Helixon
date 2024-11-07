import h5py 
import numpy as np
import os

import os
import numpy as np
import h5py

def storeAsHDF5(dataset_name, raw_data, gt_data):
    # Grouping raw data based on the specified structure
    timestamp_data = np.array([row[0] for row in raw_data], dtype=np.float64)
    nine_dof_data = np.array([row[1:10] for row in raw_data], dtype=np.float64)
    rpy_data = np.array([row[10:13] for row in raw_data], dtype=np.float64)
    bno_data = np.array([row[13] for row in raw_data], dtype=np.int64)
    bmp_data = np.array([row[14] for row in raw_data], dtype=np.float64)
    pressure_data = np.array([row[15] for row in raw_data], dtype=np.float64)

    # Ensure ground-truth data is organized correctly: timestamp, position (x, y, z), and orientation (qx, qy, qz, qw)
    gt_timestamp = np.array([row[0] for row in gt_data], dtype=np.float64)
    gt_position = np.array([row[1:4] for row in gt_data], dtype=np.float64)  # Assuming x, y, z
    gt_orientation = np.array([row[4:8] for row in gt_data], dtype=np.float64)  # Assuming qx, qy, qz, qw

    # Creating a new .h5 file in the HDF5s folder
    file_path = os.path.join("HDF5s", f"{dataset_name}.h5")
    os.makedirs("HDF5s", exist_ok=True)  # Ensure the directory exists

    with h5py.File(file_path, 'w') as f:
        # Creating the RAWDATA group
        rawdata_group = f.create_group('RAWDATA')
        rawdata_group.create_dataset('TIMESTAMP', data=timestamp_data)
        rawdata_group.create_dataset('9DOF', data=nine_dof_data)
        rawdata_group.create_dataset('RPY', data=rpy_data)
        rawdata_group.create_dataset('BNO', data=bno_data)
        rawdata_group.create_dataset('BMP', data=bmp_data)
        rawdata_group.create_dataset('PRESSURE', data=pressure_data)

        # Creating the GT_DATA group with the specified structure
        gtdata_group = f.create_group('GT_DATA')
        gtdata_group.create_dataset('TIMESTAMP', data=gt_timestamp)
        gtdata_group.create_dataset('POSITION', data=gt_position)
        gtdata_group.create_dataset('ORIENTATION', data=gt_orientation)








def readHDF5(dataset_name):
    # File path to the HDF5 file
    file_path = os.path.join("HDF5s", f"{dataset_name}.h5")

    # Opening the HDF5 file in read mode
    with h5py.File(file_path, 'r') as f:
        # Accessing raw data under RAWDATA group
        timestamp_data = f['RAWDATA/TIMESTAMP'][:]
        nine_dof_data = f['RAWDATA/9DOF'][:]
        rpy_data = f['RAWDATA/RPY'][:]
        bno_data = f['RAWDATA/BNO'][:]
        bmp_data = f['RAWDATA/BMP'][:]
        pressure_data = f['RAWDATA/PRESSURE'][:]

        # Accessing ground truth data
        gt_position = f['Outputs/Position'][:]
        gt_orientation = f['Outputs/Orientation'][:]

    return timestamp_data, nine_dof_data, rpy_data, bno_data, bmp_data, pressure_data, gt_position, gt_orientation
