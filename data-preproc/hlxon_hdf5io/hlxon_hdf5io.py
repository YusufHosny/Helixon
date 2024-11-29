import h5py 
import numpy as np
import os

def storeAsHDF5(dataset_name, raw_data, gt_data, wifidata):
    # Grouping raw data based on the specified structure
    timestamp_data = np.array([row[0] for row in raw_data], dtype=np.float64)
    nine_dof_data = np.array([row[1:10] for row in raw_data], dtype=np.float64)
    rpy_data = np.array([row[10:13] for row in raw_data], dtype=np.float64)
    bno_data = np.array([row[13] for row in raw_data], dtype=np.int64)
    bmp_data = np.array([row[14] for row in raw_data], dtype=np.float64)
    pressure_data = np.array([row[15] for row in raw_data], dtype=np.float64)

    # group wifi data
    wifi_ts_data = np.array([row[0] for row in wifidata])
    count_data = np.array([row[1] for row in wifidata])
    bssid_data = np.array([row[i] for row in wifidata for i in range(2, len(row), 2)], dtype=h5py.string_dtype(encoding='utf-8', length=None))
    rssi_data = np.array([row[i] for row in wifidata for i in range(3, len(row), 2)])
    
    # Ensure ground-truth data is organized correctly: timestamp, position (x, y, z), and orientation (qx, qy, qz, qw)
    gt_timestamp = np.array([row[0] for row in gt_data], dtype=np.float64)
    gt_position = np.array([row[1:4] for row in gt_data], dtype=np.float64)  # Assuming x, y, z
    gt_orientation = np.array([row[4:8] for row in gt_data], dtype=np.float64)  # Assuming qx, qy, qz, qw

    # Creating a new .h5 file in the HDF5s folder
    file_path = os.path.join("data", f"{dataset_name}.h5")
    os.makedirs("data", exist_ok=True)  # Ensure the directory exists

    with h5py.File(file_path, 'w') as f:
        # Creating the RAWDATA group
        rawdata_group = f.create_group('RAWDATA')
        rawdata_group.create_dataset('TIMESTAMP', data=timestamp_data)
        rawdata_group.create_dataset('9DOF', data=nine_dof_data)
        rawdata_group.create_dataset('RPY', data=rpy_data)
        rawdata_group.create_dataset('BNO', data=bno_data)
        rawdata_group.create_dataset('BMP', data=bmp_data)
        rawdata_group.create_dataset('PRESSURE', data=pressure_data)

        # create WIFIDATA group
        wifidata_group = f.create_group('WIFIDATA')
        wifidata_group.create_dataset('TIMESTAMP', data=wifi_ts_data)
        wifidata_group.create_dataset('COUNTS', data=count_data)
        wifidata_group.create_dataset('BSSIDS', data=bssid_data)
        wifidata_group.create_dataset('RSSIS', data=rssi_data)


        # Creating the GT_DATA group with the specified structure
        gtdata_group = f.create_group('GT_DATA')
        gtdata_group.create_dataset('TIMESTAMP', data=gt_timestamp)
        gtdata_group.create_dataset('POSITION', data=gt_position)
        gtdata_group.create_dataset('ORIENTATION', data=gt_orientation)

def readHDF5(dataset_name):
    # File path to the HDF5 file
    file_path = os.path.join("data", f"{dataset_name}.h5")

    # Opening the HDF5 file in read mode
    with h5py.File(file_path, 'r') as f:
        # Accessing raw data under RAWDATA group
        raw_timestamp = f['RAWDATA/TIMESTAMP'][:]
        raw_nine_dof = f['RAWDATA/9DOF'][:]
        raw_rpy = f['RAWDATA/RPY'][:]
        raw_bno = f['RAWDATA/BNO'][:]
        raw_bmp = f['RAWDATA/BMP'][:]
        raw_pressure = f['RAWDATA/PRESSURE'][:]

        # add wifi data
        wifi_timestamp_data =  f['WIFIDATA/TIMESTAMP']
        count_data = f['WIFIDATA/COUNTS']
        ssid_data = f['WIFIDATA/BSSIDS']
        rssi_data = f['WIFIDATA/RSSIS']

        # format wifi data
        wifidata = []
        offset = 0
        for i, ti in enumerate(wifi_timestamp_data):
            row = [ti, count_data[i]]
            for _ in range(count_data[i]):
                row += [ssid_data[offset], rssi_data[offset]]
                offset += 1
            wifidata += [row]

        # Accessing ground truth data
        gt_timestamp = f['GT_DATA/TIMESTAMP'][:]
        gt_position = f['GT_DATA/POSITION'][:]
        gt_orientation = f['GT_DATA/ORIENTATION'][:]

    return raw_timestamp, raw_nine_dof, raw_rpy, raw_bno, raw_bmp, raw_pressure, \
        wifidata,  \
        gt_timestamp, gt_position, gt_orientation

def readAll():
    files = os.listdir(os.path.join(os.path.curdir, 'data'))
    output = []
    for file in files:
        output += [readHDF5(file.replace('.h5', ''))]
    return output