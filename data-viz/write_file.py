import h5py 
import numpy as np
import os

def write_to_blender(dataset_name, rt_data): 
    position= np.array([row[0:3] for row in rt_data], dtype=np.float64)  # Assuming x, y, z
    orientation = np.array([row[3:7] for row in rt_data], dtype=np.float64) #Assuming x,y,z,w!! In quat 
    
    file_path = os.path.join("data", f"{dataset_name}.h5")
    os.makedirs("data", exist_ok=True) 

    with h5py.File(file_path, 'w') as f:
        data_group = f.create_group('RT_DATA')
        data_group.create_dataset('POSITION', data=position)
        data_group.create_dataset('ORIENTATION', data = orientation)