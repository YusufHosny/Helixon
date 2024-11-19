import os
from os import listdir
import sys
import numpy as np
from led_detector import ImageBrightnessDetect
from hlxon_hdf5io.hlxon_hdf5io import *


# Creates an array of images to detect the brightness of the LED (for synchronization purposes)
def imageIterator(dataset_name):

    folder_dir = rf"Measurements\{dataset_name}\ImageSet"

    # Ordered array of images
    image_files = sorted(
        [img for img in os.listdir(folder_dir) if img.endswith(".jpg")],
        key=lambda x: int(x.split('.')[0])  
    )

    led_values = []

    # Iterating through each image and extracting brightness
    for image in image_files:

        if image.endswith(".jpg"):
            
            image_path = os.path.join(folder_dir, image)

            # Creating an instance of ImageBrightnessDetect with the image path
            brightness_detector = ImageBrightnessDetect(image_path)
            brightness_level = brightness_detector.classify()

            led_values.append(brightness_level)
    
    return led_values


# Creates an array with timestamp, x, y, z, qw, qy, qz, qw coordinates
def gtPoseIterator(dataset_name):

    folder_dir = rf"Measurements\{dataset_name}\poses.txt"

    gt_data = []

    with open(folder_dir, 'r') as file:

        for line in file:

            # We skip the comment lines or empty lines
            if line.startswith("#") or not line.strip():
                continue
            
            # Converting each line into a list of float numbers and appending it to the data list
            row = list(map(float, line.strip().split()))

            gt_data.append(row)


    # Converting the list to a numpy array 
    gt_data = np.array(gt_data)

    return gt_data


# Creates an array of raw data with a direct 19-element structure per row
def rawDataIterator(dataset_name):
    path = rf"Measurements\{dataset_name}\rawdata.txt"
    
    raw_data = []

    with open(path, 'r') as file:
        # Skip the header line if it exists
        header = file.readline().strip().split(',')
        
        # Check if the header contains non-numeric entries
        if any(not element.replace('.', '', 1).isdigit() for element in header):
            # If it's a header, skip it and proceed with the next line
            pass
        else:
            # If the first line is not a header, process it
            row_data = list(map(float, header[:19]))
            raw_data.append(row_data)

        # Process the rest of the lines
        for line in file:
            # The elements in the .txt file are separated by commas
            elements = line.strip().split(',')
            
            # Convert elements to float and create a row with exactly 19 elements
            row_data = list(map(float, elements[:19]))

            # Append the row directly to raw_data
            raw_data.append(row_data)

    # Convert raw_data to a numpy array with a consistent dtype
    raw_data = np.array(raw_data, dtype=np.float64)

    return raw_data


def labelAndStore(dataset_name):

    led_values = imageIterator(dataset_name)

    filtered_gt_data = gtPoseIterator(dataset_name)
    filtered_raw_data = rawDataIterator(dataset_name)

    led_values = led_values[200:]
    filtered_gt_data = filtered_gt_data[200:]

    # Create a mask for brightness levels 7 or above, truncating if necessary
    mask = np.array(led_values[:filtered_gt_data.shape[0]]) > 7
    
    # Apply the mask to filter out entries in filtered_gt_data and filtered_raw_data
    filtered_gt_data = filtered_gt_data[mask]

    # Normalize timestamps in filtered_gt_data
    gt_initial_timestamp = filtered_gt_data[0, 0]
    filtered_gt_data[:, 0] -= gt_initial_timestamp

    # Normalize timestamps in filtered_raw_data
    raw_initial_timestamp = filtered_raw_data[0, 0]
    filtered_raw_data[:, 0] -= raw_initial_timestamp

    # Extract timestamps
    gt_timestamps = filtered_gt_data[:, 0]  # Assuming timestamp is the first column in filtered_gt_data
    raw_timestamps = filtered_raw_data[:, 0] # Assuming timestamp is the first column in filtered_raw_data
    filtered_gt_data[:, 0] = (filtered_gt_data[:, 0] * 1e6).astype(int)

    storeAsHDF5(dataset_name, filtered_raw_data, filtered_gt_data)

    

# When running the file, we set as parameter the name of the dataset folder
if __name__ == "__main__":

    # We first check if an argument is passed
    if len(sys.argv) != 2:
        print("Usage: python dataLabeller.py <database_name>")

    # If an argument is passed we call labelAndStore()
    else:
        
        dataset_name = sys.argv[1]
        labelAndStore(dataset_name)