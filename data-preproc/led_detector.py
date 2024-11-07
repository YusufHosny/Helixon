# The Image Brightness Detection.
#
# References:
# http://alienryderflex.com/hsp.html
# https://www.whydomath.org/node/wavlets/imagebasics.html
#
# We say the above formula for y is a convex combination of r', g',
# and b' since the multipliers .299, .587, and .114 are non-negative and sum to one. Actually, this is exactly the
# formula suggested by the National Television System Committee (NTSC) for converting color feeds to black and white
# televisions sets.
#
# Notes:
# Develop a generalized algorithm to detect the brightness of any image. Your algorithm should take an image
# as input and give a score between (0-10) as output (zero being low bright and 10 being high bright).
#
#
# See the README file for information on usage.

import math
import sys
from pathlib import Path
import argparse

import numpy as np
from PIL import Image, ImageStat


def create_arg_parser():
    """"
    Creates and returns the ArgumentParser object.
    """
    parser = argparse.ArgumentParser(description='Calculates image brightness on scale of 1 to 10.')
    parser.add_argument("--input_path", type=Path, help="Input image path")
    return parser


class ImageBrightnessDetect(object):
    """
    Input: Input RGB image file path
    Output: Level of brightness 1 to 10 (0 being low and 10 being the brightest)
    """

    def __init__(self, image_path):
        self.__image_path = image_path

    def classify(self):
        # File existence check
        my_file = Path(self.__image_path)
        if my_file.is_file():
            image = Image.open(self.__image_path)
            image.resize((1280, 720), Image.Resampling.LANCZOS)
            return self.__calculate_level(self.__crop_bottom_left(image))
        else:
            return "File does not exist"
    
    # def __crop_top_right(self, image):
    #     # Get image dimensions
    #     width, height = image.size

    #     # Calculate the size of the bottom-left 1/8th of the image
    #     crop_width = width // 2  # Half of the width
    #     crop_height = height // 4  # One-fourth of the height (1/8th total)

    #     # Define the box to crop (left, upper, right, lower)
    #     crop_box = (width - crop_width, 0, width, crop_height)

    #     # Crop and return the bottom-left 1/8th of the image
    #     return image.crop(crop_box)

    # def __crop_bottom_right(self, image):
    #     # Get image dimensions
    #     width, height = image.size

    #     # Calculate the dimensions for the crop (1/5th of width, 1/6th of height)
    #     crop_width = width // 5
    #     crop_height = height // 6

    #     # Define the box to crop (left, upper, right, lower)
    #     crop_box = (width - crop_width, height - crop_height, width, height)

    #     # Crop and return the bottom-right section of the image
    #     return image.crop(crop_box)

    def __crop_bottom_left(self, image):
        # Get image dimensions
        width, height = image.size

        # Calculate the dimensions for the crop (1/5th of width, 1/6th of height)
        crop_width = width // 5
        crop_height = height // 6

        # Define the box to crop (left, upper, right, lower) for the bottom-left
        crop_box = (0, height - crop_height, crop_width, height)

        # Crop and return the bottom-left section of the image
        return image.crop(crop_box)


    @staticmethod
    def __calculate_level(image):
        # Creating bins for 10 levels between 0 to 255
        levels = np.linspace(0, 255, num=10)

        # Get average pixel level for each layer
        image_stats = ImageStat.Stat(image)
        blue_channel_mean = image_stats.mean[2]  # Index 2 corresponds to the blue channel

        # Use the blue channel mean to determine brightness
        image_bright_value = blue_channel_mean  # We're now directly using the blue channel mean as the brightness value

        # Determine the brightness level based on blue channel mean
        image_bright_level = np.digitize(image_bright_value, levels, right=True)

        return image_bright_level

