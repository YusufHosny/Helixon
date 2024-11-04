import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from utils import rot_vect

from typing import Self

INITIAL_VECTOR_R = np.array([1., 0., 0.])  # Start with a vector along the x-axis
INITIAL_VECTOR_G = np.array([0., 1., 0.])  # Start with a vector along the y-axis
INITIAL_VECTOR_B = np.array([0., 0., 1.])  # Start with a vector along the z-axis


class Visualizer:

    def __init__(self: Self):
        self.quat = np.array([1., 0., 0., 0.])
        
        def qset(quat):
            self.quat = quat
            
        self.quatcallback = qset

    def show(self: Self, ) -> None:
        # Initialize the figure and axis
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Draw the initial vectors
        vecr = ax.quiver(0, 0, 0, *INITIAL_VECTOR_R, color='red')
        vecg = ax.quiver(0, 0, 0, *INITIAL_VECTOR_G, color='green')
        vecb = ax.quiver(0, 0, 0, *INITIAL_VECTOR_B, color='blue')

        # Function to update the plot animation
        def update_animation(frame,):
            rotation_quaternion = self.quat
            rotated_vector_r = rot_vect(INITIAL_VECTOR_R, rotation_quaternion)
            rotated_vector_g = rot_vect(INITIAL_VECTOR_G, rotation_quaternion)
            rotated_vector_b = rot_vect(INITIAL_VECTOR_B, rotation_quaternion)
            
            # Update the vectors in the plot
            ax.clear()
            ax.set_xlim(-2, 2)
            ax.set_ylim(-2, 2)
            ax.set_zlim(-2, 2)
            vecr = ax.quiver(0, 0, 0, *rotated_vector_r, color='red')
            vecg = ax.quiver(0, 0, 0, *rotated_vector_g, color='green')
            vecb = ax.quiver(0, 0, 0, *rotated_vector_b, color='blue')

        # Create the animation
        ani = FuncAnimation(fig, update_animation, frames=np.arange(0, 360, 5), interval=10)

        plt.show()
