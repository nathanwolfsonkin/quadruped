import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import glob
import os
import time

from energy_model.double_pendulum import DoublePendulum
from energy_model.double_pendulum import MainBody

class OverlayAnimation:
    def __init__(self):
        # Load frames saved by the OpenCV script
        frame_folder = "/workspace/src/energy_model/accuracy_check/video_frames"
        self.frame_files = sorted(glob.glob(os.path.join(frame_folder, "*.png")))

        # Define plot
        self.fig, self.ax = plt.subplots()

        # Create a DoublePendulum object
        # Initialize the DoublePendulum instance
        self.pendulum = DoublePendulum()
        
        # Define initial angles and angular velocities
        self.t1 = np.pi / 4  # initial angle for pendulum 1
        self.t2 = np.pi / 4  # initial angle for pendulum 2
        self.dt1 = 0.1       # initial angular velocity for pendulum 1
        self.dt2 = 0.1       # initial angular velocity for pendulum 2
        

        # Create animation
        ani = animation.FuncAnimation(self.fig, self.ani_update, frames=len(self.frame_files), init_func=self.ani_init, blit=False)
        plt.show()


    def draw_pendulum(self):
        # Calculate positions of the pendulum's centers of mass
        p1 = self.pendulum.get_p1(self.t1)
        pA = self.pendulum.get_pA(self.t1)
        p2 = self.pendulum.get_p2(self.t1, self.t2)
        pB = self.pendulum.get_pB(self.t1, self.t2)

        # Draw the pendulum as lines and points
        self.ax.plot([self.pendulum.origin[0], p1[0, 0]], [self.pendulum.origin[1], p1[1, 0]], 'ro-', label='Pendulum 1')  # First link
        self.ax.plot([p1[0, 0], pA[0, 0]], [p1[1, 0], pA[1, 0]], 'b-', label='Pendulum 1 Tip')  # First link tip
        self.ax.plot([pA[0, 0], p2[0, 0]], [pA[1, 0], p2[1, 0]], 'go-', label='Pendulum 2')  # Second link
        self.ax.plot([p2[0, 0], pB[0, 0]], [p2[1, 0], pB[1, 0]], 'g-', label='Pendulum 2 Tip')  # Second link tip

    def ani_init(self):
        # Determine aspect ratio
        img = plt.imread(self.frame_files[0])
        img_height, img_width, _ = img.shape
        aspect_ratio = img_width/img_height

        # Define numerical value for height
        self.plt_height = 6
        self.plt_width = aspect_ratio*self.plt_height

        self.ax.set_xlim(-self.plt_width/2, self.plt_width/2)
        self.ax.set_ylim(-self.plt_height/2, self.plt_height/2)

    def ani_update(self, frame_idx):
        # Load the corresponding frame image
        img = plt.imread(self.frame_files[frame_idx])
        self.ax.clear()
        self.ax.imshow(img, extent=[-self.plt_width / 2, self.plt_width / 2, -self.plt_height / 2, self.plt_height / 2])  # Display the image
        plt.pause(.001)

        # Draw the pendulum on top of the frame
        self.draw_pendulum()
        
        # Update the angles
        self.t1 += 0.05  # Increment angle 1
        self.t2 += 0.05  # Increment angle 2

        plt.pause(0.001)
        return self.ax

def main():
    OverlayAnimation()


if __name__ == "__main__":
    main()