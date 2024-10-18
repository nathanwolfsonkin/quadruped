import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import glob
import os
import time

from energy_model.double_pendulum import DoublePendulum
from energy_model.kinematics_data import angle_converter

class OverlayAnimation:
    def __init__(self):
        # Load frames saved by the OpenCV script
        frame_folder = "/workspace/src/energy_model/accuracy_check/video_frames"
        self.frame_files = sorted(glob.glob(os.path.join(frame_folder, "*.png")))

        # Define plot
        self.fig, self.ax = plt.subplots()

        # Create a DoublePendulum object
        # Initialize the DoublePendulum instance
        self.pendulum = DoublePendulum(l=[1.9,2])

        # Generate list of angles wrt frame
        self.theta_list = angle_converter.get_angle_lists()
        
        # Obtain list of hip positions
        front_hip_filepath = "/workspace/src/energy_model/kinematics_data/unitree_a1/front_hip.csv"
        rear_hip_filepath = "/workspace/src/energy_model/kinematics_data/unitree_a1/rear_hip.csv"
        self.front_hip_pos_list = angle_converter.get_positions(front_hip_filepath)
        self.rear_hip_pos_list = angle_converter.get_positions(rear_hip_filepath)

        self.t1 = self.theta_list[0][0][0]
        self.t2 = self.theta_list[0][1][0]
        self.dt1 = 0.1       # initial angular velocity for pendulum 1
        self.dt2 = 0.1       # initial angular velocity for pendulum 2

        # Create animation
        ani = animation.FuncAnimation(self.fig, self.ani_update, frames=len(self.frame_files), init_func=self.ani_init, blit=False)
        plt.show()


    def draw_pendulum(self, frame_idx):
        # Update origin position
        front_hip_x = self.front_hip_pos_list[0][frame_idx]
        front_hip_y = -self.front_hip_pos_list[1][frame_idx]

        # Scale to plot size
        scaled_front_hip_x = self.scaling_factor*front_hip_x
        scaled_front_hip_y = self.scaling_factor*front_hip_y

        # Reassign position of hips
        self.pendulum.origin = np.array([scaled_front_hip_x,scaled_front_hip_y]).reshape(2, 1)

        # Calculate positions of the pendulum's centers of mass
        pA = self.pendulum.get_pA(self.t1)
        pB = self.pendulum.get_pB(self.t1, self.t2)

        # Draw the pendulum as lines
        self.ax.plot([self.pendulum.origin[0], pA[0]], [self.pendulum.origin[1], pA[1]], 'b-')  # First link tip
        self.ax.plot([pA[0], pB[0]], [pA[1], pB[1]], 'g-')  # Second link tip
        plt.pause(.01)
        pass


    def ani_init(self):
        # Determine aspect ratio
        img = plt.imread(self.frame_files[0])
        img_height, img_width, _ = img.shape
        aspect_ratio = float(img_width)/float(img_height)
        self.img_height = img_height

        # Define numerical value for height
        self.plt_height = 6
        self.plt_width = aspect_ratio*self.plt_height

        self.scaling_factor = self.plt_height / 1080.0

    def ani_update(self, frame_idx):
        # Load the corresponding frame image
        img = plt.imread(self.frame_files[frame_idx])
        self.ax.clear()
        # self.ax.imshow(img, extent=[-self.plt_width / 2, self.plt_width / 2, -self.plt_height / 2, self.plt_height / 2])  # Display the image
        self.ax.imshow(img, extent=[0, self.plt_width, -self.plt_height, 0])  # Display the image
        plt.pause(.001)

        # Draw the pendulum on top of the frame
        self.draw_pendulum(frame_idx)

        # Update the angles
        self.t1 = self.theta_list[0][0][frame_idx]
        self.t2 = self.theta_list[0][1][frame_idx]

        return self.ax

def main():
    my_ani = OverlayAnimation()
    # my_ani.ani_init()
    # for i in range(298):
    #     my_ani.ani_update(i)
    #     plt.pause(.01)


if __name__ == "__main__":
    main()