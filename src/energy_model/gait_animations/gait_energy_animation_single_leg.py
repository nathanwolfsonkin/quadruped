import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import energy_model.quadruped_energy as quad
from energy_model.kinematics_data import angle_converter

class GaitAnimation:
    def __init__(self, quadruped, angles, angular_velocities, time, leg_index=0):
        # Initial setup
        self.quadruped = quadruped
        self.angles = angles
        self.angular_velocities = angular_velocities
        self.time = time
        self.leg_index = leg_index

        # Create figure and subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(10, 5))
        
        # Set up quadruped position plot
        self.ax1.set_xlim(-1, 1)
        self.ax1.set_ylim(-0.5, 0.5)
        self.ax1.set_aspect('equal')
        
        # Set up leg energy plot
        self.ax2.set_xlim(0, time[-1])
        self.ax2.set_ylim(0, 2)  # Adjust range based on expected leg energy values

        # Ground line
        self.ground_line, = self.ax1.plot([-5, 5], [-.27, -.27], lw=2, color='black')

        # Main body line
        self.main_body_line, = self.ax1.plot([], [], lw=4, color='black')

        # Initialize leg link lines
        self.leg_colors = ['red', 'red', 'darkblue', 'darkblue', 'darkblue', 'darkblue', 'darkblue', 'darkblue']
        
        # Initialize leg link lines with specified colors
        self.leg_links = []
        for color in self.leg_colors:
            line, = self.ax1.plot([], [], lw=2, color=color)
            self.leg_links.append(line)


        # Line for leg energy
        self.leg_energy_line, = self.ax2.plot([], [], lw=2, label=f'Leg {leg_index} Energy', color='red')
        self.ax2.legend()

    def pend_init(self):
        # Clear data for each animation cycle
        self.main_body_line.set_data([], [])
        for link in self.leg_links:
            link.set_data([], [])
        self.leg_energy_line.set_data([], [])
        self.leg_energy_data = []
        return [self.main_body_line, *self.leg_links, self.leg_energy_line]

    def pend_update(self, frame):
        # Update main body
        left_endpoint, right_endpoint = self.quadruped.body.get_endpoints()
        self.main_body_line.set_data([left_endpoint[0], right_endpoint[0]], [left_endpoint[1], right_endpoint[1]])

        # Update each leg's angles and set their positions
        for i, leg in enumerate(self.quadruped.leg_list):
            leg.origin = left_endpoint.reshape(2, 1) if i < 2 else right_endpoint.reshape(2, 1)
            leg.t1, leg.t2 = self.angles[i][0][frame], self.angles[i][1][frame]
            leg.dt1, leg.dt2 = self.angular_velocities[i][0][frame], self.angular_velocities[i][1][frame]

            # Update leg lines
            pA, pB = leg.get_pA().flatten(), leg.get_pB().flatten()
            self.leg_links[2 * i].set_data([leg.origin[0, 0], pA[0]], [leg.origin[1, 0], pA[1]])
            self.leg_links[2 * i + 1].set_data([pA[0], pB[0]], [pA[1], pB[1]])

        # Calculate and plot energy for the specified leg
        leg_energy = self.quadruped.leg_list[self.leg_index].total_energy()
        self.leg_energy_data.append(leg_energy)
        self.leg_energy_line.set_data(self.time[:frame + 1], self.leg_energy_data)

        return [self.main_body_line, *self.leg_links, self.leg_energy_line]

    def start_animation(self):
        pend_animation = FuncAnimation(self.fig, self.pend_update, frames=len(self.time),
                                       init_func=self.pend_init, blit=False, interval=10, repeat=True)
        plt.show()

def main():
    # Parameters for the quadruped
    leg_params = {'l': [0.2, 0.2], 'I': [0.0055, 0.003], 'm': [1.013, 0.166]}
    body_params = {'l': 0.361, 'I': 0.01, 'm': 5.66, 'origin': [0, 0], 'orientation': 0}
    
    quadruped = quad.Quadruped(leg_params=leg_params, body_params=body_params)
    theta_list = angle_converter.get_angle_lists()

    frames = len(theta_list[0][0])
    time = np.linspace(0, 5, frames)

    angles = [
        [theta_list[0][0], theta_list[0][1]],
        [theta_list[1][0], theta_list[1][1]],
        [theta_list[1][0], theta_list[1][1]],
        [theta_list[0][0], theta_list[0][1]]
    ]

    angular_velocities = [
        [np.gradient(angles[0][0], time), np.gradient(angles[0][1], time)],
        [np.gradient(angles[1][0], time), np.gradient(angles[1][1], time)],
        [np.gradient(angles[2][0], time), np.gradient(angles[2][1], time)],
        [np.gradient(angles[3][0], time), np.gradient(angles[3][1], time)]
    ]

    filtered_ang_vels = [angle_converter.moving_filter(*vel) for vel in angular_velocities]

    animation = GaitAnimation(quadruped, angles, filtered_ang_vels, time, leg_index=0)
    animation.start_animation()

if __name__ == "__main__":
    main()
