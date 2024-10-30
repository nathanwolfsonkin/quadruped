import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import energy_model.quadruped_energy as quad
from energy_model.kinematics_data import angle_converter

class GaitAnimation:
    def __init__(self, quadruped, angles, angular_velocities, time):
        self.main_body = quadruped.body
        self.legs = quadruped.leg_list
        self.angles = angles
        self.angular_velocities = angular_velocities
        self.time = time

        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(10, 5))

        self.ax1.set_xlim(-5, 5)
        self.ax1.set_ylim(-3, 3)
        self.ax1.set_aspect('equal')

        self.ax2.set_xlim(0, time[-1])
        self.ax2.set_ylim(0, 40)

        # Create the line for the main body
        self.main_body_line = self.ax1.plot([], [], lw=4, color='black')[0]

        # Colors for the pendulums (with darker shades for new ones)
        self.colors = ['darkblue','darkblue','green', 'green','darkgreen','darkgreen','blue','blue']

        self.energy_lines = []
        labels = ['Potential Energy', 'Translational KE', 'Rotational KE', 'Total Energy']
        plot_colors = ['darkblue','darkgreen','green','blue']

        for i in range(len(quadruped.leg_list)):
            energy_line_set = []
            for label in labels:
                # UNCOMMENT FOR TOTAL ENERGY ONLY
                # if label == 'Total Energy':
                #     line, = self.ax2.plot([], [], lw=2, label=f'{label} (Leg {i + 1})', color=plot_colors[i])
                #     energy_line_set.append(line)
                line, = self.ax2.plot([], [], lw=2, label=f'{label} (Leg {i + 1})', color=plot_colors[i])
                energy_line_set.append(line)
            self.energy_lines.append(energy_line_set)


        self.energy_data = [[[], [], [], []] for _ in range(len(quadruped.leg_list))]

        # Define lines for each pendulum segment
        # Initialize the pendulum lines with the specified colors
        self.lines = []
        num_pendulum_lines = 2 * len(quadruped.leg_list)  # Two lines per pendulum
        for i in range(num_pendulum_lines):
            color = self.colors[i]
            line, = self.ax1.plot([], [], lw=2, color=color)
            self.lines.append(line)

        self.ax2.legend()

    def pend_init(self):
        self.first_run = True
        self.main_body_line.set_data([], [])
        for line in self.lines:
            line.set_data([], [])
        for energy_line_set in self.energy_lines:
            for energy_line in energy_line_set:
                energy_line.set_data([], [])
        return (self.main_body_line, *self.lines, *[line for energy_line_set in self.energy_lines for line in energy_line_set])

    def pend_update(self, frame):
        left_endpoint, right_endpoint = self.main_body.get_endpoints()

        # Update the main body line
        self.main_body_line.set_data([left_endpoint[0], right_endpoint[0]], [left_endpoint[1], right_endpoint[1]])

        if self.first_run == True:
            for index in range(len(self.energy_data)):
                self.energy_data[index] = [[], [], [], []]
                self.first_run = False

        # Update each pendulum
        for i, leg in enumerate(self.legs):
            if i < 2:
                leg.origin = left_endpoint.reshape(2, 1)
            else:
                leg.origin = right_endpoint.reshape(2, 1)

            leg.t1 = self.angles[i][0][frame]
            leg.t2 = self.angles[i][1][frame]
            leg.dt1 = self.angular_velocities[i][0][frame]
            leg.dt2 = self.angular_velocities[i][1][frame]

            pA = leg.get_pA().flatten()
            pB = leg.get_pB().flatten()

            # Update pendulum lines
            self.lines[2 * i].set_data([leg.origin[0, 0], pA[0]], [leg.origin[1, 0], pA[1]])
            self.lines[2 * i + 1].set_data([pA[0], pB[0]], [pA[1], pB[1]])

            potential_energy = leg.potential_energy()
            translational_ke = leg.translational_kinetic_energy()
            rotational_ke = leg.rotational_kinetic_energy()
            total_energy = leg.total_energy()

            # Reset energy lines when the system loops
            if frame == 0:
                for energy_line_set in self.energy_lines:
                    for energy_line in energy_line_set:
                        energy_line.set_data([], [])
            
            self.energy_data[i][0].append(potential_energy)
            self.energy_data[i][1].append(translational_ke)
            self.energy_data[i][2].append(rotational_ke)
            self.energy_data[i][3].append(total_energy)

            self.energy_lines[i][0].set_data(self.time[:frame + 1], self.energy_data[i][0])
            self.energy_lines[i][1].set_data(self.time[:frame + 1], self.energy_data[i][1])
            self.energy_lines[i][2].set_data(self.time[:frame + 1], self.energy_data[i][2])
            self.energy_lines[i][3].set_data(self.time[:frame + 1], self.energy_data[i][3])
            
            # Version for total energy only
            # self.energy_lines[i][0].set_data(self.time[:frame + 1], self.energy_data[i][3])
            

        return (self.main_body_line, *self.lines, *[line for energy_line_set in self.energy_lines for line in energy_line_set])

    def start_animation(self):
        pend_animation = FuncAnimation(self.fig, self.pend_update, frames=len(self.time),
                                       init_func=self.pend_init, blit=False, interval=10, repeat=True)
        plt.show()

def main():
    leg_params = {'l':[1,1], 'I':[1,1], 'm':[1,1]}
    body_params = {'l':3, 'I':1, 'm':1, 'origin':[0,0], 'orientation':0}
    quadruped = quad.Quadruped(leg_params=leg_params, body_params=body_params)
    
    theta_list = angle_converter.get_angle_lists()

    frames = len(theta_list[0][0])
    time = np.linspace(0, 5, frames) # Total time in seconds

    angles = [
        [theta_list[0][0], theta_list[0][1]],  # Original front pendulum
        [theta_list[1][0], theta_list[1][1]],  # Original rear pendulum
        [theta_list[1][0], theta_list[1][1]],  # New front pendulum (mirroring rear)
        [theta_list[0][0], theta_list[0][1]]   # New rear pendulum (mirroring front)
    ]

    # Angular velocities
    angular_velocities = [
        [np.gradient(angles[0][0], time), np.gradient(angles[0][1], time)],  # Original front
        [np.gradient(angles[1][0], time), np.gradient(angles[1][1], time)],  # Original rear
        [np.gradient(angles[2][0], time), np.gradient(angles[2][1], time)],  # New front
        [np.gradient(angles[3][0], time), np.gradient(angles[3][1], time)]   # New rear
    ]

    filtered_ang_vels = []
    for angular_vel_list in angular_velocities:
        filt_vel_x, filt_vel_y = angle_converter.moving_filter(angular_vel_list[0],angular_vel_list[1])
        filtered_ang_vels.append([filt_vel_x,filt_vel_y])

    animation = GaitAnimation(quadruped, angles, filtered_ang_vels, time)
    animation.start_animation()

if __name__ == "__main__":
    main()