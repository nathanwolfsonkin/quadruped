import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class MainBody:
    def __init__(self, m=5, l=3, I=2, origin=[0, 0], orientation=0):
        self.m = m
        self.l = l
        self.I = I
        self.origin = np.array(origin).reshape(2, 1)
        self.orientation = orientation

    def get_endpoints(self):
        half_length = self.l / 2
        R = np.array([[np.cos(self.orientation), -np.sin(self.orientation)],
                      [np.sin(self.orientation), np.cos(self.orientation)]])
        left_endpoint = self.origin + R @ np.array([[-half_length], [0]])
        right_endpoint = self.origin + R @ np.array([[half_length], [0]])
        return left_endpoint.flatten(), right_endpoint.flatten()

class DoublePendulum:
    def __init__(self, m=[1,1], l=[1,1], I=[1,1], origin=[0,0]):
        # Initialize Parameters
        self.m = m
        self.l = l
        self.I = I
        self.origin = origin

    # positional functions
    # returns position of com1 wrt the origin
    def get_p1(self,t1):
        return self.origin + (self.l[0]/2)*np.array([[np.sin(t1)],
                                                     [-np.cos(t1)]])

    def get_pA(self,t1):
        return self.origin + self.l[0]*np.array([[np.sin(t1)],
                                                 [-np.cos(t1)]])

    # returns position of com2 wrt the origin
    def get_p2(self, t1, t2):
        return self.get_pA(t1) + (self.l[1]/2)*np.array([[np.sin(t1+t2)],
                                                       [-np.cos(t1+t2)]])
    
    def get_pB(self, t1, t2):
        return self.get_pA(t1) + self.l[1]*np.array([[np.sin(t1+t2)],
                                                   [-np.cos(t1+t2)]])

    # velocity functions
    def get_v1(self, t1,dt1):
        v = (self.l[0]/2)*np.array([[np.cos(t1)],
                            [np.sin(t1)]]) * dt1
        return v

    def get_vA(self,t1,dt1):
        v = self.l[0]*np.array([[np.cos(t1)],
                        [np.sin(t1)]]) * dt1
        return v
    
    def get_v2(self,t1,dt1,t2,dt2):
        v = self.get_vA(t1,dt1) + (self.l[1]/2)*np.array([[np.cos(t1+t2)],
                                            [np.sin(t1+t2)]])*(dt1+dt2)
        return v
        
    # energy functions
    def potential_energy(self,t1, t2):
        g = 9.81
        p1 = self.get_p1(t1)
        p2 = self.get_p2(t1,t2)

        h1 = p1[1]
        h2 = p2[1]

        u1 = self.m[0]*g*h1
        u2 = self.m[1]*g*h2
        return u1+u2

    def translational_kinetic_energy(self,t1, dt1, t2, dt2):
        v1 = self.get_v1(t1, dt1)
        v2 = self.get_v2(t1, dt1, t2, dt2)

        kt1 = .5*self.m[0]*(v1.T@v1)
        kt2 = .5*self.m[1]*(v2.T@v2)
        return kt1+kt2

    def rotational_kinetic_energy(self,dt1,dt2):
        kw1 = .5*self.I[0]*dt1**2
        kw2 = .5*self.I[1]*(dt1+dt2)**2
        return kw1+kw2
    
    def total_energy(self,t1,t2,dt1,dt2):
        return (self.potential_energy(t1,t2) + 
                self.translational_kinetic_energy(t1,dt1,t2,dt2) + 
                self.rotational_kinetic_energy(dt1,dt2)
                ).item()

class PendulumAnimation:
    def __init__(self, main_body, pendulums, angles, angular_velocities, time):
        self.main_body = main_body
        self.pendulums = pendulums
        self.angles = angles
        self.angular_velocities = angular_velocities
        self.time = time

        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(10, 5))

        self.ax1.set_xlim(-5, 5)
        self.ax1.set_ylim(-3, 3)
        self.ax1.set_aspect('equal')

        self.ax2.set_xlim(0, time[-1])
        self.ax2.set_ylim(-40, 40)

        # Create the line for the main body
        self.main_body_line = self.ax1.plot([], [], lw=4, color='black')[0]

        # Set the colors for the pendulums
        self.colors = ['blue', 'green']  # top link: blue, bottom link: green

        self.energy_lines = [[self.ax2.plot([], [], lw=2, label=f'{label} (Pendulum {i + 1})')[0]
                              for label in ['Potential Energy', 'Translational KE', 'Rotational KE', 'Total Energy']]
                              for i in range(len(pendulums))]

        self.energy_data = [[[], [], [], []] for _ in range(len(pendulums))]

        # Set the colors for each pendulum's top and bottom links
        self.lines = [self.ax1.plot([], [], lw=2, color=self.colors[i % 2])[0] for i in range(2 * len(pendulums))]

        self.ax2.legend()

    def pend_init(self):
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

        for i, pendulum in enumerate(self.pendulums):
            if i == 0:
                pendulum.origin = left_endpoint.reshape(2, 1)
            else:
                pendulum.origin = right_endpoint.reshape(2, 1)

            t1_frame = self.angles[i][0][frame]
            t2_frame = self.angles[i][1][frame]
            dt1_frame = self.angular_velocities[i][0][frame]
            dt2_frame = self.angular_velocities[i][1][frame]

            pA = pendulum.get_pA(t1_frame).flatten()
            pB = pendulum.get_pB(t1_frame, t2_frame).flatten()

            # Update the pendulum lines to extend to point B
            self.lines[2 * i].set_data([pendulum.origin[0, 0], pA[0]], [pendulum.origin[1, 0], pA[1]])  # First link (top)
            self.lines[2 * i + 1].set_data([pA[0], pB[0]], [pA[1], pB[1]])  # Second link (bottom)

            potential_energy = pendulum.potential_energy(t1_frame, t2_frame)
            translational_ke = pendulum.translational_kinetic_energy(t1_frame, dt1_frame, t2_frame, dt2_frame)
            rotational_ke = pendulum.rotational_kinetic_energy(dt1_frame, dt2_frame)
            total_energy = potential_energy + translational_ke + rotational_ke

            self.energy_data[i][0].append(potential_energy)
            self.energy_data[i][1].append(translational_ke)
            self.energy_data[i][2].append(rotational_ke)
            self.energy_data[i][3].append(total_energy)

            self.energy_lines[i][0].set_data(self.time[:frame + 1], self.energy_data[i][0])
            self.energy_lines[i][1].set_data(self.time[:frame + 1], self.energy_data[i][1])
            self.energy_lines[i][2].set_data(self.time[:frame + 1], self.energy_data[i][2])
            self.energy_lines[i][3].set_data(self.time[:frame + 1], self.energy_data[i][3])

        return (self.main_body_line, *self.lines, *[line for energy_line_set in self.energy_lines for line in energy_line_set])

    def start_animation(self):
        pend_animation = FuncAnimation(self.fig, self.pend_update, frames=len(self.time),
                                       init_func=self.pend_init, blit=True, interval=10, repeat=False)
        plt.show()

def main():
    main_body = MainBody(origin=[0, 0], orientation=0)
    pendulums = [DoublePendulum(), DoublePendulum()]

    frames = 200
    time = np.linspace(0, 20, frames)

    amp1 = 1.0
    amp2 = 0.4

    phase1 = 0
    phase2 = np.pi/2

    leg1_angles = [
        amp1 * np.sin(time+phase1) - np.pi/6,
        amp2 * np.sin(time+phase1) + np.pi/8
    ]

    leg2_angles = [
        amp1 * np.sin(time+phase2) - np.pi/8,
        amp2 * np.sin(time+phase2) + np.pi/10
    ]

    angles = [
        leg1_angles,  # First pendulum (FL and BR)
        leg2_angles   # Second pendulum (FR and BL)
    ]

    # Angular velocities (derivatives of angles)
    angular_velocities = [
        [np.gradient(angles[0][0], time), np.gradient(angles[0][1], time)],  # FL and BR velocities
        [np.gradient(angles[1][0], time), np.gradient(angles[1][1], time)]   # FR and BL velocities
    ]

    animation = PendulumAnimation(main_body, pendulums, angles, angular_velocities, time)
    animation.start_animation()
    pass

if __name__ == "__main__":
    main()