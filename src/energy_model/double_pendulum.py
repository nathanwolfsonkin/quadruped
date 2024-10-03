import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class DoublePendulum:
    def __init__(self, m=[1,1], l=[1,1], I=[1,1]):
        # Initialize Parameters
        self.m = m
        self.l = l
        self.I = I

    # positional functions
    # returns position of com1 wrt the origin
    def get_p1(self,t1):
        return (self.l[0]/2)*np.array([[np.sin(t1)],
                                [-np.cos(t1)]])

    def get_pA(self,t1):
        return self.l[0]*np.array([[np.sin(t1)],
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
        #Get positions [x,y] from origin
        p1 = self.get_p1(t1)
        p2 = self.get_p2(t1,t2)

        h1 = p1[1]
        h2 = p2[1]

        u1 = -self.m[0]*g*h1
        u2 = -self.m[1]*g*h2
        return u1+u2

    def translational_kinetic_energy(self,t1, dt1, t2, dt2):
        v1 = self.get_v1(t1, dt1)
        v2 = self.get_v2(t1, dt1, t2, dt2)

        kt1 = .5*self.m[0]*(v1.T@v1)
        kt2 = .5*self.m[1]*(v2.T@v2)
        return kt1+kt2

    def rotational_kinetic_energy(self,dt1,dt2):
        # Parallel axis theorem since I2 is defined from connection point at A
        I2_offset = self.I[1] + self.m[1]*self.l[0]**2

        kw1 = .5*self.I[0]*dt1**2
        kw2 = .5*I2_offset*(dt1+dt2)**2
        return kw1+kw2
    
    def total_energy(self,t1,t2,dt1,dt2):
        return (self.potential_energy(t1,t2) + 
                self.translational_kinetic_energy(t1,dt1,t2,dt2) + 
                self.rotational_kinetic_energy(dt1,dt2)
                ).item()
    
class PendulumAnimation:
    def __init__(self, pendulum, t1, t2, dt1, dt2, time):
        self.pendulum = pendulum
        self.t1 = t1
        self.t2 = t2
        self.dt1 = dt1
        self.dt2 = dt2
        self.time = time

        # Create figure with two subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(10, 5))

        # Pendulum plot setup
        self.ax1.set_xlim(-2, 2)
        self.ax1.set_ylim(-2, 2)
        self.ax1.set_aspect('equal')

        # Energy plot setup
        self.ax2.set_xlim(0, time[-1])
        self.ax2.set_ylim(-40, 40)
        self.energy_lines = [self.ax2.plot([], [], lw=2, label=label)[0] 
                             for label in ['Potential Energy', 'Translational KE', 'Rotational KE', 'Total Energy']]
        self.potential_energy_data = []
        self.translational_ke_data = []
        self.rotational_ke_data = []
        self.total_energy_data = []

        # Pendulum lines
        self.line1, = self.ax1.plot([], [], lw=2)
        self.line2, = self.ax1.plot([], [], lw=2)
        
        # Add a legend to the energy plot
        self.ax2.legend()

    # Initialization function
    def pend_init(self):
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        for line in self.energy_lines:
            line.set_data([], [])
        return self.line1, self.line2, *self.energy_lines

    # Update function
    def pend_update(self, frame):
        t1_frame = self.t1[frame]
        t2_frame = self.t2[frame]
        dt1_frame = self.dt1[frame]
        dt2_frame = self.dt2[frame]

        # Update pendulum positions
        pA = self.pendulum.get_pA(t1_frame).flatten()
        p2 = self.pendulum.get_p2(t1_frame, t2_frame).flatten()

        self.line1.set_data([0, pA[0]], [0, pA[1]])
        self.line2.set_data([pA[0], p2[0]], [pA[1], p2[1]])

        # Calculate and update energy data
        potential_energy = self.pendulum.potential_energy(t1_frame, t2_frame).item()
        translational_ke = self.pendulum.translational_kinetic_energy(t1_frame, dt1_frame, t2_frame, dt2_frame).item()
        rotational_ke = self.pendulum.rotational_kinetic_energy(dt1_frame, dt2_frame).item()
        total_energy = potential_energy + translational_ke + rotational_ke
        
        self.potential_energy_data.append(potential_energy)
        self.translational_ke_data.append(translational_ke)
        self.rotational_ke_data.append(rotational_ke)
        self.total_energy_data.append(total_energy)

        # Update energy plot
        self.energy_lines[0].set_data(self.time[:frame+1], self.potential_energy_data)
        self.energy_lines[1].set_data(self.time[:frame+1], self.translational_ke_data)
        self.energy_lines[2].set_data(self.time[:frame+1], self.rotational_ke_data)
        self.energy_lines[3].set_data(self.time[:frame+1], self.total_energy_data)

        return self.line1, self.line2, *self.energy_lines

    # Start animation
    def start_animation(self):
        pend_animation = FuncAnimation(self.fig, self.pend_update, frames=len(self.time),
                                       init_func=self.pend_init, blit=True, interval=10,
                                       repeat=False)
        plt.show()

def main():
    # Initialize the pendulum
    pendulum = DoublePendulum()

    # Define the time steps and angles for simulation
    frames = 200
    time = np.linspace(0, 20, frames)  # Simulate for 10 seconds, 200 frames
    t1 = np.sin(time)  # Example angles for t1
    t2 = np.cos(time)  # Example angles for t2
    dt1 = np.gradient(t1, time)  # Time derivative of t1
    dt2 = np.gradient(t2, time)  # Time derivative of t2 (derivative of cos)

    # Initialize the animation class
    pendulum_animation = PendulumAnimation(pendulum, t1, t2, dt1, dt2, time)

    # Start the animation
    pendulum_animation.start_animation()

if __name__ == "__main__":
    main()