import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class DoublePendulum:
    def __init__(self):
        # Define parameters
        self.m1 = 1
        self.m2 = 1
        self.l1 = 1
        self.l2 = 1
        self.I1 = 1
        self.I2 = 1

    # positional functions
    # returns position of com1 wrt the origin
    def get_p1(self,t1):
        return (self.l1/2)*np.array([[np.sin(t1)],
                                [-np.cos(t1)]])

    def get_pA(self,t1):
        return self.l1*np.array([[np.sin(t1)],
                            [-np.cos(t1)]])

    # returns position of com2 wrt the origin
    def get_p2(self, t1, t2):
        return self.get_pA(t1) + (self.l2/2)*np.array([[np.sin(t1+t2)],
                                                       [-np.cos(t1+t2)]])

    # velocity functions
    def get_v1(self, t1,dt1):
        v = (self.l1/2)*np.array([[np.cos(t1)],
                            [np.sin(t1)]]) * dt1
        return v

    def get_vA(self,t1,dt1):
        v = self.l1*np.array([[np.cos(t1)],
                        [np.sin(t1)]]) * dt1
        return v
    
    def get_v2(self,t1,dt1,t2,dt2):
        v = self.get_vA(t1,dt1) + (self.l2/2)*np.array([[np.cos(t1+t2)],
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

        u1 = -self.m1*g*h1
        u2 = -self.m2*g*h2
        return u1+u2

    def translational_kinetic_energy(self,t1, dt1, t2, dt2):
        v1 = self.get_v1(t1, dt1)
        v2 = self.get_v2(t1, dt1, t2, dt2)

        kt1 = .5*self.m1*(v1.T@v1)
        kt2 = .5*self.m2*(v2.T@v2)
        return kt1+kt2

    def rotational_kinetic_energy(self,dt1,dt2):
        # Parallel axis theorem since I2 is defined from connection point at A
        I2_offset = self.I2 + self.m2*self.l1^2

        kw1 = .5*self.I1*dt1^2
        kw2 = .5*I2_offset*(dt1+dt2)^2
        return kw1+kw2
    
class PendulumAnimation:
    def __init__(self, pendulum, t1, t2, time):
        self.pendulum = pendulum
        self.t1 = t1
        self.t2 = t2
        self.time = time

        # Create the figure and axis for plotting
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_aspect('equal')

        # Create the lines for the pendulum
        self.line1, = self.ax.plot([], [], lw=2)
        self.line2, = self.ax.plot([], [], lw=2)

    # Initialization function for animation
    def init(self):
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        return self.line1, self.line2

    # Animation update function
    def update(self, frame):
        t1_frame = self.t1[frame]
        t2_frame = self.t2[frame]
        
        # Get positions of the pendulum's masses
        p1 = self.pendulum.get_p1(t1_frame)
        pA = self.pendulum.get_pA(t1_frame)
        p2 = self.pendulum.get_p2(t1_frame, t2_frame)

        # Update the lines for both arms of the pendulum
        self.line1.set_data([0, pA[0]], [0, pA[1]])  # Arm 1
        self.line2.set_data([pA[0], p2[0]], [pA[1], p2[1]])  # Arm 2
        return self.line1, self.line2

    # Method to start the animation
    def start_animation(self):
        ani = FuncAnimation(self.fig, self.update, frames=len(self.time),
                            init_func=self.init, blit=True)
        plt.show()

def main():
    # Initialize the pendulum
    pendulum = DoublePendulum()

    # Define the time steps and angles for simulation
    time = np.linspace(0, 10, 500)  # Simulate for 10 seconds, 500 time steps
    t1 = np.sin(time)  # Example angles for t1
    t2 = np.cos(time)  # Example angles for t2

    # Initialize the animation class with the pendulum and angles
    pendulum_animation = PendulumAnimation(pendulum, t1, t2, time)

    # Start the animation
    pendulum_animation.start_animation()

if __name__ == "__main__":
    main()