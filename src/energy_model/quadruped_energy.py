import numpy as np

# Quadrupedal parameters and current state
class Quadruped:
    default_leg_params={'l':1, 'I':1, 'm':1}
    def __init__(self, 
                 leg_params=default_leg_params, 
                 body_params={'l':1, 'I':1, 'm':1, 'origin':[0,0], 'orientation':0},
                 FR_params = default_leg_params,
                 FL_params = default_leg_params,
                 RL_params = default_leg_params,
                 RR_params = default_leg_params,
                 gravity = 9.81):
        
        # Define main body
        l=body_params['l']
        I=body_params['I']
        m=body_params['m']
        origin=body_params['origin']
        orientation=body_params['orientation']
        self.body = MainBody(l=l, I=I, m=m, origin=origin, orientation=orientation)
        
        # Find mount points for the legs
        left_hip, right_hip = self.body.get_endpoints()
        
        # If specific leg assignments are not set, treat them as default
        if (FR_params == self.default_leg_params and
            FL_params == self.default_leg_params and
            RL_params == self.default_leg_params and
            RR_params == self.default_leg_params):
            
            FR_params = leg_params
            FL_params = leg_params
            RL_params = leg_params
            RR_params = leg_params

        self.leg_list = [
            Leg(origin=right_hip,l=FR_params['l'], I=FR_params['I'], m=FR_params['m'], gravity=gravity),
            Leg(origin=left_hip, l=FL_params['l'], I=FL_params['I'], m=FL_params['m'], gravity=gravity), 
            Leg(origin=left_hip, l=RL_params['l'], I=RL_params['I'], m=RL_params['m'], gravity=gravity), 
            Leg(origin=right_hip,l=RR_params['l'], I=RR_params['I'], m=RR_params['m'], gravity=gravity), 
            ]
        
        # Calculate quadruped mass for CoT
        self.m = 0
        self.m += self.body.m
        for leg in self.leg_list:
            # Add up mass of both links
            self.m += np.sum(leg.m)

    def total_energy(self):
        total_energy = 0
        total_energy += self.body.total_energy()
        for leg in self.leg_list:
            total_energy += leg.total_energy()
        
        return total_energy


# Main body parameters and current state
class MainBody:
    def __init__(self, m=1, l=1, I=1, origin=[0, 0], orientation=0):
        self.m = m
        self.l = l
        self.I = I
        self.origin = np.array(origin).reshape(2, 1)
        self.orientation = orientation

        # Initilization value
        self.vel = 1
        self.height = 1

    def get_endpoints(self):
        half_length = self.l / 2
        R = np.array([[np.cos(self.orientation), -np.sin(self.orientation)],
                      [np.sin(self.orientation), np.cos(self.orientation)]])
        left_endpoint = self.origin + R @ np.array([[-half_length], [0]])
        right_endpoint = self.origin + R @ np.array([[half_length], [0]])
        return left_endpoint.flatten(), right_endpoint.flatten()
    
    def potential_energy(self):
        return self.m * 9.81 * self.height

    def translational_kinetic_energy(self):
        return .5 * self.m * self.vel**2

    def total_energy(self):
        U_g = self.potential_energy()
        T_k = self.translational_kinetic_energy()
        return U_g + T_k


# Leg parameters and current state
class Leg:
    def __init__(self, m=[1,1], l=[1,1], I=[1,1], origin=[0,0], gravity=9.81):
        # Initialize Parameters
        self.m = m
        self.l = l
        self.I = I
        self.origin = np.array(origin).reshape(2,1)

        # Initilize current state variables
        self.t1 = 0.
        self.t2 = 0.
        self.dt1 = 0.
        self.dt2 = 0.
        
        self.gravity = gravity

    # positional functions
    # returns position of com1 wrt the origin
    def get_p1(self):
        return (self.origin + (self.l[0]/2)*np.array([[np.sin(self.t1)],
                                                      [-np.cos(self.t1)]])).reshape(2,1)

    def get_pA(self):
        return (self.origin + self.l[0]*np.array([[np.sin(self.t1)],
                                                  [-np.cos(self.t1)]])).reshape(2,1)

    # returns position of com2 wrt the origin
    def get_p2(self):
        return self.get_pA() + (self.l[1]/2)*np.array([[np.sin(self.t1+self.t2)],
                                                       [-np.cos(self.t1+self.t2)]])
    
    def get_pB(self):
        return self.get_pA() + self.l[1]*np.array([[np.sin(self.t1+self.t2)],
                                                   [-np.cos(self.t1+self.t2)]])

    # velocity functions
    def get_v1(self):
        v = (self.l[0]/2)*np.array([[np.cos(self.t1)],
                                    [np.sin(self.t1)]]) * self.dt1
        return v

    def get_vA(self):
        v = self.l[0]*np.array([[np.cos(self.t1)],
                                [np.sin(self.t1)]]) * self.dt1
        return v
    
    def get_v2(self):
        v = self.get_vA() + (self.l[1]/2)*np.array([[np.cos(self.t1+self.t2)],
                                                    [np.sin(self.t1+self.t2)]])*(self.dt1+self.dt2)
        return v
    
    def get_vB(self):
        v = self.get_vA() + (self.l[1])*np.array([[np.cos(self.t1+self.t2)],
                                                  [np.sin(self.t1+self.t2)]])*(self.dt1+self.dt2)
        return v
        
    # energy functions
    def potential_energy(self):
        g = self.gravity
        p1 = self.get_p1()
        p2 = self.get_p2()

        h1 = p1[1]
        h2 = p2[1]

        u1 = self.m[0]*g*h1
        u2 = self.m[1]*g*h2
        return abs(u1+u2)

    def translational_kinetic_energy(self):
        v1 = self.get_v1()
        v2 = self.get_v2()

        kt1 = .5*self.m[0]*(v1.T@v1)
        kt2 = .5*self.m[1]*(v2.T@v2)
        return kt1+kt2

    def rotational_kinetic_energy(self):
        kw1 = .5*self.I[0]*self.dt1**2
        kw2 = .5*self.I[1]*(self.dt1+self.dt2)**2
        return kw1+kw2
    
    def thigh_energy(self):
        def potential_energy(self):
            g = self.gravity
            p1 = self.get_p1()

            h1 = p1[1]

            u1 = self.m[0]*g*h1
            return abs(u1)
        
        def tranlsational_kinetic_energy(self):
            v1 = self.get_v1()

            kt1 = .5*self.m[0]*(v1.T@v1)
            return kt1
        
        def rotational_kinetic_energy(self):
            kw1 = .5*self.I[0]*self.dt1**2
            return kw1
        
        u = potential_energy(self)
        kt = tranlsational_kinetic_energy(self)
        kw = rotational_kinetic_energy(self)

        return (u + kt + kw).item()

    def calf_energy(self):
        def potential_energy(self):
            g = self.gravity
            p2 = self.get_p2()

            h2 = p2[1]

            u2 = self.m[1]*g*h2
            return abs(u2)
        
        def tranlsational_kinetic_energy(self):
            v2 = self.get_v2()

            kt2 = .5*self.m[1]*(v2.T@v2)
            return kt2
        
        def rotational_kinetic_energy(self):
            kw2 = .5*self.I[1]*(self.dt1+self.dt2)**2
            return kw2
        
        u = potential_energy(self)
        kt = tranlsational_kinetic_energy(self)
        kw = rotational_kinetic_energy(self)

        return (u + kt + kw).item()
    
    def total_energy(self):
        return (self.potential_energy() + 
                self.translational_kinetic_energy() + 
                self.rotational_kinetic_energy()
                ).item()


# Helper class to carry around quadruped state trajectory data
# Used for calculating behavior over time
class QuadrupedData:
    def __init__(self, quadruped: Quadruped, timelist, gait_data=[[[],[]],[[],[]],[[],[]],[[],[]]]):
        self.quadruped = quadruped

        # append gait_data to be of appropriate size if no initializer is given
        if gait_data == [[[],[]],[[],[]],[[],[]],[[],[]]]:
            for leg in gait_data:
                for link in leg:
                    for _ in timelist:
                        link.append(0.)

        self.timelist = timelist
        
        # Written for clarity
        leg1, leg2, leg3, leg4 = gait_data
        leg1_t1, leg1_t2 = leg1
        leg2_t1, leg2_t2 = leg2
        leg3_t1, leg3_t2 = leg3
        leg4_t1, leg4_t2 = leg4

        self.leg_list = [LegData(timelist, quadruped.leg_list[0], leg1_t1, leg1_t2),
                         LegData(timelist, quadruped.leg_list[1], leg2_t1, leg2_t2),
                         LegData(timelist, quadruped.leg_list[2], leg3_t1, leg3_t2),
                         LegData(timelist, quadruped.leg_list[3], leg4_t1, leg4_t2)]
        
    def refresh(self):
        for leg in self.leg_list:
            leg.refresh()
            
    def manual_set_velocity(self, gait_data=[[[],[]],[[],[]],[[],[]],[[],[]]]):
        leg1, leg2, leg3, leg4 = gait_data
        leg1_t1, leg1_t2 = leg1
        leg2_t1, leg2_t2 = leg2
        leg3_t1, leg3_t2 = leg3
        leg4_t1, leg4_t2 = leg4
        
        self.leg_list[0].manual_set_velocity(leg1_t1, leg1_t2)
        self.leg_list[1].manual_set_velocity(leg2_t1, leg2_t2)
        self.leg_list[2].manual_set_velocity(leg3_t1, leg3_t2)
        self.leg_list[3].manual_set_velocity(leg4_t1, leg4_t2)
    
    # For completenesss, calculates the body height of the quadruped and updates quadruped
    def calculate_body_height(self):
        # Steps:
        # 1) Identify stance vs swing phase for each foot
        stance_list, _, foot_height_list = self.find_stance_phase()

        # 3) the average foot velocity between all 4 of the feet during the stance phase is the approximated body velocity
        foot_average_height = [0,0,0,0]
        for leg_index, leg in enumerate(self.quadruped.leg_list):
            stance_frames = 0
            for index, stance in enumerate(stance_list[leg_index]):
                if stance == True:
                    foot_average_height[leg_index] += foot_height_list[leg_index][index]
                    stance_frames += 1
            
            foot_average_height[leg_index] = abs(foot_average_height[leg_index]/stance_frames)
        
        average_body_height = np.mean(foot_average_height)

        # Set the current body velocity in the 
        self.quadruped.body.height = average_body_height
        return average_body_height
    
    # Helper function to approximate quadruped body velocity based on leg trajectory
    def calculate_vel(self):
        # Steps:
        # 1) Identify stance vs swing phase for each foot
        stance_list, foot_vel_list, _ = self.find_stance_phase()

        # 3) the average foot velocity between all 4 of the feet during the stance phase is the approximated body velocity
        foot_average_vel = [0,0,0,0]
        for leg_index, leg in enumerate(self.quadruped.leg_list):
            stance_frames = 0
            for index, stance in enumerate(stance_list[leg_index]):
                if stance == True:
                    foot_average_vel[leg_index] += foot_vel_list[leg_index][index]
                    stance_frames += 1
            
            foot_average_vel[leg_index] = abs(foot_average_vel[leg_index]/stance_frames)
        
        average_body_vel = np.mean(foot_average_vel)

        # Set the current body velocity in the 
        self.quadruped.body.vel = average_body_vel
        return average_body_vel
            
    # returns list of bools defining if the leg is in stance or swing
    # returns list of velocities for the foot at each time instance
    def find_stance_phase(self):
        # Assume quadruped has 4 legs (duh)
        height_list = [[],[],[],[]]
        vB_list = [[],[],[],[]]
        stance_list = [[],[],[],[]]
        
        # itterate through each leg
        for leg_index, leg in enumerate(self.leg_list):
            
            # Itterate through each time index
            for time_index, time in enumerate(leg.timelist):
                self.quadruped.leg_list[leg_index].t1 = leg.t1[time_index]
                self.quadruped.leg_list[leg_index].t2 = leg.t2[time_index]
                self.quadruped.leg_list[leg_index].dt1 = leg.dt1[time_index]
                self.quadruped.leg_list[leg_index].dt2 = leg.dt2[time_index]
                
                # Record foot velocity in body frame
                foot_vel_x = self.quadruped.leg_list[leg_index].get_vB()[0]
                vB_list[leg_index].append(foot_vel_x)

                # Record foot distance from body
                foot_dist_y = self.quadruped.leg_list[leg_index].get_pB()[1]
                height_list[leg_index].append(foot_dist_y)

                # Record if foot is in stance or swing
                if foot_vel_x < 0:
                    # If the leg is moving in the negative x direction in the body frame
                    stance_list[leg_index].append(True)
                else:
                    stance_list[leg_index].append(False)

        return stance_list, vB_list, height_list
    
    # Returns list of total quadruped energy over time
    def energy_trajectory(self) -> list:
        
        energy_list = [[[],[]],[[],[]],[[],[]],[[],[]]]

        # For each leg
        for leg_index, leg in enumerate(self.quadruped.leg_list):
            
            # For each time instance
            for time_index, time in enumerate(self.timelist):
                
                # Update current state
                leg.t1 = self.leg_list[leg_index].t1[time_index]
                leg.t2 = self.leg_list[leg_index].t2[time_index]
                leg.dt1 = self.leg_list[leg_index].dt1[time_index]
                leg.dt2 = self.leg_list[leg_index].dt2[time_index]
                
                # Calculate current energy
                energy_list[leg_index][0].append(leg.thigh_energy())
                energy_list[leg_index][1].append(leg.calf_energy())
        
        return energy_list
    
    def power_trajectory(self):

        quadruped_power_traj_list = [[[],[]],[[],[]],[[],[]],[[],[]]]
        

        energy_trajs = self.energy_trajectory()
        for leg_index, leg in enumerate(energy_trajs):
            for link_index, link_energy_traj in enumerate(leg):
                quadruped_power_traj_list[leg_index][link_index] = np.gradient(link_energy_traj, self.timelist)
        
        abs_val_power_traj = np.zeros(len(quadruped_power_traj_list[0][0]))

        for leg_index, _ in enumerate(quadruped_power_traj_list):
            for link_index, _ in enumerate(leg):
                abs_val_power_traj += np.abs(quadruped_power_traj_list[leg_index][link_index])
        
        return abs_val_power_traj
    
    def calc_work_done(self):
        work_done = 0
        
        # Integrate the power consumption over the trajectory duration
        power_traj = self.power_trajectory()

        # Time between each frame
        time = self.timelist[1] - self.timelist[0]

        # Integrateto compute total work done
        for i in range(len(power_traj) - 1):
            work_done += (power_traj[i] + power_traj[i+1]) * time / 2
        
        return work_done
    
    def calc_distance(self):
        v = self.calculate_vel()
        time = self.timelist[-1]
        return v * time
    
    def cost_of_transport(self):
        work = self.calc_work_done() 
        g = 9.81 # m/s^2
        dist = self.calc_distance()
        return work / (self.quadruped.m * g * dist)


# Helper class to carry around leg state trajectory data
class LegData:
    def __init__(self, timelist, leg: Leg, t1_list, t2_list):
        self.timelist = timelist
        self.t1 = t1_list
        self.t2 = t2_list
        self.dt1 = np.gradient(self.t1, self.timelist)
        self.dt2 = np.gradient(self.t2, self.timelist)
        self.leg = leg

    def refresh(self):
        self.dt1 = np.gradient(self.t1, self.timelist)
        self.dt2 = np.gradient(self.t2, self.timelist)
        
    def manual_set_velocity(self, dt1, dt2):
        self.dt1 = dt1
        self.dt2 = dt2

    def energy_trajectory(self):
        thigh_energy_trajectory = []
        calf_energy_trajectory = []

        # For each time instance
        for time_index, time in enumerate(self.timelist):

            # Calculate current energy
            thigh_energy_trajectory.append(self.leg.thigh_energy())
            calf_energy_trajectory.append(self.leg.calf_energy())
        
        return thigh_energy_trajectory, calf_energy_trajectory

    def power_trajecltory(self):
        thigh_energy_traj, calf_energy_traj = self.energy_trajectory()
        
        thigh_power_traj = np.gradient(thigh_energy_traj, self.timelist)
        calf_power_traj = np.gradient(calf_energy_traj, self.timelist)

        return thigh_power_traj, calf_power_traj

def main():
    pass

if __name__ == "__main__":
    main()