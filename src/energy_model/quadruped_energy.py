import numpy as np

class Quadruped:
    def __init__(self, leg_params={'l':1, 'I':1, 'm':1}, 
                 body_params={'l':1, 'I':1, 'm':1, 'origin':[0,0], 'orientation':0}):
        
        # Define main body
        l=body_params['l']
        I=body_params['I']
        m=body_params['m']
        origin=body_params['origin']
        orientation=body_params['orientation']
        self.body = MainBody(l=l, I=I, m=m, origin=origin, orientation=orientation)
        
        # Find mount points for the legs
        left_hip, right_hip = self.body.get_endpoints()
        
        # Define legs
        l = leg_params['l']
        I = leg_params['I']
        m = leg_params['m']
        # Assume all legs have the same physical properties
        self.leg_list = [Leg(origin=left_hip, l=l, I=I, m=m), 
                         Leg(origin=left_hip, l=l, I=I, m=m), 
                         Leg(origin=right_hip,l=l, I=I, m=m), 
                         Leg(origin=right_hip,l=l, I=I, m=m)]

    def total_energy(self):
        total_energy = 0
        total_energy += self.body.total_energy()
        for leg in self.leg_list:
            total_energy += leg.total_energy()
        
        return total_energy

class MainBody:
    def __init__(self, m=1, l=1, I=2, origin=[0, 0], orientation=0):
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
    
    def potential_energy(self):
        # Approximate the height of the body based on the lowest position of the legs
        h = .2 ############################################# TEMP VALUE ##########################################################################
        return self.m * 9.81 * h

    def translational_kinetic_energy(self):
        v = 3 ############################################# TEMP VALUE #############################################################
        return .5 * self.m * v**2

    def total_energy(self):
        U_g = self.potential_energy()
        T_k = self.translational_kinetic_energy()
        return U_g + T_k


class Leg:
    def __init__(self, m=[1,1], l=[1,1], I=[1,1], origin=[0,0]):
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
        
    # energy functions
    def potential_energy(self):
        g = 9.81
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
    
    def total_energy(self):
        return (self.potential_energy() + 
                self.translational_kinetic_energy() + 
                self.rotational_kinetic_energy()
                ).item()

def main():
    pass

if __name__ == "__main__":
    main()