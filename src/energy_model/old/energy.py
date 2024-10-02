from geometry_msgs.msg import Pose
import numpy as np
import matplotlib as plt

import numpy as np

class Link():
    def __init__(self, m, l, I):
        # Initialize link
        self.m = m
        self.l = l
        self.I = I

        # Assume CoM is at centroid of link
        self.com = self.l/2


class Leg():
    def __init__(self, m=[1,1], l=[1,1], I=[1,1], t=[0,0]):
        
        # Define list of links to describe leg
        self.links = []
        for i in range(len(l)):
            self.links.append(Link(m[i], l[i], I[i]))

        self.theta = t

class Quadruped():
    def __init__(self):
        self.body = Link()
        self.legs = [Leg(), Leg()]
        
        # assume height of body wrt ground and body is parallel with ground
        self.h_body = 1

        # set body com wrt centroid
        self.body_com = 0

    # Compute a single transformation based on DH parameters
    @staticmethod
    def dh_transformation(a, alpha, d, theta):
        # Compute the DH transformation matrix
        T = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                      [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                      [0, np.sin(alpha), np.cos(alpha), d],
                      [0, 0, 0, 1]])
        return T

    def potential_energy(self):
        # Body frames
        T00 = np.linalg.eye(4)
        T0c = self.dh_transformation(self.body_com, 0, 0, 0)

        # Leg 1 frames
        Tb1 = self.dh_transformation(self.body.l/2, -np.pi/2, 0, np.pi + self.legs[0].theta[0])
        T11c = self.dh_transformation(self.legs[0].links[0].com, 0, 0, np.pi/2)
        T12 = self.dh_transformation(self.legs[0].links[0].l, 0, 0, self.legs[0].theta[1])
        T22c = self.dh_transformation(self.legs[0].links[1].com, 0, 0, 0)
        T23 = self.dh_transformation(self.legs[0].links[1].l, 0, 0, 0)
        
        # Leg 2 frames
        Tb1 = self.dh_transformation(self.body.l/2, -np.pi/2, 0, self.legs[0].theta[0])
        T11c = self.dh_transformation(self.legs[1].links[0].com, 0, 0, np.pi/2)
        T12 = self.dh_transformation(self.legs[1].links[0].l, 0, 0, self.legs[0].theta[1])
        T22c = self.dh_transformation(self.legs[1].links[1].com, 0, 0, 0)
        T23 = self.dh_transformation(self.legs[1].links[1].l, 0, 0, 0)

        # Start with zero energy
        U_g = 0
        
        # Add in energy due to torso
        U_g += self.body_potential_energy(self.m_body, self.h_body)
        
        # Itterate through each leg
        return U_g

    def trans_kin_energy(self):
        pass

    def rot_kin_energy(self):
        pass

    def energy_calc(self):
        return self.potential_energy() + self.trans_kin_energy() + self.rotat_kin_energy()


def main():
    pass

if __name__ == "main":
    main()