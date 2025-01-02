import numpy as np
from FittedCoefficient import *
from scipy.optimize import fsolve
import time

#### To Do ####


#### LegModel ####
# Using approximate coefficients instead of kinematics.
# Init param - sim: True means using wheel radius without tire. False means using actual wheel radius (including tire thick).
class LegModel:
    def __init__(self, sim=True):
        #### Constant values ####
        # max/min theta
        self.max_theta = np.deg2rad(160.0)  # maximum theta = 160 deg
        self.min_theta = np.deg2rad(17.0)   # minimum theta = 17 deg
        self.theta0 = np.deg2rad(17.0)      # theta0 = 17 deg
        self.beta0 = np.deg2rad(90.0)       # beta0  = 90 deg
        # wheel radius 
        self.R = 0.1    # 10 cm
        if sim:
            self.r = 0.0125 # no tire: 1.25 cm
        else:
            self.r = 0.019  # with tire: 1.9 cm
        self.radius = self.R + self.r
        # linkage parameters
        self.arc_HF = np.deg2rad(130)   # arc HF
        self.arc_BC = np.deg2rad(101)   # arc BC
        self.l1 = 0.8 * self.R                                                          # l1
        self.l2 = self.R - self.l1                                                      # l2
        self.l3 = 2.0 * self.R * np.sin(self.arc_BC / 2)                                # l3
        self.l4 = 0.882966335 * self.R                                                  # l4
        self.l5 = 0.9 * self.R                                                          # l5
        self.l6 = 0.4 * self.R                                                          # l6
        self.l7 = 2.0 * self.R * np.sin((self.arc_HF - self.arc_BC - self.theta0) / 2)  # l7
        self.l8 = 2.0 * self.R * np.sin((np.pi - self.arc_HF) / 2)                      # l8
        # some useful paramters in the calculation
        self.l_AE = self.l5 + self.l6                                       # length of AE
        self.l_BF = 2.0 * self.R * np.sin((self.arc_HF - self.theta0) / 2)  # length of BF
        self.l_BH = 2.0 * self.R * np.sin( self.theta0 / 2)                 # length of BH
        self.ang_UBC = (np.pi - self.arc_BC) / 2                            # angle upperBC
        self.ang_LFG = (np.pi - (np.pi - self.arc_HF)) / 2                  # angle lowerFG
        
        #### Variable values ####
        # intermediate values during the calculation
        self.l_BD = 0       # length of BD
        self.ang_OEA = 0    # angle OEA
        self.ang_BCF = 0    # angle BCF
        self.ang_DBC = 0    # angle DBC
        self.ang_OGF = 0    # angle OGF
        # get initial positions of all joints ([x, y])
        self.forward(np.deg2rad(17), np.deg2rad(0))
        
        #### Contact map variable ####
        self.rim = 3    # 1 -> 2 -> 3 -> 4 -> 5 -> 0: 
                        # U_l -> L_l -> G -> L_r -> U_r -> None
                        
        
    #### Forward kinematics ####
    # Input theta, beta: radian unit. Both can be either a single value or an 1-dimensional array with any size, but they have to be the same size.
    #           Values outside max/min limit will be set to max/min limit. 
    # Dimensions of joint position: size n array input: n*2 
    #                               single value input: 2
    def forward(self, theta, beta, vector=True):
        self.theta = np.array(theta)
        self.beta = np.array(beta)
        self.n_elements = 0 if self.theta.ndim == 0 else self.theta.shape[0]  # amount of theta given in an array, 0: single value.
        # Check theta range
        limit_u = self.theta > self.max_theta   # theta exceeding upper bound set to upper bound
        self.theta[limit_u] = self.max_theta
        limit_l = self.theta < self.min_theta   # theta below lower bound set to lower bound
        self.theta[limit_l] = self.min_theta
        if np.sum(limit_u) != 0:
            print("Limit upper bound:", np.sum(limit_u))
        if np.sum(limit_l) != 0:
            print("Limit lower bound:", np.sum(limit_l))
        # Forward kinematics
        self.calculate()
        self.rotate()
        if vector:
            self.to_vector()
    
    # Calculate by theta using coefficient
    def calculate(self, coefficient=False):
        if not coefficient:
            # Forward kinematics
            self.A_l = self.l1 * np.exp( 1j*(self.theta) )
            self.B_l = self.R * np.exp( 1j*(self.theta) )
            self.ang_OEA = np.arcsin( self.l1 / self.l_AE *np.sin(self.theta) )
            self.E = self.l1 * np.cos(self.theta) - self.l_AE * np.cos(self.ang_OEA)
            self.D_l = self.E + self.l6 * np.exp( 1j*(self.ang_OEA) )
            self.l_BD = abs(self.D_l - self.B_l)
            self.ang_DBC = np.arccos((self.l_BD**2 + self.l3**2 - self.l4**2) / (2 * self.l_BD * self.l3))
            self.C_l = self.B_l + (self.D_l - self.B_l) * np.exp( -1j*(self.ang_DBC) ) * (self.l3 / self.l_BD) # OC = OB + BC
            self.ang_BCF = np.arccos((self.l3**2 + self.l7**2 - self.l_BF**2) / (2 * self.l3 * self.l7)) 
            self.F_l = self.C_l + (self.B_l - self.C_l) * np.exp( -1j*(self.ang_BCF) ) * (self.l7 / self.l3) # OF = OC + CF
            self.ang_OGF = np.arcsin(abs(self.F_l.imag) / self.l8)
            self.G = self.F_l - self.l8 * np.exp( 1j*(self.ang_OGF) ) # OG = OF - GF
            self.U_l = self.B_l + (self.C_l - self.B_l) * np.exp( 1j*(self.ang_UBC) ) * (self.R / self.l3)   # OOU = OB + BOU
            self.L_l = self.F_l + (self.G - self.F_l) * np.exp( 1j*(self.ang_LFG) ) * (self.R / self.l8)   # OOL = OF + FOL
            self.H_l = self.U_l + (self.B_l - self.U_l) * np.exp( -1j*(self.theta0) )  # OH = OOU + OUH
        else:
            # Using coefficient
            self.A_l = A_l_poly[1](self.theta) - 1j * A_l_poly[0](self.theta)
            self.B_l = B_l_poly[1](self.theta) - 1j * B_l_poly[0](self.theta)
            self.C_l = C_l_poly[1](self.theta) - 1j * C_l_poly[0](self.theta)
            self.D_l = D_l_poly[1](self.theta) - 1j * D_l_poly[0](self.theta)
            self.E   = E_poly[1](self.theta)   - 1j * E_poly[0](self.theta)
            self.F_l = F_l_poly[1](self.theta) - 1j * F_l_poly[0](self.theta)
            self.G   = G_poly[1](self.theta)   - 1j * G_poly[0](self.theta)
            self.H_l = H_l_poly[1](self.theta) - 1j * H_l_poly[0](self.theta)
            self.U_l = U_l_poly[1](self.theta) - 1j * U_l_poly[0](self.theta)
            self.L_l = L_l_poly[1](self.theta) - 1j * L_l_poly[0](self.theta)
        self.symmetry()
        
    # Rotate by beta
    def rotate(self):
        rot_ang  = np.exp( 1j*(np.array(self.beta) + self.beta0) )
        # Rotate
        self.A_l = rot_ang * self.A_l
        self.A_r = rot_ang * self.A_r
        self.B_l = rot_ang * self.B_l
        self.B_r = rot_ang * self.B_r
        self.C_l = rot_ang * self.C_l
        self.C_r = rot_ang * self.C_r
        self.D_l = rot_ang * self.D_l
        self.D_r = rot_ang * self.D_r
        self.E   = rot_ang * self.E
        self.F_l = rot_ang * self.F_l
        self.F_r = rot_ang * self.F_r
        self.G   = rot_ang * self.G
        self.H_l = rot_ang * self.H_l
        self.H_r = rot_ang * self.H_r
        self.U_l = rot_ang * self.U_l
        self.U_r = rot_ang * self.U_r
        self.L_l = rot_ang * self.L_l
        self.L_r = rot_ang * self.L_r

    # Get right side joints before rotate beta
    def symmetry(self):
        self.A_r = np.conjugate(self.A_l)
        self.B_r = np.conjugate(self.B_l)
        self.C_r = np.conjugate(self.C_l)
        self.D_r = np.conjugate(self.D_l)
        self.F_r = np.conjugate(self.F_l)
        self.H_r = np.conjugate(self.H_l)
        self.U_r = np.conjugate(self.U_l)
        self.L_r = np.conjugate(self.L_l)
    
    # Convert position expressions from complex numbers to vectors
    def to_vector(self):
        if self.n_elements == 0:
            self.A_l = np.array([self.A_l.real, self.A_l.imag])
            self.A_r = np.array([self.A_r.real, self.A_r.imag])
            self.B_l = np.array([self.B_l.real, self.B_l.imag])
            self.B_r = np.array([self.B_r.real, self.B_r.imag])
            self.C_l = np.array([self.C_l.real, self.C_l.imag])
            self.C_r = np.array([self.C_r.real, self.C_r.imag])
            self.D_l = np.array([self.D_l.real, self.D_l.imag])
            self.D_r = np.array([self.D_r.real, self.D_r.imag])
            self.E   = np.array([self.E.real, self.E.imag])
            self.F_l = np.array([self.F_l.real, self.F_l.imag])
            self.F_r = np.array([self.F_r.real, self.F_r.imag])
            self.G   = np.array([self.G.real, self.G.imag])
            self.H_l = np.array([self.H_l.real, self.H_l.imag])
            self.H_r = np.array([self.H_r.real, self.H_r.imag])
            self.U_l = np.array([self.U_l.real, self.U_l.imag])
            self.U_r = np.array([self.U_r.real, self.U_r.imag])
            self.L_l = np.array([self.L_l.real, self.L_l.imag])
            self.L_r = np.array([self.L_r.real, self.L_r.imag])
        else:
            self.A_l = np.array([self.A_l.real, self.A_l.imag]).transpose(1, 0)
            self.A_r = np.array([self.A_r.real, self.A_r.imag]).transpose(1, 0)
            self.B_l = np.array([self.B_l.real, self.B_l.imag]).transpose(1, 0)
            self.B_r = np.array([self.B_r.real, self.B_r.imag]).transpose(1, 0)
            self.C_l = np.array([self.C_l.real, self.C_l.imag]).transpose(1, 0)
            self.C_r = np.array([self.C_r.real, self.C_r.imag]).transpose(1, 0)
            self.D_l = np.array([self.D_l.real, self.D_l.imag]).transpose(1, 0)
            self.D_r = np.array([self.D_r.real, self.D_r.imag]).transpose(1, 0)
            self.E   = np.array([self.E.real, self.E.imag]).transpose(1, 0)
            self.F_l = np.array([self.F_l.real, self.F_l.imag]).transpose(1, 0)
            self.F_r = np.array([self.F_r.real, self.F_r.imag]).transpose(1, 0)
            self.G   = np.array([self.G.real, self.G.imag]).transpose(1, 0)
            self.H_l = np.array([self.H_l.real, self.H_l.imag]).transpose(1, 0)
            self.H_r = np.array([self.H_r.real, self.H_r.imag]).transpose(1, 0)
            self.U_l = np.array([self.U_l.real, self.U_l.imag]).transpose(1, 0)
            self.U_r = np.array([self.U_r.real, self.U_r.imag]).transpose(1, 0)
            self.L_l = np.array([self.L_l.real, self.L_l.imag]).transpose(1, 0)
            self.L_r = np.array([self.L_r.real, self.L_r.imag]).transpose(1, 0)
            
            
    #### Contact map ####
    # Not consider the situation where there is no normal contact.
    # Input theta, beta: radian unit. Both can be either a single value or an 1-dimensional array with any size, but they have to be the same size.
    #           Values outside max/min limit will be set to max/min limit. 
    # Input slope: slope of ground (rad), one single value for all input theta and beta.
    # Dimensions of joint position: size n array input: n (complex number, real for x, imag for y)
    #                               single value input: 1 (complex number, real for x, imag for y)
    # Rim: the rim contacting ground of the corresponding theta and beta.
    # Alpha: alpha=0 when (theta, beta)=(17, 0). alpha=-2pi~2pi when lower or upper rim contact. alpha=alpha_G when G contact.
    def contact_map(self, theta, beta, slope=0):
        beta = np.array(beta) - slope
        self.forward(theta, beta, vector=False)

        # points on the outside of the wheel
        G_l = (self.G - self.L_l)   / self.R * self.radius + self.L_l
        G_r = (self.G - self.L_r) / self.R * self.radius + self.L_r
        H_l = (self.H_l - self.U_l) / self.R * self.radius + self.U_l
        H_r = (self.H_r - self.U_r) / self.R * self.radius + self.U_r
        F_l = (self.F_l - self.U_l) / self.R * self.radius + self.U_l
        F_r = (self.F_r - self.U_r) / self.R * self.radius + self.U_r  
        
        # minimum point and its alpha of each arc 
        self.n_elements = 0 if self.theta.ndim == 0 else self.theta.shape[0]  # amount of theta given in an array, 0: single value.
        zeros = (0, 0) if self.n_elements == 0 else (np.zeros(self.n_elements), np.zeros(self.n_elements))
        arc_list = [
            self.arc_min(H_l, F_l, self.U_l, 'left upper'),
            self.arc_min(F_l, G_l, self.L_l, 'left lower'),
            self.arc_min(G_l, G_r, self.G, 'G'),
            self.arc_min(G_r, F_r, self.L_r, 'right lower'),
            self.arc_min(F_r, H_r, self.U_r, 'right upper'),
            zeros
        ]
        arc_list = np.array(arc_list)

        self.rim = np.where(np.min(arc_list[:, 0], axis=0) == 0, 0, np.argmin(arc_list[:, 0], axis=0) + 1)
        self.alpha = arc_list[self.rim-1, 1] if self.n_elements == 0 else arc_list[self.rim-1, 1, np.arange(self.n_elements)]

    # Get lowest point and the corresponding alpha value of the rim
    def arc_min(self, p1, p2, O, rim): # alpha: arc starting from most clockwise (most left) of the rim
        lowest_points = np.array(.0) if self.n_elements == 0 else np.zeros(self.n_elements)
        alpha         = np.array(.0) if self.n_elements == 0 else np.zeros(self.n_elements)
        # mapping alpha from 0~130/50 to -2*pi~2*pi
        if rim == 'left upper':
            bias_alpha = -np.deg2rad(180)
            bias_alpha = 0
        elif rim == 'left lower':
            bias_alpha = -np.deg2rad(50)
            bias_alpha = 0
        elif rim == 'G':
            direction_G = p1+p2
            bias_alpha = np.angle( (p1 - O)/direction_G )
        elif rim == 'right lower':
            bias_alpha = np.deg2rad(0)
            bias_alpha = 0
        elif rim == 'right upper':
            bias_alpha = np.deg2rad(50)
            bias_alpha = 0
        
        cal_err = 0.000000001
        in_range = ((p2 - O).real >= -cal_err) & ((p1 - O).real <= cal_err)    # lowest point is between 2 endpoints
        lowest_points[in_range] = O[in_range].imag - self.radius
        alpha[in_range] = np.angle( -1j/(p1[in_range] - O[in_range]) )
        
        p1_out = p1[~in_range]  # lowest point is one of 2 endpoints
        p2_out = p2[~in_range]  # lowest point is one of 2 endpoints
        O_out = O[~in_range]    # lowest point is one of 2 endpoints
        smaller = np.where(p1_out.imag < p2_out.imag, p1_out, p2_out)   # smaller y value of 2 endpoints
        lowest_points[~in_range] = 1.0    # if not normal contact, set very large
        alpha[~in_range] = np.angle( (smaller - O_out) / (p1_out - O_out) )
        return lowest_points, alpha + bias_alpha  
    
    
    #### Inverse kinematics ####
    # Input pos: joint position relative to hip, an array with size 2 ([x, y]).
    # Input joint: 'G', 'Ul', 'Ur', 'Ll' or 'Lr', indicating pos is position of this joint.
    # Input forward: True means using forward kinematics to get all other joint positions after get theta and beta,
    #                   and storing theta and beta. Otherwise, just return theta and beta.
    def inverse(self, pos, joint='G', forward=True):
        abs_pos = np.linalg.norm(pos)
        if joint == 'G':
            theta = inv_G_dist_poly(abs_pos)
            beta = np.angle( (pos[0] + 1j*pos[1]) / -1j*abs_pos)
        elif joint in ['Ul', 'Ur']:
            theta = inv_U_dist_poly(abs_pos)
            if joint == 'Ul':
                U_x_beta0 = U_l_poly[0](theta)
                U_y_beta0 = U_l_poly[1](theta)
            else:   # Ur
                U_x_beta0 = U_r_poly[0](theta)
                U_y_beta0 = U_r_poly[1](theta)               
            beta = np.angle( (pos[0] + 1j*pos[1]) / (U_x_beta0 + 1j*U_y_beta0))
        elif joint in ['Ll', 'Lr']:
            theta = inv_L_dist_poly(abs_pos)
            if joint == 'Ll':
                L_x_beta0 = L_l_poly[0](theta)
                L_y_beta0 = L_l_poly[1](theta)
            else:   # Lr
                L_x_beta0 = L_r_poly[0](theta)
                L_y_beta0 = L_r_poly[1](theta)               
            beta = np.angle( (pos[0] + 1j*pos[1]) / (L_x_beta0 + 1j*L_y_beta0))
        else:
            raise RuntimeError("joint need to be 'G', 'Ul', 'Ur', 'Ll' or 'Lr.")
        if forward:
            self.forward(theta, beta)
        return theta, beta


    #### Move ####
    # Assume ground slope is 0 and the leg is currently contacting the ground.
    # Input move_vec: vector of hip movement [delta x, delta y]
    def move(self, theta, beta, move_vec, contact_upper=True):
        self.contact_map(theta, beta) # also get all joint positions in polar coordinate (x+jy).
        if contact_upper:   # if upper rim can contact ground, set lowest point as contact point.
            contact_rim = self.rim
        else:   # if upper rim can not contact ground, set lowest point among lower rim and G as contact point.
            if self.rim in [2, 3, 4]:
                contact_rim = self.rim
            elif self.beta > 0:
                contact_rim = 2
            else:   # beta < 0
                contact_rim = 4
        
        result, infodict, ier, mesg = fsolve(lambda d_q: self.objective(d_q, (self.theta, self.beta), move_vec, contact_rim), np.array([0, 0]), full_output=True)     
        assert ier, "Not converge. Not found a feasible solution for the disired hip position."
        # if np.linalg.norm(infodict['fvec']) > np.linalg.norm(move_v):    # if distance between hip_d and calculated position > distance between hip_d and hip_c -> error, not found a feasible solution.
            # raise RuntimeError("Not found a feasible solution for the disired hip position.")
            
        d_theta, d_beta = result
        self.theta += d_theta
        self.beta  += d_beta
        self.forward(self.theta, self.beta)
        return self.theta, self.beta

    # Objective function for move
    def objective(self, d_q, current_q, move_vec, contact_rim): # [d_teata, d_beta], [current_teata, current_beta], [delta x, delta y]
        guessed_q = current_q + d_q    # guessed next [theta, beta]
        
        # Assume current hip at [0, 0], solve for theta/beta such that next hip at move_v = next foothold - P, where P is contact point relative to hip and next foothold = current foothold + rolling distance.
        if contact_rim == 1:    # left upper rim
            current_F_exp = ( F_l_poly[0](current_q[0])+1j*F_l_poly[1](current_q[0]) ) *np.exp( 1j*(current_q[1]) )  # current left F          , in current hip coordinate (current hip at [0, 0])
            current_U_exp = ( U_l_poly[0](current_q[0])+1j*U_l_poly[1](current_q[0]) ) *np.exp( 1j*(current_q[1]) )  # current upper rim center, in current hip coordinate (current hip at [0, 0])
            guessed_F_exp = ( F_l_poly[0](guessed_q[0])+1j*F_l_poly[1](guessed_q[0]) ) *np.exp( 1j*(guessed_q[1]) )  # guessed left F          , in guessed next hip coordinate (next hip at 0+j0)
            guessed_U_exp = ( U_l_poly[0](guessed_q[0])+1j*U_l_poly[1](guessed_q[0]) ) *np.exp( 1j*(guessed_q[1]) )  # guessed upper rim center, in guessed next hip coordinate (next hip at 0+j0)
            d_alpha = np.angle( -1j/(guessed_F_exp - guessed_U_exp) ) - np.angle( -1j/(current_F_exp - current_U_exp) )   # delta alpha = next alpha - current alpha
            roll_d = d_alpha * self.radius  # rolling distance caused by rotation of contact point
            next_U    = np.array([current_U_exp.real, current_U_exp.imag]) + np.array([roll_d, 0])   # next_U = current_U + rolling distance, in current hip coordinate (current hip at [0, 0])
            guessed_U = np.array([guessed_U_exp.real, guessed_U_exp.imag])  # in guessed next hip coordinate (next hip at [0, 0])
            guessed_hip = next_U - guessed_U
        elif contact_rim == 2:    # left lower rim
            current_G_exp = 1j*G_poly[1](current_q[0]) *np.exp( 1j*(current_q[1]) )  # current G, in current hip coordinate (current hip at [0, 0])
            current_L_exp = ( L_l_poly[0](current_q[0])+1j*L_l_poly[1](current_q[0]) ) *np.exp( 1j*(current_q[1]) )  # current lower rim center, in current hip coordinate (current hip at [0, 0])
            guessed_G_exp = 1j*G_poly[1](guessed_q[0]) *np.exp( 1j*(guessed_q[1]) )  # in guessed next hip coordinate (next hip at 0+j0)
            guessed_L_exp = ( L_l_poly[0](guessed_q[0])+1j*L_l_poly[1](guessed_q[0]) ) *np.exp( 1j*(guessed_q[1]) )  # guessed lower rim center, in guessed next hip coordinate (next hip at 0+j0)
            d_alpha = np.angle( -1j/(guessed_G_exp - guessed_L_exp) ) - np.angle( -1j/(current_G_exp - current_L_exp) )   # delta alpha = next alpha - current alpha
            roll_d = d_alpha * self.radius  # rolling distance caused by rotation of contact point
            next_L    = np.array([current_L_exp.real, current_L_exp.imag]) + np.array([roll_d, 0])   # next_L = current_L + rolling distance, in current hip coordinate (current hip at [0, 0])
            guessed_L = np.array([guessed_L_exp.real, guessed_L_exp.imag])  # in guessed next hip coordinate (next hip at [0, 0])
            guessed_hip = next_L - guessed_L
        elif contact_rim == 3:    # G
            current_G_exp = 1j*G_poly[1](current_q[0]) *np.exp( 1j*(current_q[1]) )  # current G, in current hip coordinate (current hip at [0, 0])
            guessed_G_exp = 1j*G_poly[1](guessed_q[0]) *np.exp( 1j*(guessed_q[1]) )  # in guessed next hip coordinate (next hip at 0+j0)
            roll_d = -d_q[1] * self.r  # rolling distance caused by rotation of contact point
            next_G    = np.array([current_G_exp.real, current_G_exp.imag]) + np.array([roll_d, 0])   # next_G = current_G + rolling distance, in current hip coordinate (current hip at [0, 0])
            guessed_G = np.array([guessed_G_exp.real, guessed_G_exp.imag])  # in guessed next hip coordinate (next hip at [0, 0])
            guessed_hip = next_G - guessed_G
        elif contact_rim == 4:    # right lower rim
            current_G_exp = 1j*G_poly[1](current_q[0]) *np.exp( 1j*(current_q[1]) )  # current G, in current hip coordinate (current hip at [0, 0])
            current_L_exp = ( L_r_poly[0](current_q[0])+1j*L_r_poly[1](current_q[0]) ) *np.exp( 1j*(current_q[1]) )  # current lower rim center, in current hip coordinate (current hip at [0, 0])
            guessed_G_exp = 1j*G_poly[1](guessed_q[0]) *np.exp( 1j*(guessed_q[1]) )  # in guessed next hip coordinate (next hip at 0+j0)
            guessed_L_exp = ( L_r_poly[0](guessed_q[0])+1j*L_r_poly[1](guessed_q[0]) ) *np.exp( 1j*(guessed_q[1]) )  # guessed lower rim center, in guessed next hip coordinate (next hip at 0+j0)
            d_alpha = np.angle( -1j/(guessed_G_exp - guessed_L_exp) ) - np.angle( -1j/(current_G_exp - current_L_exp) )   # delta alpha = next alpha - current alpha
            roll_d = d_alpha * self.radius  # rolling distance caused by rotation of contact point
            next_L    = np.array([current_L_exp.real, current_L_exp.imag]) + np.array([roll_d, 0])   # next_L = current_L + rolling distance, in current hip coordinate (current hip at [0, 0])
            guessed_L = np.array([guessed_L_exp.real, guessed_L_exp.imag])  # in guessed next hip coordinate (next hip at [0, 0])
            guessed_hip = next_L - guessed_L
        elif contact_rim == 5:    # right upper rim
            current_F_exp = ( F_r_poly[0](current_q[0])+1j*F_r_poly[1](current_q[0]) ) *np.exp( 1j*(current_q[1]) )  # current right F         , in current hip coordinate (current hip at [0, 0])
            current_U_exp = ( U_r_poly[0](current_q[0])+1j*U_r_poly[1](current_q[0]) ) *np.exp( 1j*(current_q[1]) )  # current upper rim center, in current hip coordinate (current hip at [0, 0])
            guessed_F_exp = ( F_r_poly[0](guessed_q[0])+1j*F_r_poly[1](guessed_q[0]) ) *np.exp( 1j*(guessed_q[1]) )  # guessed right F         , in guessed next hip coordinate (next hip at 0+j0)
            guessed_U_exp = ( U_r_poly[0](guessed_q[0])+1j*U_r_poly[1](guessed_q[0]) ) *np.exp( 1j*(guessed_q[1]) )  # guessed upper rim center, in guessed next hip coordinate (next hip at 0+j0)
            d_alpha = np.angle( -1j/(guessed_F_exp - guessed_U_exp) ) - np.angle( -1j/(current_F_exp - current_U_exp) )   # delta alpha = next alpha - current alpha
            roll_d = d_alpha * self.radius  # rolling distance caused by rotation of contact point
            next_U    = np.array([current_U_exp.real, current_U_exp.imag]) + np.array([roll_d, 0])   # next_U = current_U + rolling distance, in current hip coordinate (current hip at [0, 0])
            guessed_U = np.array([guessed_U_exp.real, guessed_U_exp.imag])  # in guessed next hip coordinate (next hip at [0, 0])
            guessed_hip = next_U - guessed_U
        else:
            raise RuntimeError("The leg dosen't contact ground.")
        
        return guessed_hip - move_vec
    

if __name__ == '__main__':
    legmodel = LegModel(sim=True)
    
    #### Forward kinematics ####
    print("****************************************")
    print("****** Forward kinematics example ******")
    print("****************************************")
    # input single value
    print("==========Single Input==========")
    theta = np.deg2rad(130)
    beta  = 0
    legmodel.forward(theta, beta)
    print("Output G with single value input:", legmodel.G)
    legmodel.contact_map(theta, beta)
    print("Output rim with single value input:", legmodel.rim)
    print("Output alpha with single value input:", legmodel.alpha)
    # input array with 1 element
    print("==========1 Element Array Input==========")
    theta = np.array([theta])
    beta  = np.array([beta])
    legmodel.forward(theta, beta)
    print("Output G with 1 element array input:", legmodel.G)
    legmodel.contact_map(theta, beta)
    print("Output rim with 1 element array input:", legmodel.rim)
    print("Output alpha with 1 element array input:", legmodel.alpha)
    # input array with n elements
    print("==========n Elements Array Input==========")
    theta = np.linspace(17, 160, 3)
    theta = np.deg2rad(theta)
    beta = np.linspace(0, 360, 3)
    beta = np.deg2rad(beta)
    legmodel.forward(theta, beta)
    print("Output G with n elements array input:", legmodel.G)
    legmodel.contact_map(theta, beta)
    print("Output rim with n elements array input:", legmodel.rim)
    print("Output alpha with n elements array input:", legmodel.alpha)

    #### Inverse kinematics ####
    print("\n")
    print("****************************************")
    print("****** Inverse kinematics example ******")
    print("****************************************")
    # inverse for G
    print("==========Inverse for G==========")
    G_p = [0.05, -0.25]
    print("Input G:", G_p)
    legmodel.inverse(G_p, 'G')  # can also use " theta, beta = legmodel.inverse(G_p, 'G') "
    print("Output theta, beta (degree):", np.rad2deg(legmodel.theta), np.rad2deg(legmodel.beta))
    print("Output G:", legmodel.G)
    # inverse for left upper rim
    print("==========Inverse for U_l==========")
    Ul_p = [-0.01, -0.015]
    print("Input U_l:", Ul_p)
    legmodel.inverse(Ul_p, 'Ul')
    print("Output theta, beta (degree):", np.rad2deg(legmodel.theta), np.rad2deg(legmodel.beta))
    print("Output U_l:", legmodel.U_l)
    # inverse for right lower rim
    print("==========Inverse for L_r==========")
    Lr_p = [-0.01, -0.015]
    print("Input L_r:", Lr_p)
    legmodel.inverse(Lr_p, 'Lr')
    print("Output theta, beta (degree):", np.rad2deg(legmodel.theta), np.rad2deg(legmodel.beta))
    print("Output L_r:", legmodel.L_r)

    #### Move ####
    print("\n")
    print("**************************")
    print("****** Move example ******")
    print("**************************")
    # current theta, beta, hip
    theta = np.deg2rad(50)
    beta = np.deg2rad(40)
    hip = np.array([0.1, 0])
    desired_hip = np.array([0.2, 0])
    theta, beta = legmodel.move(theta, beta, desired_hip - hip)
    print(f"Use theta = {theta}, beta = {beta} allows the leg to roll from {hip} to {desired_hip} along the ground.")
    
    