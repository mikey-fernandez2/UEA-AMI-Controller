# BUGMAN Feb 1 2022
# The whole model for two degree of freedom ankle
"""

"""

from re import I
import torch
from torch import nn
from torch.functional import F

from Dynamics2.Muscle import Muscle
from Dynamics2.Joint_1dof_Bilinear_NN import Joint_1dof

# For Testing
# from Muscle import Muscle
# from Joint_2dof import Joint_2dof

class Hand_4dof(nn.Module):
    def __init__(self, device, EMG_Channel_Count, Left, Dyanmic_Lr, EMG_mat_Lr, NN_ratio, \
                 K0_scale=2000, K1_scale=40000, L0_scale=0.03, L1_scale=0.006, I_scale=0.008, M_scale=0.05, speed_mode=False):
        super().__init__()
        
        self.device = device
        self.EMG_Channel_Count = EMG_Channel_Count
        self.NN_ratio = NN_ratio
        self.EMG_mat_Lr = EMG_mat_Lr
        
        # Muscles
        
        m1 = Muscle(100, 2000, 0.06, 0.006, [-0.05])
        m2 = Muscle(100, 2000, 0.06, 0.006, [0.05])
        
        m3 = Muscle(100, 2000, 0.06, 0.006, [-0.05])
        m4 = Muscle(100, 2000, 0.06, 0.006, [0.05])
        
        m5 = Muscle(100, 2000, 0.06, 0.006, [-0.05])
        m6 = Muscle(100, 2000, 0.06, 0.006, [0.05])
        
        m7 = Muscle(100, 2000, 0.06, 0.006, [-0.05])
        m8 = Muscle(100, 2000, 0.06, 0.006, [0.05])
        
        self.muscles = [m1, m2, m3, m4, m5, m6, m7, m8]
         
        self.AMI1 = [m1, m2]
        self.AMI2 = [m3, m4]
        self.AMI3 = [m5, m6]
        self.AMI4 = [m7, m8]
        
        self.muscle_num = len(self.muscles)

        # Inertia
        self.I = [0.004]
        
        # Joint
        self.Joint1 = Joint_1dof(self.device, self.AMI1, inertias = self.I, Lr_scale = Dyanmic_Lr, NN_ratio=NN_ratio, \
                                K0_scale = K0_scale, K1_scale = K1_scale, L0_scale =  L0_scale, \
                                L1_scale = L1_scale, I_scale = I_scale, M_scale = M_scale, speed_mode = speed_mode)
        
        self.Joint2 = Joint_1dof(self.device, self.AMI2, inertias = self.I, Lr_scale = Dyanmic_Lr, NN_ratio=NN_ratio, \
                                K0_scale = K0_scale, K1_scale = K1_scale, L0_scale =  L0_scale, \
                                L1_scale = L1_scale, I_scale = I_scale, M_scale = M_scale, speed_mode = speed_mode)
        
        self.Joint3 = Joint_1dof(self.device, self.AMI3, inertias = self.I, Lr_scale = Dyanmic_Lr, NN_ratio=NN_ratio, \
                                K0_scale = K0_scale, K1_scale = K1_scale, L0_scale =  L0_scale, \
                                L1_scale = L1_scale, I_scale = I_scale, M_scale = M_scale, speed_mode = speed_mode)
        
        self.Joint4 = Joint_1dof(self.device, self.AMI4, inertias = self.I, Lr_scale = Dyanmic_Lr, NN_ratio=NN_ratio, \
                                K0_scale = K0_scale, K1_scale = K1_scale, L0_scale =  L0_scale, \
                                L1_scale = L1_scale, I_scale = I_scale, M_scale = M_scale, speed_mode = speed_mode)
        
        # EMG to Muscle activation matrix
        # There will be two initialization style of the EMG to muscle activation matrix.
        # If the electrodes are right upon the targeted muscle eye matrix should be chose, otherwise all-one matrix should be the way to go 
        # I'm using the scaled all-one matrix implementation here.
        # self.EMG_to_Activation_Mat = nn.Parameter(torch.ones((self.EMG_Channel_Count, self.muscle_num ), dtype=torch.float, device=self.device)/self.EMG_mat_Lr/self.EMG_Channel_Count)
        self.EMG_to_Activation_Mat = nn.Parameter(torch.eye(self.EMG_Channel_Count, self.muscle_num , dtype=torch.float, device=self.device)/self.EMG_mat_Lr)
        
    def forward(self, SS, EMG, dt):
        # Get the muscle activations then pass them into the joint model.
        Alphas = torch.matmul(EMG[:,0:self.EMG_Channel_Count], self.EMG_to_Activation_Mat*self.EMG_mat_Lr)
        
        # SS1 = SS[:,0:2]
        # SS2 = SS[:,2:4]
        # SS3 = SS[:,4:6]
        # SS4 = SS[:,6:8]
        
        
        # TODO: #3 Add nonlinear calculation to EMG
        # print(Alphas)
        rw1, rs1 = self.Joint1(SS[:, 0:2], Alphas[:, 0:2], dt)
        rw2, rs2 = self.Joint2(SS[:, 2:4], Alphas[:, 2:4], dt)
        rw3, rs3 = self.Joint3(SS[:, 4:6], Alphas[:, 4:6], dt)
        rw4, rs4 = self.Joint4(SS[:, 6:8], Alphas[:, 6:8], dt)
        
        rw = torch.hstack([rw1, rw2, rw3, rw4])
        rs = torch.hstack([rs1, rs2, rs3, rs4])
        
        return rw, rs

    def lock_EMG_mat(self, switch=True):
        self.EMG_to_Activation_Mat.requires_grad = not switch
        
    def lock_I(self, switch=True):
        self.Joint1.I.requires_grad = not switch
        self.Joint2.I.requires_grad = not switch
        self.Joint3.I.requires_grad = not switch
        self.Joint4.I.requires_grad = not switch
        
    def lock_for_robot(self, switch=True):
        self.lock_EMG_mat(switch)
        self.lock_I(switch)
        
    def disable_NN(self):
        self.Joint1.disalbe_NN()
        self.Joint2.disalbe_NN()
        self.Joint3.disalbe_NN()
        self.Joint4.disalbe_NN()
        
    def enable_NN(self):
        self.Joint1.enable_NN()
        self.Joint2.enable_NN()
        self.Joint3.enable_NN()
        self.Joint4.enable_NN()
        
    def print_params(self):
        # print all parameters of the dynamic model
        self.Joint1.print_params()
        self.Joint2.print_params()
        self.Joint3.print_params()
        self.Joint4.print_params()
        print("EMG to Muscle Activation mat:\n", (self.EMG_to_Activation_Mat*self.EMG_mat_Lr).detach().cpu().numpy())

# Test
"""
device = torch.device("cpu")
model = Ankle_2dof(device, 4, Left=True, Dyanmic_Lr=5, EMG_mat_Lr=20.0, NN_ratio=0.0)
model.eval()
X = torch.FloatTensor([[0,0,0,0],[0,0,0,0],[1,0,0,0]])
a1 = torch.FloatTensor([[0,0,0,0],[0.1,0,1,0],[0,0,1,0]])
for i in range(10):
    X = model(X,a1)
    print(X)
    # print(X[1,0])
"""