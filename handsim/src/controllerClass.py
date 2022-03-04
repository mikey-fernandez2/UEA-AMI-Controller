# Mikey Fernandez 12/13/2021
#
# impedanceController.py
# Make the impedance controller class

# need to add the location of Junqing's model + functions to the path
import sys
sys.path.append("/home/haptix-e15-463/haptix/haptix_controller/handsim/MinJerk")

import math
# from Dynamics.AMI_Joint_Dynamic import AMI_Joint
# from Dynamics.Muscle import Muscle
# from Dynamics.System_Dynamic_Model import System_Dynamic_Model
from Dynamics2.Hand_1dof import Hand_1dof
from Dynamics2.Hand_4dof import Hand_4dof
import torch
from torch.utils.data import DataLoader
import time

class LUKEControllers:
    def __init__(self, numMotors=8, freq_n=3, numElectrodes=16, LUKEArm=None, emg=None):
        self.LUKEArm = LUKEArm
        self.emg = emg
        self.numMotors = numMotors
        self.freq_n = freq_n
        self.numElectrodes = numElectrodes

        self.prevT = [0]*numMotors
        self.prev2T = [0]*numMotors

        # [thumbP, thumbY, index, mrp, wristRot, wristFlex, humRot, elbow]
        self.motorElectrodeMap = [[3, 4], [0, 0], [10, 9], [10, 9], [6, 5], [0, 0], [0, 0], [2, 0]]
        self.K_act_arr = [1]*self.numMotors
        self.K_pas_arr = [0.01]*self.numMotors

        self.gainMatrix = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # thumbP 
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # thumbY 
                           [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # index
                           [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # mrp
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # wristRot
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # wristFlex
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # humRot
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]  # elbow

        self.motorMap = {0: 'thumbPPos', 1:'thumbYPos', 2:'indexPos', 3:'mrpPos', 4:'wristRot', 5:'wristFlex', 6:'humPos', 7:'elbowPos'}
        self.th_0 = [0, 0, 0, 0, 0, 0, 0, 0]

        # set the used EMG channels here
        # self.usedChannels = [9, 3] # for wrist
        self.usedChannels = [0, 2, 3, 4, 5, 6, 9, 10]

        self.k_p = [1]*self.LUKEArm.numMotors
        self.k_i = [.001]*self.LUKEArm.numMotors
        self.k_d = [.1]*self.LUKEArm.numMotors

        self.I = [0 for i in range(self.LUKEArm.numMotors)]
        self.lastError = [0 for i in range(self.LUKEArm.numMotors)]
        self.windup = [100 for i in range(self.LUKEArm.numMotors)]

    # get the electrodes corresponding to the agonist and antagonist muscles for given motor/joint
    def getElectrodesforMotor(self, motor):
        # motorMap[i][0] is the electrode corresponding to the agonist muscle for motor i
        # motorMap[i][1] is the electrode corresponding to the antagonist muscle for motor i
        agonist = self.motorElectrodeMap[motor][0]
        antagonist = self.motorElectrodeMap[motor][1]

        return (agonist, antagonist)

    def generateGainsforMotor(self, motor):
        gains = []
        for i in range(self.numElectrodes):
            gains.append(self.gainMatrix[motor][i])

        return gains

    def secondOrderDynamics(self, T_des):
        # calculates command = wn^2/(s^2 + 2*zeta*wn*s + wn^2) [unit gain]
        Wn = 2*math.pi*self.freq_n # desired natural frequency
        b = 2*Wn                   # 2*zeta*Wn, but critically damped so zeta = 1
        k = Wn**2                  # k = Wn^2

        Ts = 1/self.emg.samplingFreq
        Ts2 = Ts**2
        k1 = (1/Ts2 + b/Ts + k)
        k2 = -(2/Ts2 + b/Ts)
        k3 = 1/Ts2

        prev = self.prevT
        prev2 = self.prev2T

        commands = []
        for i in range(self.numMotors):
            commands.append(k/k1*T_des[i] - k2/k1*prev[i] - k3/k1*prev2[i])
        
        # update previous commands in struct
        self.prev2T = self.prevT.copy()
        self.prevT = commands.copy()

        return commands

    def motorIntents(self):
        intents = []
        for i in range(self.numMotors):
            intents.append(sum(x*y for x, y in zip(self.emg.muscleAct, self.generateGainsforMotor(i))))

        return intents

    def normalizeTorques(self, intents):
        maxTorques = [1]*self.numMotors
        minTorques = [1]*self.numMotors

        normedTorques = []
        for i in range(self.numMotors):
            if intents[i] > 0:
                normedTorques.append(intents[i]/maxTorques[i])
            else:
                normedTorques.append(intents[i]/minTorques[i])

        return normedTorques

    def normalizeCommands(self, T_filt):
        p_filt = [0]*self.numMotors
        for i in range(self.numMotors):
            motor = self.motorMap[i]
            RoM = self.LUKEArm.jointRoM[motor]
            p = (RoM[1] - RoM[0])*T_filt[i]

            if p > RoM[1]: p = RoM[1]
            elif p < RoM[0]: p = RoM[0]

            p_filt[i] = p

        return p_filt

    def calculateEMGCommand(self):
        # Control law:
        # T_des[i] = K_active[i]*dot(gains[i]*muscleAct)/TorqueNorm[i] - K_passive[i]*(motorPos[i] - th_0[i])
        #
        #   where:
        #     T_des[i] is the desired motor torque (position) input to motor i
        #     K_active[i] is the active torque gain for motor i
        #     gains[i] are the 16 synergy gains for the DoF controlled by motor i
        #     muscleAct are the current muscle activation values
        #     TorqueNorm[i] is the normalization factor from the synergy motor intent for motor i
        #     K_passive[i] is the pasive spring stiffness for motor i (for opening the hand)
        #     motorPos[i] is the position of motor i
        #     th_0[i] is the reference position of motor i for the passive spring

        assert self.LUKEArm.usingEMG, 'calculateCommand(): Using EMG command when arm not set up with EMG'

        intents = self.motorIntents()
        normedTorques = self.normalizeTorques(intents)
        
        T_des = []
        for i in range(self.numMotors):
            T_act = self.K_act_arr[i]*normedTorques[i]
            T_pas = self.K_pas_arr[i]*(self.LUKEArm.sensors[self.motorMap[i]] - self.th_0[i])
            T_des.append(T_act + T_pas)

        T_filt = self.secondOrderDynamics(T_des)

        # T_filt should be between ~[-1, 1]
        # do some stupid scaling so its between [0, 1]
        T_filt = [(i + 1)/2 for i in T_filt]
        p_filt = self.normalizeCommands(T_filt)
        # posCom = p_filt

        ## FOR TESTING - SET JUST ONE JOINT
        posCom = self.LUKEArm.buildCurPosCommand()
        posCom[5] = p_filt[5]

        return posCom

    def differentialActCommand(self, threshold, gain):
        # get arm current position
        curPos = self.LUKEArm.getCurPos()
        lastCom = self.LUKEArm.lastposCom

        # get the difference in activation between the two muscles
        diffs = [self.emg.muscleAct[elec[0]] - self.emg.muscleAct[elec[1]] for elec in self.motorElectrodeMap]

        # then return a directional array if the difference is above threshold and based on direction of movement
        # if the value is 0, below threshold - dont move
        # if value is 1, increase the angle
        # if value is -1, decrease the angle
        direction = [0 if abs(diff) < threshold else (1 if diff > 0 else -1) for diff in diffs]
        newCom = []
        for i in range(self.numMotors):
            motor = self.motorMap[i]
            limits = self.LUKEArm.jointRoM[motor]
            RoM = limits[1] - limits[0]
            diff = gain*RoM*direction[i]
            # thisNew = lastCom[i] + diff
            thisNew = curPos[i] + diff

            # check bounds
            thisNew = limits[0] if thisNew <= limits[0] else (limits[1] if thisNew >= limits[1] else thisNew)
            
            newCom.append(thisNew)
            # if motor == 'elbowPos': print(f"{motor}: {lastCom[i]:06.3f}, {diff:06.3f}, {thisNew:06.3f}, {self.LUKEArm.posToCAN(thisNew, motor):06.3f}")

        # print(f"newCom: {newCom}\n")
        return newCom

    def resetModel(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Build whole model based on muscles and masses
        learningRate = 5
        # self.system_dynamic_model = Hand_1dof(self.device, 4, True, learningRate, 1, 0)
        self.system_dynamic_model = Hand_4dof(self.device, 4, True, learningRate, 20, 0.3)


        # self.model_save_path = '/home/haptix-e15-463/haptix/haptix_controller/handsim/MinJerk/wrist.tar'
        self.model_save_path = '/home/haptix-e15-463/haptix/haptix_controller/handsim/MinJerk/upper.tar'
        checkpoint = torch.load(self.model_save_path, map_location=self.device)
        # checkpoint = torch.load(model_save_path, map_location=torch.device('cpu'))
        self.system_dynamic_model.load_state_dict(checkpoint['model_state_dict'])
        # self.system_dynamic_model.I = [0.4]
        self.system_dynamic_model.eval()

        # set initial conditions
        self.hidden = torch.FloatTensor([[0,0,0,0,0,0,0,0]]).to(self.device)

    def forwardDynamics(self):
        allEMG = self.emg.normedEMG
        usedEMG = allEMG[self.usedChannels]
        EMG = torch.FloatTensor([usedEMG]).to(self.device)

        # joint1, joint2, joint3, joint4, self.hidden1, self.hidden2, self.hidden3, self.hidden4 = self.system_dynamic_model.forward(EMG, self.hidden1, self.hidden2, self.hidden3, self.hidden4, dt=1/self.LUKEArm.Hz)
        # wristAngle, self.hidden = self.system_dynamic_model(self.hidden, EMG, dt = 1/self.LUKEArm.Hz)
        with torch.no_grad():
            jointAngles, self.hidden = self.system_dynamic_model(self.hidden, EMG, dt=1/self.LUKEArm.Hz)
        # posDegrees = [math.degrees(rad) for rad in jointAngles] # convert the radian output to degrees
        # wristAngle = math.degrees(wristAngle)

        jointPos = self.LUKEArm.getCurPos()

        # joint order: [thumbPPos, thumbYPos, indexPos, mrpPos, wristRot, wristFlex, humPos, elbowPos]
        # TODO For PP, 1 AMI for both dof of thumb, 1 AMI for all fingers, 1 AMI for wrist rot, 1 AMI for elbow
        # print(wristAngle)
        # jointPos[2] = 0.5*(90 + wristAngle)
        # jointPos[3] = jointPos[2]
        # jointPos[0] = jointPos[2]
        # jointPos[1] = jointPos[2]

        # print(jointAngles.detach().numpy())

        jointPos[7] = (jointAngles[0][0].detach().cpu().numpy() + 1.2)/2.4*135
        jointPos[2] = (-(jointAngles[0][1].detach().cpu().numpy()) + 0.6)/1.2*90
        jointPos[3] = (-(jointAngles[0][1].detach().cpu().numpy()) + 0.6)/1.2*90
        jointPos[0] = (-jointAngles[0][1].detach().cpu().numpy() + 0.6)/1.2*75
        jointPos[1] = 37.5
        # jointPos[5] = 0
        # jointPos[0] = (-jointAngles[0][2] + 0.6)/1.2*75
        # jointPos[4] = (jointAngles[0][3].detach().cpu().numpy() + .178 + 1.4)/2.8*295 - 120

        # jointPos[7] = ((jointAngles[0][0].detach().numpy() - 0.0413)/.1 + 0.05)*135
        # jointPos[2] = ((jointAngles[0][1].detach().numpy() - 0.0594)/.1 + 0.05)*90
        # jointPos[3] = ((jointAngles[0][1].detach().numpy() - 0.0594)/.1 + 0.05)*90
        # jointPos[0] = ((jointAngles[0][2].detach().numpy() + 0.0548)/.09 + 0.05)*75
        # jointPos[4] = 0.5*(((jointAngles[0][3].detach().numpy() -.1527)/.62 + 0.31)*295 - 120)

        # print(jointAngles)

        return jointPos

    # applies a velocity controller when the joint command is out of the joint's range

    def rateLimit(self, posCom):
        newPosCom = [0]*len(posCom)
        rateLim = [30, 30, 30, 30, 10, 10, 10, 30]
        kp = [22.5, 22.5, 22.5, 22.5, 7.5, 7.5, 7.5, 15]
        curPos = self.LUKEArm.getCurPos()
        for i in range(self.LUKEArm.numMotors):
            if abs(posCom[i] - curPos[i]) > rateLim[i]:
                diff = kp[i] if posCom[i] > curPos[i] else -kp[i]
            else: diff = 0
            newPosCom[i] = curPos[i] + diff
        return newPosCom

    def PIDcontroller(self, posCom):
        curPos = self.LUKEArm.getCurPos() # reference position

        error = [posCom[i] - curPos[i] for i in range(len(posCom))] # position error

        self.I = [(self.I[i] + error[i]/self.LUKEArm.Hz if abs(self.I[i] + error[i]/self.LUKEArm.Hz) < self.windup[i] else 0) for i in range(len(posCom))] # integrate error, with windup built in
        derivError = [(self.lastError[i] - error[i])*self.LUKEArm.Hz for i in range(len(posCom))] # get the derivative error
        self.lastError = error # store last error

        PIDCommand = [self.k_p[i]*error[i] + self.k_i[i]*self.I[i] + self.k_d[i]*derivError[i] for i in range(len(posCom))] # get PID command

        return [PIDCommand[i] + curPos[i] for i in range(len(posCom))] # increment the current position by the PID command