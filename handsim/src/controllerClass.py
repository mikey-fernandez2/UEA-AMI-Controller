# Mikey Fernandez 12/13/2021
#
# impedanceController.py
# Make the impedance controller class

# need to add the location of Junqing's model + functions to the path
import sys
# sys.path.append("/home/haptix/haptix/haptix_controller/handsim/MinJerk")
sys.path.append('/home/haptix/haptix/Upper Extremity Models/Upper Extremity Shadmehr')

# from Dynamics2.Hand_1dof import Hand_1dof
# from Dynamics2.Hand_4dof import Hand_4dof
from Dynamics2.upperLimbModel import upperExtremityModel
import torch
from torch.utils.data import DataLoader
import numpy as np

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
        self.usedChannels = [0, 1, 4, 5, 6, 7, 8, 9]

        self.k_p = [1]*self.LUKEArm.numMotors
        self.k_i = [.001]*self.LUKEArm.numMotors
        self.k_d = [.1]*self.LUKEArm.numMotors

        self.I = [0]*self.LUKEArm.numMotors
        self.lastError = [0]*self.LUKEArm.numMotors
        self.windup = [100]*self.LUKEArm.numMotors

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
        Wn = 2*np.pi*self.freq_n # desired natural frequency
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
        DoF = 4
        muscleType = 'bilinear'
        numChannels = 8
        # self.system_dynamic_model = Hand_1dof(self.device, 4, True, learningRate, 1, 0)
        # self.system_dynamic_model = Hand_4dof(self.device, 8, True, learningRate, 20, 0.3)
        self.system_dynamic_model = upperExtremityModel(muscleType=muscleType, numDoF=DoF, device=self.device, EMG_Channel_Count=numChannels, Dynamic_LR=learningRate, EM_mat_lr=20, NN_ratio=0.3)

        # self.model_save_path = '/home/haptix-e15-463/haptix/haptix_controller/handsim/MinJerk/wrist.tar'
        # self.model_save_path = '/home/haptix/UE AMI Clinical Work/P1 - 729/P1_0307_2022/P1_0307_2022v2_upper.tar'
        # self.model_save_path = '/home/haptix/haptix/haptix_controller/handsim/Controllers/Mikey2DoF-04-12-22.tar'
        self.model_save_path = '/home/haptix/haptix/haptix_controller/handsim/Controllers/mikey4DoF-0501_2022.tar'
        checkpoint = torch.load(self.model_save_path, map_location=self.device)
        # checkpoint = torch.load(model_save_path, map_location=torch.device('cpu'))
        self.system_dynamic_model.load_state_dict(checkpoint['model_state_dict'])
        # self.system_dynamic_model.I = [0.4]
        self.system_dynamic_model.eval()

        # set initial conditions
        # self.hidden = torch.FloatTensor([[0,0,0,0,0,0,0,0]]).to(self.device)
        self.hidden = torch.FloatTensor([[0]*DoF*self.system_dynamic_model.numStates]).to(self.device)


    def forwardDynamics(self):
        allEMG = self.emg.normedEMG
        usedEMG = allEMG[self.usedChannels]
        EMG = torch.FloatTensor([usedEMG]).to(self.device)

        # joint1, joint2, joint3, joint4, self.hidden1, self.hidden2, self.hidden3, self.hidden4 = self.system_dynamic_model.forward(EMG, self.hidden1, self.hidden2, self.hidden3, self.hidden4, dt=1/self.LUKEArm.Hz)
        with torch.no_grad():
            jointAngles, self.hidden = self.system_dynamic_model(self.hidden, EMG, dt=1/self.LUKEArm.Hz)
        jointAngles = jointAngles.detach().cpu().numpy()

        jointPos = self.LUKEArm.getCurPos()

        # joint order: [thumbPPos, thumbYPos, indexPos, mrpPos, wristRot, wristFlex, humPos, elbowPos]
        # For PP, 1 AMI for both dof of thumb, 1 AMI for all fingers, 1 AMI for wrist rot, 1 AMI for elbow

        elbowAng = jointAngles[0][0] if jointAngles[0][0] > 0 else jointAngles[0][0]
        digitsAng = jointAngles[0][1] if jointAngles[0][1] > 0 else jointAngles[0][1]
        indexAng = jointAngles[0][2] if jointAngles[0][2] > 0 else jointAngles[0][2]
        wristAng = jointAngles[0][3] if jointAngles[0][3] > 0 else jointAngles[0][3]
        jointPos[7] = 0.75*((elbowAng + 1.2)/2.4*135)
        jointPos[2] = (indexAng + 0.6)/1.2*90
        jointPos[3] = (digitsAng + 0.6)/1.2*90
        jointPos[4] = 0.33*((wristAng + 0.6)/1.2*295 - 120)

        return jointPos

    # applies a velocity controller when the joint command is out of the joint's range
    def rateLimit(self, posCom):
        newPosCom = [0]*len(posCom)
        rateLim = [30, 30, 30, 30, 10, 10, 10, 30]
        kp = [22.5, 22.5, 22.5, 22.5, 7.5, 7.5, 7.5, 15]
        curPos = np.asarray(self.LUKEArm.getCurPos())

        gains = np.where(posCom > curPos, kp, -kp)
        diff = np.where(np.abs(posCom - curPos) > rateLim, gains, 0)
        newPosCom = curPos + diff

        # for i in range(self.LUKEArm.numMotors):
        #     if abs(posCom[i] - curPos[i]) > rateLim[i]:
        #         diff = kp[i] if posCom[i] > curPos[i] else -kp[i]
        #     else:
        #         diff = 0
        #     newPosCom[i] = curPos[i] + diff
        # return newPosCom

    def PIDcontroller(self, posCom):
        curPos = np.asrray(self.LUKEArm.getCurPos()) # reference position

        # error = [posCom[i] - curPos[i] for i in range(len(posCom))] # position error
        error = posCom - curPos

        # self.I = [(self.I[i] + error[i]/self.LUKEArm.Hz if abs(self.I[i] + error[i]/self.LUKEArm.Hz) < self.windup[i] else 0) for i in range(len(posCom))] # integrate error, with windup built in
        self.I = np.where(np.abs(self.I + error/self.LUKEArm.Hz) < self.windup, self.I + error/self.LUKEArm.Hz)
        # derivError = [(self.lastError[i] - error[i])*self.LUKEArm.Hz for i in range(len(posCom))] # get the derivative error
        derivError = (self.lastError - error)*self.LUKEArm.Hz
        self.lastError = error # store last error

        # PIDCommand = [self.k_p[i]*error[i] + self.k_i[i]*self.I[i] + self.k_d[i]*derivError[i] for i in range(len(posCom))] # get PID command
        PIDCommand = self.k_p*error + self.k_i*self.I + self.k_d*derivError

        # return [PIDCommand[i] + curPos[i] for i in range(len(posCom))] # increment the current position by the PID command
        return PIDCommand + curPos