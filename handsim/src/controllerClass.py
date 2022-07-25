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
np.set_printoptions(linewidth=200)

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
        # self.usedChannels = [0, 1, 4, 5, 6, 7, 8, 9]

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
        DoF = 3
        muscleType = 'bilinear'
        numChannels = 8
        # self.system_dynamic_model = Hand_1dof(self.device, 4, True, learningRate, 1, 0)
        # self.system_dynamic_model = Hand_4dof(self.device, 8, True, learningRate, 20, 0.3)
        self.system_dynamic_model = upperExtremityModel(muscleType=muscleType, numDoF=DoF, device=self.device, EMG_Channel_Count=numChannels, Dynamic_Lr=learningRate, EMG_mat_Lr=20, NN_ratio=0.3)

        # self.model_save_path = '/home/haptix/haptix/haptix_controller/handsim/Controllers/mikey4DoF-0501_2022.tar'
        self.model_save_path = '/home/haptix/haptix/haptix_controller/handsim/Controllers/JS-0725_2022-3joints.tar'
        checkpoint = torch.load(self.model_save_path, map_location=self.device)
        # checkpoint = torch.load(model_save_path, map_location=torch.device('cpu'))
        self.system_dynamic_model.load_state_dict(checkpoint['model_state_dict'])
        # self.system_dynamic_model.I = [0.4]
        self.system_dynamic_model.eval()

        # set initial conditions
        # self.hidden = torch.FloatTensor([[0,0,0,0,0,0,0,0]]).to(self.device)
        self.hidden = torch.FloatTensor([[0]*DoF*self.system_dynamic_model.numStates]).to(self.device)

    def forwardDynamics(self):
        # allEMG = self.emg.normedEMG
        # usedEMG = allEMG[self.usedChannels]
        # EMG = torch.FloatTensor([usedEMG]).to(self.device)
        EMG = torch.FloatTensor(np.array([self.emg.synergyProd()])).to(self.device)
        # activations = np.clip((EMG @ self.system_dynamic_model.EMG_to_Activation_Mat).detach().cpu().numpy(), 0, None)
        # diffs = []
        # for i in range(4):
        #     diffs.append(activations[0, 2*i] - activations[0, 2*i + 1])
        # print('[', ([f'{i:012.7f}' for i in diffs]), ']')

        with torch.no_grad():
            jointAngles, self.hidden = self.system_dynamic_model(self.hidden, EMG, dt=1/self.LUKEArm.Hz)
        jointAngles = jointAngles.detach().cpu().numpy()
        # jointVels = self.hidden.detach().cpu().numpy()

        jointPos = self.LUKEArm.getCurPos()

        # joint order: [thumbPPos, thumbYPos, indexPos, mrpPos, wristRot, wristFlex, humPos, elbowPos]
        # For PP, 1 AMI for both dof of thumb, 1 AMI for all fingers, 1 AMI for wrist rot, 1 AMI for elbow
        # For JS 07-23, jointAngles = [digits, thumb, wristFlex, wristPro]

        digitsAng = jointAngles[0][0] if jointAngles[0][0] > 0 else 2*jointAngles[0][0]
        flexAng = jointAngles[0][1] if jointAngles[0][1] > 0 else jointAngles[0][1]
        rotAng = jointAngles[0][2] if jointAngles[0][2] > 0 else jointAngles[0][2]

        # digitsAng = jointAngles[0][0] if jointAngles[0][0] > 0 else jointAngles[0][0]
        # thumbAng = jointAngles[0][1] if jointAngles[0][1] > 0 else jointAngles[0][1]
        # flexAng = jointAngles[0][2] if jointAngles[0][2] > 0 else jointAngles[0][2]
        # rotAng = jointAngles[0][3] if jointAngles[0][3] > 0 else jointAngles[0][3]
        # digitsAng = 1.2*jointAngles[0][0] if jointAngles[0][0] > 0 else 5*jointAngles[0][0]
        # thumbAng = 2*jointAngles[0][1] if jointAngles[0][1] > 0 else 2*jointAngles[0][1]
        # flexAng = 1.5*jointAngles[0][2] if jointAngles[0][2] > 0 else 2*jointAngles[0][2]
        # rotAng = 2*jointAngles[0][3] if jointAngles[0][3] > 0 else 1.5*jointAngles[0][3]

        # digitsSpeed = jointVels[0][4]*180/np.pi
        # thumbSpeed = jointVels[0][5]*180/np.pi
        # flexSpeed = jointVels[0][6]*180/np.pi
        # rotSpeed = jointVels[0][7]*180/np.pi

        # jointPos[0] = (thumbAng + 0.6)/1.2*100
        # # jointPos[1] = (thumbAng + 0.6)/1.2*75
        # jointPos[2] = (digitsAng + 0.6)/1.2*90
        # jointPos[3] = (digitsAng + 0.6)/1.2*90
        # jointPos[5] = (flexAng + 0.6)/1.2*110 - 55
        # jointPos[4] = (-rotAng + 0.6)/1.2*295 - 120

        # thresh = np.array([0.2, 0.05, 0.2, 0.1])

        # digits, thumb, flex, rot
        # diffsThreshP = np.array([0.0005, 0.0001, 0.0005, 0.0005])
        # diffsThreshN = -np.array([0.00005, 0.00001, 0.0005, 0.0005])
        # diffsThreshP = np.zeros(4)
        # diffsThreshN = np.zeros(4)

        # newThumb = (thumbAng + 5*np.pi/18)/(5*np.pi/9)*100
        # (thumbAng + 0.6)/1.2*75
        newDigits = (digitsAng + np.pi/4)/(np.pi/2)*90
        newThumb = (digitsAng + np.pi/4)/(np.pi/2)*100
        # (digitsAng + np.pi/4)/(np.pi/2)*90
        newFlex = (-flexAng + 55*np.pi/180)/(55*np.pi/180)*110 - 95
        newRot = (rotAng + 175*np.pi/180)/(295*np.pi/180)*295 - 175
        # newPos = [newThumb, newDigits, newFlex, newRot]

        # percentRoms = [np.abs(newThumb - jointPos[0])/100, np.abs(newDigits - jointPos[2])/90, np.abs(newRot - jointPos[4])/295, np.abs(newFlex - jointPos[5])/110]
        # percentSpeed = np.abs([thumbSpeed/100, digitsSpeed/90, rotSpeed/295, flexSpeed/110])
        # mostMove = np.max(percentRoms)
        # fastestMove = np.max(percentSpeed)

        # moveBool = np.asarray(percentRoms) > thresh
        # diffsBool = np.logical_or(diffs[0] > diffsThreshP, diffs[0] < diffsThreshN)
        # print('[', ([f'{i}' for i in diffsBool]), ']')

        jointPos[0] = newThumb
        jointPos[1] = newThumb
        jointPos[2] = newDigits
        jointPos[3] = newDigits
        jointPos[4] = newRot
        jointPos[5] = newFlex

        # jointPos[0] = newThumb if moveBool[0] else jointPos[0]
        # jointPos[1] = newThumb if moveBool[0] else jointPos[1]
        # jointPos[2] = newDigits if moveBool[1] else jointPos[3]
        # jointPos[3] = newDigits if moveBool[1]  else jointPos[3]
        # jointPos[4] = newRot if moveBool[2] else jointPos[4]
        # jointPos[5] = newFlex if moveBool[3] else jointPos[5]

        # jointPos[0] = newThumb if diffsBool[1] else jointPos[0]
        # jointPos[1] = newThumb if diffsBool[1] else jointPos[0]
        # jointPos[2] = newDigits if diffsBool[0] else jointPos[3]
        # jointPos[3] = newDigits if diffsBool[0]  else jointPos[3]
        # jointPos[4] = newRot if diffsBool[3] else jointPos[4]
        # jointPos[5] = newFlex if diffsBool[2] else jointPos[5]

        # jointPos[0] = newThumb if percentRoms[0] == mostMove else jointPos[0]
        # # jointPos[1] = newThumb if percentRoms[0] == mostMove else jointPos[1]
        # jointPos[2] = newDigits if percentRoms[1] == mostMove else jointPos[3]
        # jointPos[3] = newDigits if percentRoms[1] == mostMove else jointPos[3]
        # jointPos[4] = newRot if percentRoms[2] == mostMove else jointPos[4]
        # jointPos[5] = newFlex if percentRoms[3] == mostMove else jointPos[5]

        # jointPos[0] = newThumb if percentSpeed[0] == fastestMove else jointPos[0]
        # jointPos[1] = newThumb if percentRoms[0] == mostMove else jointPos[1]
        # jointPos[2] = newDigits if percentSpeed[1] == fastestMove else jointPos[3]
        # jointPos[3] = newDigits if percentSpeed[1] == fastestMove else jointPos[3]
        # jointPos[4] = newRot if percentSpeed[2] == fastestMove else jointPos[4]
        # jointPos[5] = newFlex if percentSpeed[3] == fastestMove else jointPos[5]

        # jointPos = self.rateLimit(jointPos)

        return jointPos

    # applies a velocity controller when the joint command is out of the joint's range
    def rateLimit(self, posCom):
        newPosCom = [0]*len(posCom)
        rateLim = np.array(([50]*4 + [25]*4))
        kp = 15*np.ones(8)
        curPos = np.asarray(self.LUKEArm.getCurPos())

        gains = np.multiply(kp, np.sign(posCom - curPos))
        newPosCom = np.where(np.abs(posCom - curPos) > rateLim, curPos + gains, posCom)
        # newPosCom = curPos + np.multiply(diff, np.abs(posCom - curPos))

        print(newPosCom == posCom)

        return newPosCom

    def PIDcontroller(self, posCom):
        curPos = np.asrray(self.LUKEArm.getCurPos()) # reference position

        error = posCom - curPos

        self.I = np.where(np.abs(self.I + error/self.LUKEArm.Hz) < self.windup, self.I + error/self.LUKEArm.Hz)
        derivError = (self.lastError - error)*self.LUKEArm.Hz
        self.lastError = error # store last error

        PIDCommand = self.k_p*error + self.k_i*self.I + self.k_d*derivError

        return PIDCommand + curPos
