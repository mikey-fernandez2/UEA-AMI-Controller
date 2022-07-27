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
from BesselFilter import BesselFilterArr
from CausalButter import CausalButterArr, CausalButter

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

        self.probFilter = BesselFilterArr(numChannels=3, order=4, critFreqs=[3], fs=self.LUKEArm.Hz, filtType='lowpass')

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
        DoF = 5
        muscleType = 'bilinear'
        numChannels = 8
        # self.system_dynamic_model = Hand_4dof(self.device, 8, True, learningRate, 20, 0.3)
        self.system_dynamic_model = upperExtremityModel(muscleType=muscleType, numDoF=DoF, device=self.device, EMG_Channel_Count=numChannels, Dynamic_Lr=learningRate, EMG_mat_Lr=20, NN_ratio=0.3)

        self.model_save_path = '/home/haptix/haptix/haptix_controller/handsim/Controllers/JS-0726_2022_5DoFmult.tar'
        checkpoint = torch.load(self.model_save_path, map_location=self.device)
        self.system_dynamic_model.load_state_dict(checkpoint['model_state_dict'])
        self.system_dynamic_model.eval()

        # set initial conditions
        self.hidden = torch.FloatTensor([[0]*DoF*self.system_dynamic_model.numStates]).to(self.device)

    def forwardDynamics(self):
        # allEMG = self.emg.normedEMG
        # usedEMG = allEMG[self.usedChannels]
        # EMG = torch.FloatTensor([usedEMG]).to(self.device)
        EMG = torch.FloatTensor(np.array([self.emg.synergyProd()])).to(self.device)

        with torch.no_grad():
            jointAngles, self.hidden, predictions = self.system_dynamic_model(self.hidden, EMG, dt=1/self.LUKEArm.Hz)
        jointAngles = jointAngles.detach().cpu().numpy()
        probabilities = predictions.detach().cpu().numpy()[0]

        # probabilities = self.softmax(predictions[0]) # get probabilities

        # if probabilities[0] > 0.3: probabilities[0] += 0.5*probabilities[1]; probabilities[1] = 0
        # elif probabilities[1] > 0.5: probabilities[1] += 0.5*probabilities[0]; probabilities[0] = 0
        # elif probabilities[2] > 0.2 and probabilities[0] < 0.75: probabilities[0] *= 0.25
        # if probabilities[0] > probabilities[1]: probabilities[1] = 0
        # elif probabilities[1] > probabilities[0]: probabilities[0] = 0

        # probabilities = probabilities/np.sum(probabilities) # renormalize probabilities
        # probabilities = self.probFilter.filter([probabilities])[0]
        print([f'{prob:07.6f}' for prob in probabilities])

        jointPos = self.LUKEArm.getCurPos()

        moveBool = probabilities > np.array([0.3, 0.5, 0.5, 0.3, 0.5])

        # joint order: [thumbPPos, thumbYPos, indexPos, mrpPos, wristRot, wristFlex, humPos, elbowPos]
        # For PP, 1 AMI for both dof of thumb, 1 AMI for all fingers, 1 AMI for wrist rot, 1 AMI for elbow
        # For JS 07-23, jointAngles = [digits, thumb, wristFlex, wristPro]
        # For JS 07-26 (old) jointAngles = [digits, wristFlex, wristPro]
        # For JS 07-26 jointAngles = [thumb, index, digits, wristPro, wristFlex]

        thumbAng = 2*jointAngles[0][0] if jointAngles[0][0] > 0 else 2*jointAngles[0][0]
        indexAng = jointAngles[0][1] if jointAngles[0][1] > 0 else jointAngles[0][1]
        digitsAng = 3*jointAngles[0][2] if jointAngles[0][2] > 0 else 2*jointAngles[0][2]
        rotAng = jointAngles[0][3] if jointAngles[0][3] > 0 else 2*jointAngles[0][3]
        flexAng = jointAngles[0][4] if jointAngles[0][4] > 0 else jointAngles[0][4]

        # predictedJoint = np.argmax(predictions)
        # self.predictionHistory = np.roll(self.predictionHistory, -1)
        # self.predictionHistory[-1] = predictedJoint
        # elems, counts = np.unique(self.predictionHistory, return_counts=True)
        # mostCommon = np.argmax(counts)
        # predictedJoint = elems[mostCommon]
        # print(predictedJoint)

        newThumb = (thumbAng + 0.13 + 5*np.pi/18)/(5*np.pi/9)*100
        newIndex = (indexAng + 0.046 + np.pi/4)/(np.pi/2)*90
        newDigits = (digitsAng + 0.144 + np.pi/4)/(np.pi/2)*90
        newRot = (rotAng - 0.1 + 175*np.pi/180)/(295*np.pi/180)*295 - 175
        newFlex = (flexAng + 0.03 + 55*np.pi/180)/(110*np.pi/180)*110 - 55

        # jointPos[0] = jointPos[0] + (newThumb - jointPos[0])*probabilities[0]
        # jointPos[1] = jointPos[0] + (newThumb - jointPos[0])*probabilities[0]
        # jointPos[2] = jointPos[2] + (newIndex - jointPos[2])*probabilities[1]
        # jointPos[3] = jointPos[3] + (newDigits - jointPos[3])*probabilities[2]
        # jointPos[4] = jointPos[4] + (newRot - jointPos[4])*probabilities[3]
        # jointPos[5] = jointPos[5] + (newFlex - jointPos[5])*probabilities[4]

        jointPos[0] = newThumb if moveBool[0] else jointPos[0]
        jointPos[1] = 25 #newThumb if moveBool[0] else jointPos[1]
        jointPos[2] = newDigits if moveBool[2] else jointPos[3] # newIndex if moveBool[1] else jointPos[2]
        jointPos[3] = newDigits if moveBool[2] else jointPos[3]
        jointPos[4] = newRot if moveBool[3] else jointPos[4]
        jointPos[5] = newFlex if moveBool[4] else jointPos[5]

        # jointPos[0] = newThumb if predictedJoint == 0 else jointPos[0]
        # jointPos[1] = newThumb if predictedJoint == 0 else jointPos[0]
        # jointPos[2] = newDigits if predictedJoint == 0 else jointPos[3]
        # jointPos[3] = newDigits if predictedJoint == 0 else jointPos[3]
        # jointPos[4] = newRot if predictedJoint == 2 else jointPos[4]
        # jointPos[5] = newFlex if predictedJoint == 1 else jointPos[5]

        # convert commanded position back for model
        # digitsRad = jointPos[3]/90*(np.pi/2) - np.pi/4
        # flexRad = (jointPos[5] + 55)/110*(110*np.pi/180) - 55*np.pi/180
        # rotRad = (jointPos[4] + 175)/295*(295*np.pi/180) - 175*np.pi/180

        # digitsRad = digitsRad/1.5 if digitsRad > 0 else digitsRad/2
        # rotRad = rotRad if rotRad > 0 else rotRad/1.5

        # jointPos = self.rateLimit(jointPos)
        # self.hidden[0, [0, 2, 4]] = torch.tensor([[digitsRad, flexRad, rotRad]], dtype=torch.float, device=self.device, requires_grad=False)

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

        return newPosCom

    def PIDcontroller(self, posCom):
        curPos = np.asrray(self.LUKEArm.getCurPos()) # reference position

        error = posCom - curPos

        self.I = np.where(np.abs(self.I + error/self.LUKEArm.Hz) < self.windup, self.I + error/self.LUKEArm.Hz)
        derivError = (self.lastError - error)*self.LUKEArm.Hz
        self.lastError = error # store last error

        PIDCommand = self.k_p*error + self.k_i*self.I + self.k_d*derivError

        return PIDCommand + curPos

    @staticmethod
    def softmax(x):
        """ gets the probability of each option from a list of values """
        e_x = np.exp(x - np.max(x))
        return e_x/e_x.sum()
