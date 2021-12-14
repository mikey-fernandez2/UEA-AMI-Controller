# Mikey Fernandez 12/13/2021
#
# impedanceController.py
# Make the impedance controller class

class impedanceController:
    def __init__(self, numMotors=8, freq_n=3, numElectrodes=16, LUKEArm=None, emg=None):
        self.LUKEArm = LUKEArm
        self.emg = emg
        self.numMotors = numMotors
        self.freq_n = freq_n
        self.numElectrodes = numElectrodes

        self.prevT = [0]*numMotors
        self.prev2T = [0]*numMotors

        # [thumbP, thumbY, index, mrp, wristRot, wristFlex, humRot, elbow]
        self.motorElectrodeMap = [[0, 0], [0, 0], [1, 2], [3, 4], [0, 0], [0, 0], [0, 0], [0, 0]]
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

            # TODO: figure out how to get the command in [0, 1] to be scaled better
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