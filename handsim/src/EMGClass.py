# Mikey Fernandez, 11/09/2021
#
# EMGClass.py
# Define the EMG class for use with the LUKE arm

import struct, os, sys

class EMG:
    def __init__(self, numElectrodes=16, tauA=0.05, tauD=0.1):
        self.numElectrodes = numElectrodes
        self.tauA = tauA
        self.tauD = tauD

        self.emgPath = "/tmp/emg"
        self.boundsPath = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/scaleFactors.txt"
        self.deltasPath = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/deltas.txt"

        # this is how the EMG package gets broken up
        self.OS_time = None
        self.OS_tick = None
        self.trigger = None
        self.switch1 = None
        self.switch2 = None
        self.end = None
        self.samplingFreq = None

        self.getBounds() # first 16: maximum values, second 16: minimum values
        self.getDeltas() # first 8: maximum deltas, second 8: minimum deltas

        self.rawEMG = [None]*self.numElectrodes
        self.normedEMG = [None]*self.numElectrodes # array of normalized EMG values
        self.muscleAct = [None]*self.numElectrodes # array of muscle activation (through low pass muscle activation dynamics)
        self.prevAct = [None]*self.numElectrodes # array of previous muscle activation values

        self.motorMap = [[0, 1],                              # elbow
                        [2, 3], [4, 5], [6, 7],               # wristx, wristy, wristz
                        [8, 9], [14, 15], [14, 15], [14, 15], # thumb0, thumb1, thumb2, thumb3
                        [10, 11], [10, 11],                   # index0, index1
                        [12, 13],                             # middle1
                        [12, 13],                             # ring1
                        [12, 13], [12, 13]]                   # pinky0, pinky1

        self.gainMatrix = [[1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor0:  elbow 
                           [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor1:  wrist_y 
                           [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor2:  wrist_x
                           [-0.0181, 0.6033, 0.3266, 0.5346, -1.0191, -0.2806, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor3:  wrist_z
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor4:  thumb_0
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor5:  thumb_1
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor6:  thumb_2
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor7:  thumb_3
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor8:  index_0
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor9:  index_1
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor10: middle_1
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor11: ring_1
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # motor12: pinky_0
                           [-3.8913, 0.4458, 0.3357, -0.2121, 0.15244, 3.1729, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]  # motor13: pinky_1

    def getBounds(self):
        try:
            with open(self.boundsPath, 'rb') as fifo:
                normsPack = fifo.read()
    
            norms = struct.unpack("ffffffffffffffffffffffffffffffff", normsPack)
            self.bounds = list(norms)

        except OSError as e:
            print("Could not read bounds - %s" % e)

    def getDeltas(self):
        try:
            with open(self.deltasPath, 'rb') as fifo:
                deltasPack = fifo.read()
    
            deltas = struct.unpack("ffffffffffffffff", deltasPack)
            self.deltas = list(deltas)

        except OSError as e:
            print("Could not read deltas - %s" % e)

    def readEMG(self):
        try:
            with open(self.emgPath, 'rb') as fifo:
                emgPack = fifo.read()

            emg = struct.unpack("ffffffffffffffffffIIIIf", emgPack)
            self.rawEMG = emg[0:16]
            self.OS_time = emg[16]
            self.OS_tick = emg[17]
            self.trigger = emg[18]
            self.switch1 = emg[19]
            self.switch2 = emg[20]
            self.end = emg[21]
            self.samplingFreq = emg[22]

        except OSError as e:
            print("Could not read EMG - %s" % e)

    def normEMG(self):
        for i in range(self.numElectrodes):
            maxVal = self.bounds[i]
            minVal = self.bounds[self.numElectrodes + i]

            normed = (self.rawEMG[i] - minVal)/maxVal

            if normed < 0:
                normed = 0
            elif normed > 1:
                normed = 1

            self.normedEMG[i] = normed

    def getNormedEMG(self, electrode):
        return self.normedEMG[electrode]

    def getFilteredEMG(self, electrode):
        return self.muscleAct[electrode]

    def muscleDynamics(self):
        tauA = self.tauA
        tauD = self.tauD
        b = tauA/tauD
        Ts = 1/self.samplingFreq

        for i in range(self.numElectrodes):
            u = self.normedEMG[i]
            self.prevAct[i] = self.muscleAct[i]
            prevA = self.prevAct[i]

            self.muscleAct[i] = (u/tauA + prevA/Ts)/(1/Ts + (b + (1 - b)*u)/tauA)

    # get the electrodes corresponding to the agonist and antagonist muscles for given motor/joint
    def getElectrodes(self, motor):
        # motorMap[i][0] is the electrode corresponding to the agonist muscle for motor i
        # motorMap[i][1] is the electrode corresponding to the antagonist muscle for motor i
        agonist = self.motorMap[motor][0]
        antagonist = self.motorMap[motor][1]

        return (agonist, antagonist)

    def getGains(self, motor):
        gains = []
        for i in range(self.numElectrodes):
            gains.append(self.gainMatrix[motor][i])

        return gains

    def printNorms(self):
        print("EMG Bounds:")
        norms = self.bounds
        
        print("\tMaxes:\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f" % 
            (norms[0], norms[4], norms[8],  norms[12],
            norms[1], norms[5], norms[9],  norms[13],
            norms[2], norms[6], norms[10], norms[14],
            norms[3], norms[7], norms[11], norms[15]))

        print("\tMins:\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f" % 
            (norms[16], norms[20], norms[24], norms[28],
            norms[17], norms[21], norms[25], norms[29],
            norms[18], norms[22], norms[26], norms[30],
            norms[19], norms[23], norms[27], norms[31]))