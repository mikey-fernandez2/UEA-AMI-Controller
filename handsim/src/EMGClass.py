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

        self.rawEMG = [-1]*self.numElectrodes
        self.normedEMG = [-1]*self.numElectrodes # array of normalized EMG values
        self.muscleAct = [-1]*self.numElectrodes # array of muscle activation (through low pass muscle activation dynamics)
        self.prevAct = [-1]*self.numElectrodes # array of previous muscle activation values

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

    def printDeltas(self):
        print("EMG Deltas:")
        deltas = self.deltas
        
        print("\tMaxes:\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f" % 
            (deltas[0], deltas[1], deltas[2], deltas[3],
            deltas[4], deltas[5], deltas[6], deltas[7]))

        print("\tMins:\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f" % 
            (deltas[8], deltas[9], deltas[10], deltas[11],
            deltas[12], deltas[13], deltas[14], deltas[15]))

    def printRawEMG(self):
        print("Raw EMG:")
        raw = self.rawEMG

        print("\t%07.2f %07.2f %07.2f %07.2f\n\t%07.2f %07.2f %07.2f %07.2f\n\t%07.2f %07.2f %07.2f %07.2f\n\t%07.2f %07.2f %07.2f %07.2f" % 
            (raw[0], raw[4], raw[8],  raw[12],
             raw[1], raw[5], raw[9],  raw[13],
             raw[2], raw[6], raw[10], raw[14],
             raw[3], raw[7], raw[11], raw[15]))

    def getBounds(self):
        try:
            with open(self.boundsPath, 'rb') as fifo:
                normsPack = fifo.read()
    
            norms = struct.unpack("ffffffffffffffffffffffffffffffff", normsPack)
            self.bounds = list(norms)

        except OSError as e:
            print("getBounds(): Could not read bounds - %s" % e)

    def getDeltas(self):
        try:
            with open(self.deltasPath, 'rb') as fifo:
                deltasPack = fifo.read()
    
            deltas = struct.unpack("ffffffffffffffff", deltasPack)
            self.deltas = list(deltas)

        except OSError as e:
            print("getDeltas(): Could not read deltas - %s" % e)

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
            print("readEMG(): Could not read EMG - %s" % e)

    def normEMG(self):
        for i in range(self.numElectrodes):
            maxVal = self.bounds[i]
            minVal = self.bounds[self.numElectrodes + i]

            normed = (self.rawEMG[i] - minVal)/maxVal

            if normed < 0: normed = 0
            elif normed > 1: normed = 1

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