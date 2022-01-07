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
        
        print(f"""\tMaxes:\n\t\t{norms[0]:07.2f} {norms[1]:07.2f} {norms[2]:07.2f} {norms[3]:07.2f}
            \t\t{norms[4]:07.2f} {norms[5]:07.2f} {norms[6]:07.2f} {norms[7]:07.2f}
            \t\t{norms[8]:07.2f} {norms[9]:07.2f} {norms[10]:07.2f} {norms[11]:07.2f}
            \t\t{norms[12]:07.2f} {norms[13]:07.2f} {norms[14]:07.2f} {norms[15]:07.2f}""")

        print(f"""\Mins:\n\t\t{norms[16]:07.2f} {norms[17]:07.2f} {norms[18]:07.2f} {norms[19]:07.2f}
            \t\t{norms[20]:07.2f} {norms[21]:07.2f} {norms[22]:07.2f} {norms[23]:07.2f}
            \t\t{norms[24]:07.2f} {norms[25]:07.2f} {norms[26]:07.2f} {norms[27]:07.2f}
            \t\t{norms[28]:07.2f} {norms[29]:07.2f} {norms[30]:07.2f} {norms[31]:07.2f}""")

    def printDeltas(self):
        print("EMG Deltas:")
        deltas = self.deltas
        
        print(f"""\tMaxes:\n\t\t{deltas[0]:07.2f} {deltas[1]:07.2f} {deltas[2]:07.2f} {deltas[3]:07.2f}\n\t\t{deltas[4]:07.2f} {deltas[5]:07.2f} {deltas[6]:07.2f} {deltas[7]:07.2f}""")

        print(f"""\tMins:\n\t\t{deltas[8]:07.2f} {deltas[9]:07.2f} {deltas[10]:07.2f} {deltas[11]:07.2f}\n\t\t{deltas[12]:07.2f} {deltas[13]:07.2f} {deltas[14]:07.2f} {deltas[15]:07.2f}""")


    def printRawEMG(self):
        print("Raw EMG:")
        raw = self.rawEMG

        print(f"""\t{raw[0]:07.2f} {raw[1]:07.2f} {raw[2]:07.2f} {raw[3]:07.2f}\n\t{raw[4]:07.2f} {raw[5]:07.2f} {raw[6]:07.2f} {raw[7]:07.2f}\n\t{raw[8]:07.2f} {raw[9]:07.2f} {raw[10]:07.2f} {raw[11]:07.2f}\n\t{raw[12]:07.2f} {raw[13]:07.2f} {raw[14]:07.2f} {raw[15]:07.2f}""")

    def printNormedEMG(self):
        print("Normed EMG:")
        emg = self.normedEMG

        print(f"""\t{emg[0]:07.2f} {emg[1]:07.2f} {emg[2]:07.2f} {emg[3]:07.2f}\n\t{emg[4]:07.2f} {emg[5]:07.2f} {emg[6]:07.2f} {emg[7]:07.2f}\n\t{emg[8]:07.2f} {emg[9]:07.2f} {emg[10]:07.2f} {emg[11]:07.2f}\n\t{emg[12]:07.2f} {emg[13]:07.2f} {emg[14]:07.2f} {emg[15]:07.2f}""")

    def printMuscleAct(self):
        print("Muscle Activation:")
        mAct = self.muscleAct

        print(f"""\t{mAct[0]:07.2f} {mAct[1]:07.2f} {mAct[2]:07.2f} {mAct[3]:07.2f}\n\t{mAct[4]:07.2f} {mAct[5]:07.2f} {mAct[6]:07.2f} {mAct[7]:07.2f}\n\t{mAct[8]:07.2f} {mAct[9]:07.2f} {mAct[10]:07.2f} {mAct[11]:07.2f}\n\t{mAct[12]:07.2f} {mAct[13]:07.2f} {mAct[14]:07.2f} {mAct[15]:07.2f}""")

    def getBounds(self):
        try:
            with open(self.boundsPath, 'rb') as fifo:
                normsPack = fifo.read()
    
            norms = struct.unpack("ffffffffffffffffffffffffffffffff", normsPack)
            self.bounds = list(norms)

        except OSError as e:
            print(f"getBounds(): Could not read bounds - {e}")

    def getDeltas(self):
        try:
            with open(self.deltasPath, 'rb') as fifo:
                deltasPack = fifo.read()
    
            deltas = struct.unpack("ffffffffffffffff", deltasPack)
            self.deltas = list(deltas)

        except OSError as e:
            print(f"getDeltas(): Could not read deltas - {e}")

    def readEMG(self):
        try:
            with open(self.emgPath, 'rb') as fifo:
                emgPack = fifo.read()

            emg = struct.unpack("ffffffffffffffffffIIIIf", emgPack)
            self.OS_time = emg[0]
            self.OS_tick = emg[1]
            self.rawEMG = emg[2:18]
            self.trigger = emg[18]
            self.switch1 = emg[19]
            self.switch2 = emg[20]
            self.end = emg[21]
            self.samplingFreq = emg[22]

        except OSError as e:
            print(f"readEMG(): Could not read EMG - {e}")

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

            self.muscleAct[i] = abs((u/tauA + prevA/Ts)/(1/Ts + (b + (1 - b)*u)/tauA))