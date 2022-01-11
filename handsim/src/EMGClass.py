# Mikey Fernandez, 11/09/2021
#
# EMGClass.py
# Define the EMG class for use with the LUKE arm

import struct, os, sys, zmq, math
import numpy as np
from scipy import signal

class EMG:
    def __init__(self, numElectrodes=16, tauA=0.05, tauD=0.1):
        self.numElectrodes = numElectrodes
        self.tauA = tauA
        self.tauD = tauD

        self.boundsPath = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/scaleFactors.txt"
        self.deltasPath = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/deltas.txt"

        self.socketAddr = "tcp://127.0.0.1:1235"
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect(self.socketAddr)
        self.sock.subscribe("")

        # this is how the EMG package gets broken up
        self.OS_time = None
        self.OS_tick = None
        self.trigger = None
        self.switch1 = None
        self.switch2 = None
        self.end = None
        self.samplingFreq = None # this SHOULD be 1 kHz - but don't assume that

        self.getBounds() # first 16: maximum values, second 16: minimum values
        self.getDeltas() # first 8: maximum deltas, second 8: minimum deltas

        self.rawEMG = [-1]*self.numElectrodes # this is RAW from the board - need to get iEMG
        self.iEMG = [-1]*self.numElectrodes # this is iEMG
        self.normedEMG = [-1]*self.numElectrodes # array of normalized EMG values
        self.muscleAct = [-1]*self.numElectrodes # array of muscle activation (through low pass muscle activation dynamics)
        self.prevAct = [-1]*self.numElectrodes # array of previous muscle activation values

    def __del__(self):
        try:
            # close the socket
            self.sock.close()
            self.ctx.term()
        except:
            print("__del__: Socket closing error")

    def initFilters(self):
        # parameters for calculating iEMG
        self.int_window = .05 # sec - 50 ms integration window
        self.samples_window = math.ceil(self.int_window*self.samplingFreq)
        self.rawHistory = np.zeros((self.numElectrodes, self.samples_window))
        self.powerLineFilter = signal.butter(4, [58, 62], btype='bandstop', output='sos', fs=self.samplingFreq)
        self.highpassFilter = signal.butter(4, 20, btype='highpass', output='sos', fs=self.samplingFreq)
        self.lowpassFilter = signal.butter(4, 10, btype='lowpass', output='sos', fs=self.samplingFreq)

    ##########################################################################
    # print functions
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
        print(f"Raw EMG: {self.OS_time}")
        raw = self.rawEMG

        print(f"""\t{raw[0]:07.2f} {raw[1]:07.2f} {raw[2]:07.2f} {raw[3]:07.2f}\n\t{raw[4]:07.2f} {raw[5]:07.2f} {raw[6]:07.2f} {raw[7]:07.2f}\n\t{raw[8]:07.2f} {raw[9]:07.2f} {raw[10]:07.2f} {raw[11]:07.2f}\n\t{raw[12]:07.2f} {raw[13]:07.2f} {raw[14]:07.2f} {raw[15]:07.2f}\n""")

    def printiEMG(self):
        print(f"iEMG: {self.OS_time}")
        iEMG = self.iEMG

        print(f"""\t{iEMG[0]:07.2f} {iEMG[1]:07.2f} {iEMG[2]:07.2f} {iEMG[3]:07.2f}\n\t{iEMG[4]:07.2f} {iEMG[5]:07.2f} {iEMG[6]:07.2f} {iEMG[7]:07.2f}\n\t{iEMG[8]:07.2f} {iEMG[9]:07.2f} {iEMG[10]:07.2f} {iEMG[11]:07.2f}\n\t{iEMG[12]:07.2f} {iEMG[13]:07.2f} {iEMG[14]:07.2f} {iEMG[15]:07.2f}\n""")

    def printNormedEMG(self):
        print(f"Normed EMG: {self.OS_time}")
        emg = self.normedEMG

        print(f"""\t{emg[0]:07.2f} {emg[1]:07.2f} {emg[2]:07.2f} {emg[3]:07.2f}\n\t{emg[4]:07.2f} {emg[5]:07.2f} {emg[6]:07.2f} {emg[7]:07.2f}\n\t{emg[8]:07.2f} {emg[9]:07.2f} {emg[10]:07.2f} {emg[11]:07.2f}\n\t{emg[12]:07.2f} {emg[13]:07.2f} {emg[14]:07.2f} {emg[15]:07.2f}\n""")

    def printMuscleAct(self):
        print(f"Muscle Activation: {self.OS_time}")
        mAct = self.muscleAct

        print(f"""\t{mAct[0]:07.2f} {mAct[1]:07.2f} {mAct[2]:07.2f} {mAct[3]:07.2f}\n\t{mAct[4]:07.2f} {mAct[5]:07.2f} {mAct[6]:07.2f} {mAct[7]:07.2f}\n\t{mAct[8]:07.2f} {mAct[9]:07.2f} {mAct[10]:07.2f} {mAct[11]:07.2f}\n\t{mAct[12]:07.2f} {mAct[13]:07.2f} {mAct[14]:07.2f} {mAct[15]:07.2f}\n""")

    ##########################################################################
    # get fields
    def getRawEMG(self, electrode):
        return self.rawEMG[electrode]

    def getiEMG(self, electrode):
        return self.iEMG[electrode] 

    def getNormedEMG(self, electrode):
        return self.normedEMG[electrode]

    def getFilteredEMG(self, electrode):
        return self.muscleAct[electrode]

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

    ##########################################################################
    # actual calculations
    def readEMG(self):
        try:
            emgPack = self.sock.recv()

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

    ## The below are all done using numpy, make sure you understand what they do
    # calculate integrated EMG
    def intEMG(self):
        # shift the time history and add the latest raw EMG signal
        emg = np.hstack((self.rawHistory[:, 1:], np.reshape(self.rawEMG, (-1, 1))))
        self.rawHistory = emg

        emg = signal.sosfilt(self.powerLineFilter, emg, axis=1) # remove the electrical noise
        emg = signal.sosfilt(self.highpassFilter, emg, axis=1) # remove the drift and motion artifacts
        emg = abs(emg)
        emg = signal.sosfilt(self.lowpassFilter, emg, axis=1) # low pass filter to smooth signal
        
        self.iEMG = np.trapz(emg, axis=1)/self.samples_window # trapezoidal numerical integration

    # normalize the EMG
    def normEMG(self):
        maxVals = np.asarray(self.bounds[:self.numElectrodes])
        minVals = np.asarray(self.bounds[self.numElectrodes:])

        normed = (self.iEMG - minVals)/(maxVals - minVals)

        # correct the bounds
        normed[normed < 0] = 0
        normed[normed > 1] = 1

        self.normedEMG = normed

    # first order muscle activation dynamics
    def muscleDynamics(self):
        tauA = self.tauA
        tauD = self.tauD
        b = tauA/tauD
        Ts = 1/self.samplingFreq

        u = np.asarray(self.normedEMG)
        self.prevAct = self.muscleAct
        prevA = np.asarray(self.prevAct)

        self.muscleAct = abs((u/tauA + prevA/Ts)/(1/Ts + (b + (1 - b)*u)/tauA))

    # full EMG update pipeline
    def pipelineEMG(self):
        self.readEMG()
        self.intEMG()
        self.normEMG()
        self.muscleDynamics()