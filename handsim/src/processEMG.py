# Mikey Fernandez, 04/22/2021
#
# processEMG.py
# Perform EMG processing steps. 3 options, elaborated below:
# 
# "norm"
#    Collect EMG from Wifi Board and save the maximum and minimum values from each electrode.
#    Saved into a txt file that will be read by the controller to normalize EMG signals.
#
# "manual"
#    Manually input EMG normalization factors for each electrode. They will be saved into a txt file
#    that will be read by the controller to normalize EMG signals.
#
# "delta"
#    Collect EMG from Wifi Board, normalize the incoming EMG and calculate the difference between each pair
#    Save the maximum and minimum of these differences into a txt file that will be read by the controller
#
# "print"
#    Print the EMG normalization factors and the EMG deltas

import os, sys, time, struct, zmq, math
import numpy as np
from EMGClass import EMG

class EMGProcessing(object):
    def __init__(self, socketAddr, scaleFactorsPath, deltasPath, numElectrodes):
        self.sfPath = scaleFactorsPath
        self.dPath = deltasPath

        self.emg = EMG(socketAddr, numElectrodes, tauA=0.05, tauD=0.1)
        self.emg.readEMG()
        self.emg.initFilters()

        self.allReadings = np.empty((numElectrodes, 1))

        self.numElectrodes = numElectrodes
        self.numPairs = numElectrodes//2

        self.makeMaxMin()

    def makeMaxMin(self):
        # EMG normalization
        try:
            # If EMG norms exist, use them to initialize arrays
            with open(self.sfPath, 'rb') as fifo:
                normsPack = fifo.read()

            norms = struct.unpack("ffffffffffffffffffffffffffffffff", normsPack)
            self.maxes = norms[:16]
            self.mins = norms[16:]

        except:
            # Otherwise, initialize to 0 (maxes) and maximum value for system (mins)
            self.maxes = [-math.inf]*self.numElectrodes
            self.mins = [sys.maxsize]*self.numElectrodes

        # EMG Deltas
        try:
            # If EMG deltas exist, use them to initialize arrays
            with open(self.dPath, 'rb') as fifo:
                deltasPack = fifo.read()

            deltas = struct.unpack("ffffffffffffffff", deltasPack)
            self.maxDelta = deltas[:8]
            self.minDelta = deltas[8:]

        except:
            # Otherwise, initialize to 0 (maxes) and maximum value for system (mins)
            self.maxDelta = [-math.inf]*self.numPairs
            self.minDelta = [sys.maxsize]*self.numPairs

        self.deltas = [0]*self.numPairs

    def printNorms(self, norms):
        print("EMG Bounds:")    
        print(f"""\tMaxes:\n\t\t{norms[0]:07.2f} {norms[1]:07.2f} {norms[2]:07.2f} {norms[3]:07.2f}\n\t\t{norms[4]:07.2f} {norms[5]:07.2f} {norms[6]:07.2f} {norms[7]:07.2f}\n\t\t{norms[8]:07.2f} {norms[9]:07.2f} {norms[10]:07.2f} {norms[11]:07.2f}\n\t\t{norms[12]:07.2f} {norms[13]:07.2f} {norms[14]:07.2f} {norms[15]:07.2f}""")

        print(f"""\tMins:\n\t\t{norms[16]:07.2f} {norms[17]:07.2f} {norms[18]:07.2f} {norms[19]:07.2f}\n\t\t{norms[20]:07.2f} {norms[21]:07.2f} {norms[22]:07.2f} {norms[23]:07.2f}\n\t\t{norms[24]:07.2f} {norms[25]:07.2f} {norms[26]:07.2f} {norms[27]:07.2f}\n\t\t{norms[28]:07.2f} {norms[29]:07.2f} {norms[30]:07.2f} {norms[31]:07.2f}""")

    def printDeltas(self, deltas):
        print("EMG Deltas:")       
        print(f"""\tMaxes:\n\t\t{deltas[0]:07.2f} {deltas[1]:07.2f} {deltas[2]:07.2f} {deltas[3]:07.2f}\n\t\t{deltas[4]:07.2f} {deltas[5]:07.2f} {deltas[6]:07.2f} {deltas[7]:07.2f}""")
        print(f"""\tMins:\n\t\t{deltas[8]:07.2f} {deltas[9]:07.2f} {deltas[10]:07.2f} {deltas[11]:07.2f}\n\t\t{deltas[12]:07.2f} {deltas[13]:07.2f} {deltas[14]:07.2f} {deltas[15]:07.2f}""")
    
    # def compareNorms(self, iemg):
    #     if not iemg:
    #         for electrode in range(self.numElectrodes):
    #             # Compare maxes
    #             if self.emg.rawEMG[electrode] > self.maxes[electrode]: self.maxes[electrode] = self.emg.rawEMG[electrode]
    #             # Compare mins
    #             if self.emg.rawEMG[electrode] < self.mins[electrode]: self.mins[electrode] = self.emg.rawEMG[electrode]

    #     else:
    #         for electrode in range(self.numElectrodes):
    #             # Compare maxes
    #             if self.emg.iEMG[electrode] > self.maxes[electrode]: self.maxes[electrode] = self.emg.iEMG[electrode]
    #             # Compare mins
    #             if self.emg.iEMG[electrode] < self.mins[electrode]: self.mins[electrode] = self.emg.iEMG[electrode]

    def compareDeltas(self):
        for pair in range(self.numPairs):
            # Compare maxes
            if self.deltas[pair] > self.maxDelta[pair]: self.maxDelta[pair] = self.deltas[pair]

            # Compare mins
            if self.deltas[pair] < self.minDelta[pair]: self.minDelta[pair] = self.deltas[pair]

    def receiveEMG(self):
        self.emg.sock.recv()

        # self.emg.readEMG()
        # self.emg.intEMG()
        self.emg.readEMGPacket()
        self.emg.intEMGPacket()
        self.emg.normEMG()
        self.emg.muscleDynamics()

    def stream(self):
        try:
            while True:
                self.receiveEMG()
                useiEMG = True

                if useiEMG:
                    # self.emg.printiEMG()
                    self.emg.printNormedEMG()
                else:
                    self.emg.printRawEMG()

        except KeyboardInterrupt:
            print(f"\nStreaming complete.")

    def writeNorms(self):
        normsBytes = struct.pack("ffffffffffffffffffffffffffffffff", self.maxes[0], self.maxes[1], self.maxes[2], self.maxes[3], self.maxes[4], self.maxes[5], self.maxes[6], self.maxes[7],
            self.maxes[8], self.maxes[9], self.maxes[10], self.maxes[11], self.maxes[12], self.maxes[13], self.maxes[14], self.maxes[15],
            self.mins[0], self.mins[1], self.mins[2], self.mins[3], self.mins[4], self.mins[5], self.mins[6], self.mins[7],
            self.mins[8], self.mins[9], self.mins[10], self.mins[11], self.mins[12], self.mins[13], self.mins[14], self.mins[15])

        with open(self.sfPath, 'wb') as output:
            output.write(normsBytes)

    def writeDeltas(self):
        deltasBytes = struct.pack("ffffffffffffffff", self.maxDelta[0], self.maxDelta[1], self.maxDelta[2], self.maxDelta[3], self.maxDelta[4], self.maxDelta[5], self.maxDelta[6], self.maxDelta[7], 
            self.minDelta[0], self.minDelta[1], self.minDelta[2], self.minDelta[3], self.minDelta[4], self.minDelta[5], self.minDelta[6], self.minDelta[7])

        with open(self.dPath, 'wb') as output:
            output.write(deltasBytes)

    def emgExtrema(self, iemg):
        try:
            print("Recording EMG normalization factors...\n")
            while True:
                self.receiveEMG()

                if not iemg:
                    self.emg.printRawEMG()
                    self.allReadings = np.append(self.allReadings, np.reshape(self.emg.rawEMG, (-1, 1)), axis=1)
                else:
                    # pass
                    self.emg.printiEMG()
                    self.allReadings = np.append(self.allReadings, np.reshape(self.emg.iEMG, (-1, 1)), axis=1)

                time.sleep(1/self.emg.samplingFreq)

        except KeyboardInterrupt:
            print(f"\nReceiving complete.\nWriting normalization factors to {self.sfPath}")

            # use 98th percentile for the maxes and 1st percentile for the mins
            self.maxes = np.percentile(self.allReadings, 98, axis=1)
            self.mins = np.percentile(self.allReadings, 1, axis=1)

            self.writeNorms()

    def emgManual(self):
        try:
            print("Enter 16 maximum electrode readings for normalization.\n")
            while True:
                maxesRaw = input() # Take input

                if maxesRaw == "exit": # Escape valve
                    return
                
                try: # Turn input into array of floats
                    maxes = [float(x) for x in maxesRaw.split()]
                except: # Uh-oh! Formatting wrong
                    maxes = []

                if len(maxes) == self.numElectrodes: # Exit if 16 floats received
                    break
                else: # Otherwise ask for the input again
                    print("Input formatted incorrectly. Enter 16 floats.\n")

            print("\nEnter 16 minimum electrode readings for normalization.\n")
            while True:
                minsRaw = input() # Take input

                if minsRaw == "exit": # Escape value
                    return

                try: # Turn into array of floats
                    mins = [float(x) for x in minsRaw.split()]
                except: # Uh-oh! Formatting wrong
                    mins = []

                if len(mins) == self.numElectrodes: # Exit if 16 floats received
                    break
                else: # Otherwise ask for the input again
                    print("Input formatted incorrectly. Enter 16 floats.\n")

        except KeyboardInterrupt:
            print("\nStopping manual input.")
            return

        # Save the input arrays
        self.maxes = maxes
        self.mins = mins
        print(f"\nInput received.\nWriting normalization factors to {self.sfPath}")
        self.writeNorms()

    def deltaExtrema(self):
        try:
            print("Recording activation extrema...\n")

            self.maxDelta = [-math.inf]*(self.numPairs)
            self.minDelta = [sys.maxsize]*(self.numPairs)

            while True:
                self.receiveEMG()

                for i in range(self.numPairs):
                    # self.deltas[i] = self.emg.normedEMG[2*i] - self.emg.normedEMG[2*i + 1]
                    self.deltas[i] = self.emg.iEMG[2*i] - self.emg.iEMG[2*i + 1]

                self.compareDeltas()

                time.sleep(1/self.emg.samplingFreq)

        except KeyboardInterrupt:
            print(f"\nReceiving complete.\nWriting activation extrema to {self.dPath}")
            self.writeDeltas()

    def printVals(self):
        try:
            with open(self.sfPath, 'rb') as fifo:
                normsPack = fifo.read()

            norms = struct.unpack("ffffffffffffffffffffffffffffffff", normsPack)
            self.printNorms(norms)

        except OSError as e:
            print(f"Could not open scale factors file - {e}")

        print()
     
        try:
            with open(self.dPath, 'rb') as fifo:
                deltasPack = fifo.read()

            deltas = struct.unpack("ffffffffffffffff", deltasPack)
            self.printDeltas(deltas)

        except OSError as e:
            print(f"Could not open EMG deltas file - {e}")

def callback():
    run = ""
    while run not in ["norm", "iemg", "delta", "print", "manual", "stream", "exit"]:
        run = input("Enter 'norm' to find raw EMG normalization factors.\nEnter 'iemg' to find normalization factors with iEMG.\nEnter 'manual' to enter EMG normalization factors.\nEnter 'delta' to find max/min EMG deltas.\nEnter 'print' to print EMG bounds.\nEnter 'stream' to print EMG without recording.\nEnter 'exit' to quit:\n")

    return run

if __name__ == "__main__":

    socketAddr = "tcp://127.0.0.1:1235"
    scaleFactorsPath = "/home/haptix/haptix/haptix_controller/handsim/include/scaleFactors.txt"
    deltasPath = "/home/haptix/haptix/haptix_controller/handsim/include/deltas.txt"
    numElectrodes = 16

    emg = EMGProcessing(socketAddr, scaleFactorsPath, deltasPath, numElectrodes)

    try:
        while True:
            run = callback()

            if run == "norm":
                print("\n\nFinding EMG norming factors")
                emg.emgExtrema(iemg=False)
                print("\n\n")

            if run == "iemg":
                print("\n\nFinding iEMG norming factors")
                emg.emgExtrema(iemg=True)
                print("\n\n")

            elif run == "delta":
                print("\n\nFinding activation differences")
                emg.deltaExtrema()
                print("\n\n")

            elif run == "print":
                print("\n\nPrinting EMG bounds")
                emg.printVals()
                print("\n\n")

            elif run == "manual":
                print("\n\nManually setting EMG normalization factors")
                emg.emgManual()
                print("\n\n")

            elif run == "stream":
                print("\n\nStreaming EMG...")
                emg.stream()
                print("\n\n")
          
            elif run == "exit":
                print("\n\nExiting.")
                break

            else:
                print("Invalid command.")

    except KeyboardInterrupt:
        print("\n\nExiting.")
