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

import os, sys, time, struct
   
def printNorms(norms):
    print("EMG Normalization Factors:")
    
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

def printDeltas(deltas):
    print("EMG Deltas:")
    
    print("\tMaxes:\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f" % 
        (deltas[0], deltas[1], deltas[2], deltas[3],
        deltas[4], deltas[5], deltas[6], deltas[7]))

    print("\tMins:\n\t\t%07.2f %07.2f %07.2f %07.2f\n\t\t%07.2f %07.2f %07.2f %07.2f" % 
        (deltas[8], deltas[9], deltas[10], deltas[11],
        deltas[12], deltas[13], deltas[14], deltas[15]))

class EMGProcessing(object):
    def __init__(self, path, scaleFactorsPath, deltasPath, numElectrodes, sendingFreq):
        self.path = path
        self.sfPath = scaleFactorsPath
        self.dPath = deltasPath
        self.sendingFreq = sendingFreq

        self.numElectrodes = numElectrodes
        self.numPairs = numElectrodes//2

        self.msgLen = struct.calcsize("ffffffffffffffffffIIIIf")

        self.rawData = []
        self.rawEMG = []

        self.makeMaxMin()

    def makeMaxMin(self):
        # EMG normalization
        try:
            # If EMG norms exist, use them to initialize arrays
            with open(self.sfPath, 'rb') as fifo:
                normsPack = fifo.read()

            norms = struct.unpack("ffffffffffffffffffffffffffffffff", normsPack)
            self.maxes = norms[0:16]
            self.mins = norms[16:len(norms)]

        except:
            # Otherwise, initialize to 0 (maxes) and maximum value for system (mins)
            self.maxes = [0]*self.numElectrodes
            self.mins = [sys.maxsize]*self.numElectrodes

        # EMG Deltas
        try:
            # If EMG deltas exist, use them to initialize arrays
            with open(self.dPath, 'rb') as fifo:
                deltasPack = fifo.read()

            deltas = struct.unpack("ffffffffffffffff", deltasPack)
            self.maxDelta = deltas[0:8]
            self.minDelta = deltas[8:len(deltas)]

        except:
            # Otherwise, initialize to 0 (maxes) and maximum value for system (mins)
            self.maxDelta = [0]*self.numPairs
            self.minDelta = [sys.maxsize]*self.numPairs

        self.normedEMG = [0]*self.numElectrodes
        self.deltas = [0]*self.numPairs

    def compareNorms(self):
        for electrode in range(self.numElectrodes):
            # Compare maxes
            if self.rawEMG[electrode] > self.maxes[electrode]:
                self.maxes[electrode] = self.rawEMG[electrode]
        
            # Compare mins
            if self.rawEMG[electrode] < self.mins[electrode]:
                self.mins[electrode] = self.rawEMG[electrode]

    def compareDeltas(self):
        for pair in range(self.numPairs):
            # Compare maxes
            if self.deltas[pair] > self.maxDelta[pair]:
                self.maxDelta[pair] = self.deltas[pair]
            
            # Compare mins
            if self.deltas[pair] < self.minDelta[pair]:
                self.minDelta[pair] = self.deltas[pair]

    def receiveEMG(self):
        recLen = 0
        while recLen == 0:
            with open(self.path, 'rb') as fifo:
                EMGData = fifo.read()
            recLen = len(EMGData)

        self.rawData = struct.unpack("ffffffffffffffffffIIIIf", EMGData)
        self.rawEMG = self.rawData[2:2 + self.numElectrodes] # extract the EMG data

    def normEMG(self):
        for i in range(self.numElectrodes):
            normed = (self.rawEMG[i] - self.mins[i])/self.maxes[i]

            # Implement bounds on normalized EMG
            if normed < 0: normed = 0
            elif normed > 1: normed = 1

            self.normedEMG[i] = normed

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

    def emgExtrema(self):
        try:
            print("Recording EMG normalization factors...\n")

            self.maxes = [0]*self.numElectrodes
            self.mins = [sys.maxsize]*self.numElectrodes

            while(1):
                self.receiveEMG()
                self.compareNorms()

                time.sleep(1/self.sendingFreq)

        except KeyboardInterrupt:
            print("\nReceiving complete.\nWriting normalization factors to %s\n" % self.sfPath)
            self.writeNorms()

    def emgManual(self):
        try:
            print("Enter 16 maximum electrode readings for normalization.\n")
            while (True):
                maxesRaw = input() # Take input

                if (maxesRaw == "exit"): # Escape valve
                    return
                
                try: # Turn input into array of floats
                    maxes = [float(x) for x in maxesRaw.split()]
                except: # Uh-oh! Formatting wrong
                    maxes = []

                if (len(maxes) == self.numElectrodes): # Exit if 16 floats received
                    break
                else: # Otherwise ask for the input again
                    print("Input formatted incorrectly. Enter 16 floats.\n")

            print("\nEnter 16 minimum electrode readings for normalization.\n")
            while (True):
                minsRaw = input() # Take input

                if (minsRaw == "exit"): # Escape value
                    return
               
                try: # Turn into array of floats
                    mins = [float(x) for x in minsRaw.split()]
                except: # Uh-oh! Formatting wrong
                    mins = []

                if (len(mins) == self.numElectrodes): # Exit if 16 floats received
                    break
                else: # Otherwise ask for the input again
                    print("Input formatted incorrectly. Enter 16 floats.\n")
        
        except KeyboardInterrupt:
            print("\nStopping manual input.")
            return

        # Save the input arrays
        self.maxes = maxes
        self.mins = mins
        print("\nInput received.\nWriting normalization factors to %s\n" % self.sfPath)
        self.writeNorms()

    def deltaExtrema(self):
        try:
            print("Recording activation extrema...\n")

            self.maxDelta = [0]*(self.numPairs)
            self.minDelta = [sys.maxsize]*(self.numPairs)

            while(1):
                self.receiveEMG()
                self.normEMG()

                for i in range(self.numPairs):
                    self.deltas[i] = self.normedEMG[2*i] - self.normedEMG[2*i + 1]

                self.compareDeltas()
                
                time.sleep(1/self.sendingFreq)

        except KeyboardInterrupt:
            print("\nReceiving complete.\nWriting activation extrema to %s\n" % self.dPath)
            self.writeDeltas()

    def printEMG(self):
        try:
            with open(self.sfPath, 'rb') as fifo:
                normsPack = fifo.read()

            norms = struct.unpack("ffffffffffffffffffffffffffffffff", normsPack)
            printNorms(norms)

        except OSError as e:
            print("Could not open scale factors file - %s" % e)

        try:
            with open(self.dPath, 'rb') as fifo:
                deltasPack = fifo.read()

            deltas = struct.unpack("ffffffffffffffff", deltasPack)
            printDeltas(deltas)

        except OSError as e:
            print("Could not open EMG deltas file - %s" % e)

def callback():
    run = ""
    while run not in ["norm", "delta", "print", "manual", "exit"]:
        run = input("Enter 'norm' to find EMG normalization factors.\nEnter 'manual' to enter EMG normalization factors.\nEnter 'delta' to find max/min EMG deltas.\nEnter 'print' to print EMG bounds.\nEnter 'exit' to quit:\n")

    return run

if __name__ == "__main__":

    path = "/tmp/emg"
    scaleFactorsPath = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/scaleFactors.txt"
    deltasPath = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/deltas.txt"
    sendingFreq = 20
    numElectrodes = 16

    emg = EMGProcessing(path, scaleFactorsPath, deltasPath, numElectrodes, sendingFreq)

    try:
        while(1):
            run = callback()

            if run == "norm":
                print("\n\nFinding EMG norming factors")
                emg.emgExtrema()
                print("\n\n")

            elif run == "delta":
                print("\n\nFinding activation differences")
                emg.deltaExtrema()
                print("\n\n")

            elif run == "print":
                print("\n\nPrinting EMG bounds")
                emg.printEMG()
                print("\n\n")

            elif run == "manual":
                print("\n\nManually setting EMG normalization factors")
                emg.emgManual()
                print("\n\n")
            
            elif run == "exit":
                print("\n\nExiting.")
                break

            else:
                print("Invalid command.")

    except KeyboardInterrupt:
        print("\n\nExiting.")