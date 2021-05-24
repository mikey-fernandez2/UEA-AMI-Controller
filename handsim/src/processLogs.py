# processLogs.py
#
# Translates HAPTIX log files into arrays for plotting and analysis
# 
# Mikey Fernandez, 05/14/2021

import os, sys, string, matplotlib.pyplot as plt, re

# logParser class contains information about the logged data
class logParser:
    "This is a HAPTIX log file parsing class"
    def __init__(self, logFile, rawText):
        self.logFile = logFile # full file name
        self.rawText = rawText # raw input text
        self.numElectrodes = 16 # number of EMG sensors on SeongHo's board
        self.numPoses = 4 # number of Polhemus sensors (maximum)
        self.blockIdx = []
        self.lastSetting = 70 # number of last line in 'settings' section of log
        self.settings = []
        self.usingEMG = False
        self.usingPolhemus = False
        self.robot = {}
        self.emgSettings = {}
        self.rest = []
        self.blocks = []

        # initializing functions
        self.getSettings()
        self.getBlocks()
        self.setStorage()

    def getSettings(self):
        self.settings = self.rawText[0:self.lastSetting]

        if (self.settings[4].split()[-1] == "true"): self.usingEMG = True
        if (self.settings[5].split()[-1] == "true"): self.usingEMG = True

        robotBase = 8
        self.robot["motors"] = int(self.settings[robotBase].split()[-1])
        self.robot["joints"] = int(self.settings[robotBase + 1].split()[-1])
        self.robot["sensors"] = int(self.settings[robotBase + 2].split()[-1])
        self.robot["IMUs"] = int(self.settings[robotBase + 3].split()[-1])
        self.robot["updateRate"] = float(self.settings[robotBase + 4].split()[-2])
        self.robot["jointLimits"] = dict()

        motorsBase = robotBase + 7
        for i in range(self.robot["motors"]):
            minLim = float(self.settings[motorsBase + 3*i].split()[-2])
            maxLim = float(self.settings[motorsBase + 1 + 3*i].split()[-2])
            self.robot["jointLimits"]["motor" + str(i)] = [minLim, maxLim]

        emgBase = 15 + self.robot["motors"]*3
        self.emgSettings["Tau_A"] = float(self.settings[emgBase].split()[-2])
        self.emgSettings["Tau_D"] = float(self.settings[emgBase + 1].split()[-2])
        self.emgSettings["norms"] = dict()
        self.emgSettings["deltas"] = dict()
        normsMax = [float(norm) for norm in re.split(':|,', self.settings[emgBase + 5])[1:]]
        normsMin = [float(norm) for norm in re.split(':|,', self.settings[emgBase + 6])[1:]]
        for i in range(self.numElectrodes):
            self.emgSettings["norms"]["elec" + str(i)] = [normsMin[i], normsMax[i]]
        deltaMax = [float(norm) for norm in re.split(':|,', self.settings[emgBase + 8])[1:]]
        deltaMin = [float(norm) for norm in re.split(':|,', self.settings[emgBase + 9])[1:]]
        for i in range(self.numElectrodes//2):
            self.emgSettings["deltas"]["pair" + str(2*i) + "-" + str(2*i+1)] = [deltaMin[i], deltaMax[i]]

        self.emgSettings["omega_n"] = float(self.settings[-2].split()[-2])

    def getBlocks(self):
        self.rest = self.rawText[self.lastSetting:-4]

        for i in range(len(self.rest)):
            if ("--------------------------------------------------" in self.rest[i]):
                self.blockIdx.append(i)

                if (len(self.blockIdx) > 1):
                    self.blocks.append(self.rest[self.blockIdx[-2]:self.blockIdx[-1]])

            continue

    def setStorage(self):
        self.times = []
        self.motorPos = [[] for i in range(self.robot["motors"])]
        self.motorCom = [[] for i in range(self.robot["motors"])]
        self.jointPos = [[] for i in range(self.robot["joints"])]
        
        if (self.usingEMG):
            self.triggerSwitches = [[], [], []]
            self.rawEMG = [[] for i in range(self.numElectrodes)]
            self.normedEMG = [[] for i in range(self.numElectrodes)]
            self.muscleActivation = [[] for i in range(self.numElectrodes)]

        if (self.usingPolhemus):
            self.poses = [[[] for i in range(6)] for i in range(self.numPoses)]

    def transposeMatrix(self, matrix):
        transposed = []
        for col in range(len(matrix)):
            for row in range(len(matrix)):
                line = matrix[row].split()
                transposed.append(float(line[col]))

        return transposed

    def parseBlock(self, block):
        time = float(block[2][len("Running Time: "):-4])
        self.times.append(time)

        motorBase = 5
        for i in range(self.robot["motors"]):
            thisMotorRaw = block[motorBase + i]
            thisMotor = thisMotorRaw.split()
            self.motorPos[i].append(float(thisMotor[6]))
            self.motorCom[i].append(float(thisMotor[3]))

        jointBase = 20
        for i in range(self.robot["joints"]):
            thisJointRaw = block[jointBase + i]
            thisJoint = thisJointRaw.split()
            self.jointPos[i].append(float(thisJoint[3]))

        if (self.usingEMG):
            emgBase = 45
            self.EMGFreq = float(block[emgBase].split()[-1])

            thisTrigger = [int(block[emgBase + 1][-2]), int(block[emgBase + 2][-2]), int(block[emgBase + 3][-2])]

            rawEMG = self.transposeMatrix(block[emgBase + 5:emgBase + 9])
            normedEMG = self.transposeMatrix(block[emgBase + 10:emgBase + 14])
            muscleAct = self.transposeMatrix(block[emgBase + 15:emgBase + 19])

            for i in range(self.numElectrodes):
                self.rawEMG[i].append(rawEMG[i])
                self.normedEMG[i].append(normedEMG[i])
                self.muscleActivation[i].append(muscleAct[i])

        if (self.usingPolhemus):
            polhemusBase = 66

            lenPose = 6
            for i in range(self.numPoses):
                sensorOut = block[polhemusBase + i].split()
                thisPose = [float(data) for data in sensorOut[2:]]

                for j in range(lenPose):
                    self.poses[i][j].append(thisPose[j])

    def makePlots(self):
        plotNum = 1

        plt.figure(plotNum)
        plt.suptitle("Motor Commands and Positions")
        for i in range(self.robot["motors"]//2):
            ax = plt.subplot(self.robot["motors"]//2, 2, 2*i + 1)
            ax.plot(self.times, self.motorPos[2*i], 'b', linewidth=1)
            ax.plot(self.times, self.motorCom[2*i], 'b--', linewidth=0.5)
            if (i == self.robot["motors"]//2 - 1): plt.xlabel('Time (s)')
            plt.ylabel(str(2*i))

            ax = plt.subplot(self.robot["motors"]//2, 2, 2*i + 2)
            ax.plot(self.times, self.motorPos[2*i + 1], 'b', linewidth=1)
            ax.plot(self.times, self.motorCom[2*i + 1], 'b--', linewidth=0.5)
            if (i == self.robot["motors"]//2 - 1): plt.xlabel('Time (s)')
            plt.ylabel(str(2*i + 1))
        plotNum += 1

        plt.figure(plotNum)
        plt.suptitle("Joint Positions")
        for i in range(self.robot["joints"]//2):
            ax = plt.subplot(self.robot["joints"]//2, 2, 2*i + 1)
            ax.plot(self.times, self.jointPos[2*i], 'b', linewidth=1)
            if (i == self.robot["joints"]//2 - 1): plt.xlabel('Time (s)')
            plt.ylabel(str(2*i))


            ax = plt.subplot(self.robot["joints"]//2, 2, 2*i + 2)
            ax.plot(self.times, self.jointPos[2*i + 1], 'b', linewidth=1)
            if (i == self.robot["joints"]//2 - 1): plt.xlabel('Time (s)')
            plt.ylabel(str(2*i + 1))
        plotNum += 1

        if (self.usingEMG):
            plt.figure(plotNum)
            plt.suptitle("Normalized EMG")
            for i in range(self.numElectrodes):
                ax = plt.subplot(4, 4, i + 1)
                ax.plot(self.times, self.normedEMG[i], 'b', linewidth=1)
                plt.ylim(0, 1)

                if (i >= 12):
                    plt.xlabel('Time (s)')
            plotNum += 1

            plt.figure(plotNum)
            plt.suptitle("Muscle Activation")
            for i in range(self.numElectrodes):
                ax = plt.subplot(4, 4, i + 1)
                ax.plot(self.times, self.muscleActivation[i], 'b', linewidth=1)
                plt.ylim(0, 1)

                if (i >= 12):
                    plt.xlabel('Time (s)')
            plotNum += 1

        plt.show()

# Convert the log file to a usable format
def main(defaultLogPath, path):

    fileName = defaultLogPath + path

    logFile = open(fileName, 'r')

    raw = logFile.readlines()

    file = logParser(logFile, raw) # make a new logParser class instance

    # get the part of log added in each recording and parse it
    for block in file.blocks:
        file.parseBlock(block)

    # make plots of the commanded and reference positions
    file.makePlots()

    print("\nDone.")

if __name__ == "__main__":

    assert len(sys.argv) == 2, "Wrong number of arguments - exactly 1 .txt log file expected"
    print("Processing", sys.argv[1])

    defaultLogPath = "/home/haptix-e15-463/haptix/haptix_controller/logs/"

    main(defaultLogPath, sys.argv[1])