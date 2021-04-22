# Mikey Fernandez, 04/07/2021
#
# noramlizeEMG.py
# Collect EMG from Wifi Board and save the maximum value reached from each electrode.
# Saved into a txt file that will be read by the controller to normalize EMG signals.

import os, sys, time, struct

def compareMax(maxes, raw_emg):
    for electrode in range(len(raw_emg)):
        if raw_emg[electrode] > maxes[electrode]:
            maxes[electrode] = raw_emg[electrode]

    return maxes

def compareMin(mins, raw_emg):
    for electrode in range(len(raw_emg)):
        if raw_emg[electrode] < mins[electrode]:
            mins[electrode] = raw_emg[electrode]

    return mins

def main(path, numElectrodes, scaleFactorsPath, sendingFreq):    
    try:
        print("Receiving...")
        maxes = [0]*numElectrodes
        mins = [sys.maxsize]*numElectrodes
        msgLen = struct.calcsize("ffffffffffffffffffIIIIf")
        print(msgLen)
        recLen = 0
        while(1):
            while recLen == 0:
                with open(path, 'rb') as fifo:
                    EMGData = fifo.read()
                recLen = len(EMGData)

            raw_data = struct.unpack("ffffffffffffffffffIIIIf", EMGData)
            raw_emg = raw_data[2:2 + numElectrodes] # extract the EMG data

            maxes = compareMax(maxes, raw_emg)
            mins = compareMin(mins, raw_emg)

            recLen = 0
            time.sleep(1/sendingFreq)

    except KeyboardInterrupt:
        print("\nReceiving complete.\nWriting normalization factors to %s\n" % scaleFactorsPath)

        # print(maxes)
        # print(mins)
        
        normsBytes = struct.pack("ffffffffffffffffffffffffffffffff", maxes[0], maxes[1], maxes[2], maxes[3], maxes[4], maxes[5], maxes[6], maxes[7],
                                                                     maxes[8], maxes[9], maxes[10], maxes[11], maxes[12], maxes[13], maxes[14], maxes[15],
                                                                     mins[0], mins[1], mins[2], mins[3], mins[4], mins[5], mins[6], mins[7],
                                                                     mins[8], mins[9], mins[10], mins[11], mins[12], mins[13], mins[14], mins[15])

        with open(scaleFactorsPath, 'wb') as output:
            output.write(normsBytes)

    return

if __name__ == "__main__":

    path = "/tmp/emg"
    scaleFactorsPath = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/scaleFactors.txt"
    numElectrodes = 16
    sendingFreq = 20

    main(path, numElectrodes, scaleFactorsPath, sendingFreq)