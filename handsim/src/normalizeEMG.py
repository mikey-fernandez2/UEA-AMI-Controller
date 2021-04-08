# Mikey Fernandez, 03/19/2021
#
# noramlizeEMG.py
# Collect EMG from Wifi Board and save the maximum value reached from each electrode.
# Saved into a txt file that will be read by the controller to normalize EMG signals.

# from signal import signal, SIGINT
import os, sys, time, struct

def compareMax(maxes, EMGData, numElectrodes):
    # byte_data = struct.pack("IHffffffffffffffffBBBB", data[4], data[5], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14],
    raw_data = struct.unpack("IHffffffffffffffffBBBB", EMGData)
    sensors = raw_data[2:2 + numElectrodes] # extract the EMG data

    for electrode in range(numElectrodes):
        if sensors[electrode] > maxes[electrode]:
            maxes[electrode] = sensors[electrode]

    return maxes

def main(path, numElectrodes, scaleFactorsPath):    
    try:
        print("Receiving...")
        maxes = [0]*numElectrodes
        msgLen = 76
        while(1):
            fifo = os.open(path, os.O_RDONLY)
            EMGData = os.read(fifo, msgLen)
            os.close(fifo)

            maxes = compareMax(maxes, EMGData, numElectrodes)

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nTest send complete\n")
        os.remove(path)

        maxesbytes = struct.pack("ffffffffffffffff", maxes[0], maxes[1], maxes[2], maxes[3], maxes[4], maxes[5], maxes[6], maxes[7],
                                                     maxes[8], maxes[9], maxes[10], maxes[11], maxes[12], maxes[13], maxes[14], maxes[15])

        output = os.open(scaleFactorsPath, os.O_WRONLY)
        os.write(output, maxesbytes)
        os.close(output)

    return

if __name__ == "__main__":
     # Tell Python to run the handler() function when SIGINT is recieved
    # signal(SIGINT, handler)

    # Path to be created
    path = "/tmp/emg"
    scaleFactorsPath = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/scaleFactors.txt"
    numElectrodes = 16

    main(path, numElectrodes, scaleFactorsPath)