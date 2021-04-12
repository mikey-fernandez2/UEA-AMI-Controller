# Mikey Fernandez, 04/12/2021
#
# printNorms.py
# Print out the saved EMG normalization factors from the file at 'path'

import os, sys, struct

def printNorms(norms):
    print("Scaling factors:")
    
    for elec in range(len(norms)//4):
        print("\t%06.2f %06.2f %06.2f %06.2f" % (norms[elec], norms[elec + 1], norms[elec + 2], norms[elec + 3]))

    return

def main(path):    
    try:
        print("Accessing EMG normalization factors...\n")
        fifo = open(path, 'rb')
        normsPack = fifo.read()
        fifo.close()

        norms = struct.unpack("ffffffffffffffff", normsPack)
        printNorms(norms)

    except OSError as e:
        print("Could not open file - %s" % e)

    return

if __name__ == "__main__":
    path = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/scaleFactors.txt"

    main(path)