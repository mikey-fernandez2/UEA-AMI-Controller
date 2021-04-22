# Mikey Fernandez, 04/12/2021
#
# printNorms.py
# Print out the saved EMG normalization factors from the file at 'path'

import os, sys, struct

def printNorms(norms):
    print("EMG Bounds:")
    
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

    return

def main(path):    
    try:
        print("Accessing EMG normalization factors...\n")
        with open(path, 'rb') as fifo:
            normsPack = fifo.read()

        norms = struct.unpack("ffffffffffffffffffffffffffffffff", normsPack)
        printNorms(norms)

    except OSError as e:
        print("Could not open file - %s" % e)

    return

if __name__ == "__main__":
    path = "/home/haptix-e15-463/haptix/haptix_controller/handsim/include/scaleFactors.txt"

    main(path)