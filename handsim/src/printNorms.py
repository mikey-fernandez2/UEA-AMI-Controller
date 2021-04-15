# Mikey Fernandez, 04/12/2021
#
# printNorms.py
# Print out the saved EMG normalization factors from the file at 'path'

import os, sys, struct

def printNorms(norms):
    print("Scaling factors:")
    
    print("\t%06.2f %06.2f %06.2f %06.2f\n\t%06.2f %06.2f %06.2f %06.2f\n\t%06.2f %06.2f %06.2f %06.2f\n\t%06.2f %06.2f %06.2f %06.2f" % 
         (norms[0], norms[1], norms[2], norms[3], norms[4], norms[5], norms[6], norms[7],
          norms[8], norms[9], norms[10], norms[11], norms[12], norms[13], norms[14], norms[15]))

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