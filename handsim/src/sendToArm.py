# Mikey Fernandez 12/09/2021
#
# sendToArm.py
# Send messages to the LUKE arm

import sys
import time
import os
import can
from can.interface import Bus

# what I need to do in this file
    # set up the reader to receive the arm commands
    # then open and read from the command file
    # send these commands to the arm (arbitrarily fast, knowing arm will only accept ~100 Hz commands)

class LUKE_Command_Sender:
    def __init__(self, commPath):

        self.path = commPath
        self.emptyCom = bytearray([0]*24) # all zeros

        self.dACI1 = None
        self.dACI2 = None
        self.dACI3 = None

        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = 'can0'
        can.rc['bitrate'] = 1000000
        can.rc['state'] = can.bus.BusState.PASSIVE
        try:
            self.bus = Bus()
        except:
            print("__init__(): Make sure to run 'CANon' first to establish communication.")

        self.dACI1 = None
        self.dACI2 = None
        self.dACI3 = None

        self.startup()

    def __del__(self):
        """ Garbage collection """
        try:
            # shutdown the bus
            self.bus.shutdown()
        except:
            pass

        try:
            # delete the empty file
            os.remove(self.path)
        except:
            pass

        print("Exited safely.")

    def writeEmptyCommand(self):
        """ Write an empty command to the file """
        with open(self.path, 'wb') as output:
            output.write(self.emptyCom)

    def startup(self):
        """ Ensure the arm will be started up with a "do nothing" command for safety """

        # do nothing until the arm is on?
        self.bus.recv()
        print("First message received - beginning communication with the arm")

        self.writeEmptyCommand()

    def safetyCheck(self, commands):
        return len(commands) != 0 and commands[0:8] != [] and commands[8:16] != [] and commands[16:] != []

    def receiveData(self):
        try:
            while True:
                # open the commands file, read, prepare to send
                with open(self.path, 'rb') as fifo:
                    commsPacked = fifo.read()

                commands = list(commsPacked)
                # try to do some safey checks here?
                if not self.safetyCheck(commands):
                    ValueError('receiveData(): problem with received data')

                # should be packing just a bunch of bytes, just access the right elements of the byte array here?
                self.dACI1 = commands[0:8]
                self.dACI2 = commands[8:16]
                self.dACI3 = commands[16:]

                if (self.dACI1 == [] or self.dACI2 == [] or self.dACI3 == []):
                    ValueError('receiveData(): empty commands send to arm')

                self.sendData()
                time.sleep(0.007) # sleep for a little bit before sending the new message, give time to update
                print(self.dACI1, self.dACI2, self.dACI2)

        except KeyboardInterrupt:
            print("Exiting.\n")

    def sendData(self):
        """ Send the messages to the arm """
        ACI1 = can.Message(timestamp=time.time(), arbitration_id=0x210, data=self.dACI1, is_extended_id=False)
        ACI2 = can.Message(timestamp=time.time(), arbitration_id=0x211, data=self.dACI2, is_extended_id=False)
        ACI3 = can.Message(timestamp=time.time(), arbitration_id=0x212, data=self.dACI3, is_extended_id=False)
        self.bus.send(ACI1, timeout=None)
        self.bus.send(ACI2, timeout=None)
        self.bus.send(ACI3, timeout=None)

if __name__ == "__main__":

    commPath = "/tmp/command"
    sender = LUKE_Command_Sender(commPath)
    sender.receiveData()