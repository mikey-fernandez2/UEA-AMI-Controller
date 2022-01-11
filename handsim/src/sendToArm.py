# Mikey Fernandez 12/09/2021
#
# sendToArm.py
# Send messages to the LUKE arm

import sys
import time
import can
from can.interface import Bus
import threading
import zmq

# what I need to do in this file
    # set up the reader to receive the arm commands
    # then make the socket that will receive commands
    # send these commands to the arm (arbitrarily fast, knowing arm will only execute ~100 Hz commands)

class LUKE_Command_Sender:
    def __init__(self, socketAddr="tcp://127.0.0.1:1234"):
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

        self.receiveThread = None

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect(socketAddr)
        self.sock.subscribe("") # Subscribe to all topics
        self.notStopped = True

        self.startup()

    def __del__(self):
        """ Garbage collection """
        try:
            # shutdown the bus
            self.bus.shutdown()
        except:
            print("__del__: Bus shutdown error")

        try:
            # close the socket
            self.sock.close()
            self.ctx.term()
        except:
            print("__del__: Socket closing error")

        print("Exited safely.")

    def printCommandHex(self):
        """ Print the commands in a nice format """
        print("[", ", ".join("{:02x}".format(i) for i in self.dACI1),
             "] [", ", ".join("{:02x}".format(i) for i in self.dACI2),
              "] [", ", ".join("{:02x}".format(i) for i in self.dACI3), "]")

    def writeEmptyCommand(self):
        """ Write an empty command """
        self.dACI1 = [0]*8
        self.dACI2 = [0]*8
        self.dACI3 = [0]*8

    def startup(self):
        """ Ensure the arm will be started up with a "do nothing" command for safety """
        # do nothing until the arm is on - this is a blocking call, so no issues
        self.bus.recv()
        print("First message received - beginning communication with the arm")

        self.writeEmptyCommand()

        self.sendThread = threading.Thread(target=self.sendData)
        self.sendThread.daemon = True
        self.sendThread.start()

    def safetyCheck(self, commands):
        return len(commands) != 0 and commands[0:8] != [] and commands[8:16] != [] and commands[16:] != []

    def receiveData(self):
        while True:
            try:
                commsPacked = self.sock.recv()
            except KeyboardInterrupt:
                break

            commands = list(commsPacked)

            # try to do some safey checks here?
            if not self.safetyCheck(commands):
                raise ValueError('receiveData(): problem with received data')

            # should be packing just a bunch of bytes, just access the right elements of the byte array here?
            self.dACI1 = commands[0:8]
            self.dACI2 = commands[8:16]
            self.dACI3 = commands[16:]

            self.printCommandHex()

            if (self.dACI1 == [] or self.dACI2 == [] or self.dACI3 == []):
                raise ValueError('receiveData(): empty commands send to arm')

        self.notStopped = False
        self.sendThread.join()
        print("\nExiting...")

    def sendData(self):
        """ Send the messages to the arm """
        while self.notStopped:
            ACI1 = can.Message(timestamp=time.time(), arbitration_id=0x210, data=self.dACI1, is_extended_id=False)
            ACI2 = can.Message(timestamp=time.time(), arbitration_id=0x211, data=self.dACI2, is_extended_id=False)
            ACI3 = can.Message(timestamp=time.time(), arbitration_id=0x212, data=self.dACI3, is_extended_id=False)
            self.bus.send(ACI1, timeout=None)
            self.bus.send(ACI2, timeout=None)
            self.bus.send(ACI3, timeout=None)

            time.sleep(0.007) # sleep for a little bit before sending the new message, give time to update

if __name__ == "__main__":
    print("Waiting for arm to be turned on...")

    socketAddr = "tcp://127.0.0.1:1234"
    sender = LUKE_Command_Sender(socketAddr=socketAddr)
    sender.receiveData()