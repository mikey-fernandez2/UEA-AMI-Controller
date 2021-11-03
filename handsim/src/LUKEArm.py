# Mikey Fernandez, 10/22/2021
#
# LUKEArm.py
# Handle communication with the LUKE arm

import can
import time
from can.interface import Bus
import math

class LUKEArm:
    def __init__(self, config='HC', hand='L', commandDes='DF', commandType='P', interface='socketcan', channel='can0', bitrate=1000000, state=can.bus.BusState.PASSIVE):        
        self.config = config
        self.hand = hand
        self.commandDes = commandDes
        self.commandType = commandType
        can.rc['interface'] = interface
        can.rc['channel'] = channel
        can.rc['bitrate'] = bitrate
        can.rc['state'] = state

        self.bus = Bus()

        self.message = None
        self.arbitration_id = 0
        self.data = None
        self.timestamp = 0

        self.grip = None
        self.mode = None

        self.syncID = 0x080
        self.sensorIDs = [0x4AA, 0x4BF, 0x1A0, 0x1A4, 0x241, 0x341, 0x4C2]

        self.sensors = {'wristRot': -1, 'wristFlex': -1, 'indexPos': -1, 'mrpPos': -1,
                        'thumbPPos': -1, 'thumbYPos': -1,
                        'humPos': -1,
                        'elbowPos': -1,
                        'indLatF': -1, 'indTipF': -1, 'midTipF': -1, 'ringTipF': -1, 'pinkTipF': -1, 'pinkTipStat': -1, 'ringTipStat': -1, 'midTipStat': -1, 'indTipStat': -1, 'indLatStat': -1,
                        'palmDistF': -1, 'palmProxF': -1, 'handEdgeF': -1, 'handDorsF': -1, 'handDorsStat': -1, 'handEdgeStat': -1, 'palmProxStat': -1, 'palmDistStat': -1,
                        'thumbUlF': -1, 'thumbRaF': -1, 'thumbTipF': -1, 'thumbDorsF': -1, 'thumbDorsStat': -1, 'thumbTipStat': -1, 'thumbRaStat': -1, 'thumbUlStat': -1}

        self.command = {'modeSelect': 0x000, 'thumbP': 0x000, 'thumbY': 0x000, 'index': 0x000, 'mrp': 0x000,
                        'handOC': 0x000, 'grip': 0x000,
                        'wristRot': 0x000, 'wristFlex': 0x000, 'humRot': 0x000, 'elbow': 0x000}

        self.ACICountsPerDegree = {'wristRot': 2.90, 'wristFlex': 8.47, 'index': 10.36, 'mrp': 10.36, 'thumbP': 9.32, 'thumbY': 12.43, 'humRot': 0, 'elbow': 0}
        self.zeroPos = {'wrist': 511, 'fingers': 50, 'humRot': 511, 'elbow': 511} # the ACI command for a position of 0
        self.zeroVel = 512 # the ACI command for a velocity of 0

        self.initialize()

    def initialize(self):
        # For initialization
        # This is to get all the joint positions to prevent faults
        print("Initializing with sensor readings...")
        self.recv()
        self.startTimestamp = self.timestamp # record very first message received timestamp
        missingReadings = self.sensorIDs.copy()
        seenSync = False
        while len(missingReadings) > 0 and not seenSync:
            if self.arbitration_id in missingReadings:
                self.messageCallback()
                missingReadings.remove(self.arbitration_id)
            elif self.arbitration_id == self.syncID:
                status = self.data[0]
                self.grip = status >> 2 & 111
                self.changeMode(status & 11)
                seenSync = True

                # TODO: I FOUND YOU ERROR - I'm not acknowledging sync messages while I do this, so the arm is faulting
                # Adjust to reply to syncs in here with 0x00 commands or change how I initialize the arm position
                #   second option probably better
            
            self.recv()
        print("All sensor readings recorded.")
        self.printSensors()

    def recv(self):
        message = self.bus.recv()
        self.arbitration_id = message.arbitration_id
        self.data = message.data
        self.timestamp = message.timestamp
        self.message = message

    def printSensors(self):
        s = self.sensors # to save me from typing
        print("\nTimestamp: %f" % self.timestamp)
        print("\tCurrent grip: %i | current mode: %i" % (self.grip, self.mode))
        print("\n\tJoint positions:")
        print("\t\t  Hum rot: %8.3f |   Elbow pos: %8.3f" % (s['humPos'], s['elbowPos']))
        print("\t\tWrist rot: %8.3f |  wrist flex: %8.3f" % (s['wristRot'], s['wristFlex']))
        print("\t\tThumb yaw: %8.3f | thumb pitch: %8.3f" % (s['thumbYPos'], s['thumbPPos']))
        print("\t\tIndex pos: %8.3f |     MRP pos: %8.3f" % (s['indexPos'], s['mrpPos']))
        print("\n\tForce sensors (status):")
        print("\t\t  Index Lat: %8.3f (%1i) | index tip: %8.3f (%1i) |   mid tip: %8.3f (%1i) |   ring tip: %8.3f (%1i) | pinky tip: %8.3f (%1i)" %
                (s['indLatF'], s['indLatStat'], s['indTipF'], s['indTipStat'], s['midTipF'], s['midTipStat'], s['ringTipF'], s['ringTipStat'], s['pinkTipF'], s['pinkTipStat']))
        print("\t\t  Palm dist: %8.3f (%1i) | palm prox: %8.3f (%1i) | hand edge: %8.3f (%1i) |  hand dors: %8.3f (%1i)" %
                (s['palmDistF'], s['palmDistStat'], s['palmProxF'], s['palmProxStat'], s['handEdgeF'], s['handEdgeStat'], s['handDorsF'], s['handDorsStat']))
        print("\t\tThumb ulnar: %8.3f (%1i) | thumb rad: %8.3f (%1i) | thumb tip: %8.3f (%1i) | thumb dors: %8.3f (%1i)" %
                (s['thumbUlF'], s['thumbUlStat'], s['thumbRaF'], s['thumbRaStat'], s['thumbTipF'], s['thumbTipStat'], s['thumbDorsF'], s['thumbDorsStat']))
        print("\n")

    def printCommand(self):
        c = self.command # to save me from typing
        print("\nMode Select: %i" % c['modeSelect'])
        print("Joint positions:")
        print("\t  Hum rot: %03x |   Elbow pos: %03x" % (c['humRot'], c['elbow']))
        print("\tWrist rot: %03x |  wrist flex: %03x" % (c['wristRot'], c['wristFlex']))
        print("\tThumb yaw: %03x | thumb pitch: %03x" % (c['thumbY'], c['thumbP']))
        print("\tIndex pos: %03x |     MRP pos: %03x" % (c['index'], c['mrp']))
        print("Hand Mode:")
        print("\tHand OC: %i | Grip: %i" % (c['handOC'], c['grip']))
        print("\n")

    def isValidCommand(self):
        # returns true if all commands in the list are between 0x000 and 0x3FF (0 to 1023)
        commandCAN = self.command.values()
        valid = all([com >= 0x000 and com <= 0x3FF for com in commandCAN])
        return valid

    def syncAck(self):
        status = self.data[0]
        self.grip = status >> 2 & 111
        self.changeMode(status & 11)
        # print("grip: ", self.grip, " | mode: ", self.modeSelect)

        print(self.message)
        # print("Sync message: ", bin(status))
        # self.sendCommand()
        indCom = self.genIndexSinusoid(5)
        # indCom = 0
        self.buildCommand(posCom=[0, 0, indCom, 0, 0, 0, 0, 0])
        self.sendCommand()

    def messageCallback(self):
        msg_id = self.arbitration_id
        if msg_id == self.syncID:
            self.syncAck()

        elif msg_id in self.sensorIDs:
            # print(self.sensors, "\n\n")
            data = self.data
            scale = 2**6
            if msg_id == 0x4AA:
                self.sensors['wristRot'] = data[0]//scale << 8 + data[1]//scale
                self.sensors['wristFlex'] = data[2]//scale << 8 + data[3]//scale
                self.sensors['indexPos'] = data[4]//scale << 8 + data[5]//scale
                self.sensors['mrpPos'] = data[6]//scale << 8 + data[7]//scale
            elif msg_id == 0x4BF:
                self.sensors['thumbPPos'] = data[0]//scale << 8 + data[1]//scale
                self.sensors['thumbYPos'] = data[2]//scale << 8 + data[3]//scale
            elif msg_id == 0x1A0:
                self.sensors['humPos'] = data[4]//scale << 8 + data[5]//scale
            elif msg_id == 0x1AF:
                self.sensors['elbowPos'] = data[4]//scale << 8 + data[5]//scale
            elif msg_id == 0x241:
                self.sensors['indLatF'] = data[0] / 10
                self.sensors['indTipF'] = data[1] / 10
                self.sensors['midTipF'] = data[2] / 10
                self.sensors['ringTipF'] = data[3] / 10
                self.sensors['pinkTipF'] = data[4] / 10
                self.sensors['pinkTipStat'] = data[5] & (1 << 4)
                self.sensors['ringTipStat'] = data[5] & (1 << 3)
                self.sensors['midTipStat'] = data[5] & (1 << 2)
                self.sensors['indTipStat'] = data[5] & (1 << 1)
                self.sensors['indLatStat'] = data[5] & 1
            elif msg_id == 0x341:
                self.sensors['palmDistF'] = data[0] / 10
                self.sensors['palmProxF'] = data[1] / 10
                self.sensors['handEdgeF'] = data[2] / 10
                self.sensors['handDorsF'] = data[3] / 10
                self.sensors['handDorsStat'] = data[4] & (1 << 3)
                self.sensors['handEdgeStat'] = data[4] & (1 << 2)
                self.sensors['palmProxStat'] = data[4] & (1 << 1)
                self.sensors['palmDistStat'] = data[4] & 1
            elif msg_id == 0x4C2:
                self.sensors['thumbUlF'] = data[0] / 10
                self.sensors['thumbRaF'] = data[1] / 10
                self.sensors['thumbTipF'] = data[2] / 10
                self.sensors['thumbDorsF'] = data[3] / 10
                self.sensors['thumbDorsStat'] = data[4] & (1 << 3)
                self.sensors['thumbTipStat'] = data[4] & (1 << 2)
                self.sensors['thumbRaStat'] = data[4] & (1 << 1)
                self.sensors['thumbUlStat'] = data[4] & 1
            else:
                ValueError('messageCallback(): Invalid sensor message arbitration_id %X' % msg_id)

    def buildCommand(self, posCom=None, velCom=None, gripCom=None):
        self.command['modeSelect'] = self.mode

        if posCom != None and velCom == None and gripCom == None:
            # build the position commands here
            # CONTRACT: these positions will all be valid in the RoM for the given joint
            self.command['thumbP'] = math.floor(self.ACICountsPerDegree['thumbP']*posCom[0]) + self.zeroPos['fingers']
            self.command['thumbY'] = math.floor(self.ACICountsPerDegree['thumbY']*posCom[1]) + self.zeroPos['fingers']
            self.command['index'] = math.floor(self.ACICountsPerDegree['index']*posCom[2]) + self.zeroPos['fingers']
            self.command['mrp'] = math.floor(self.ACICountsPerDegree['mrp']*posCom[3]) + self.zeroPos['fingers']

            self.command['wristRot'] = math.floor(self.ACICountsPerDegree['wristRot']*posCom[4]) + self.zeroPos['wrist']
            self.command['wristFlex'] = math.floor(self.ACICountsPerDegree['wristFlex']*posCom[5]) + self.zeroPos['wrist']
            self.command['humRot'] = math.floor(self.ACICountsPerDegree['humRot']*posCom[6]) + self.zeroPos['humRot']
            self.command['elbow'] = math.floor(self.ACICountsPerDegree['elbow']*posCom[7]) + self.zeroPos['elbow']

        elif posCom == None and velCom != None and gripCom == None:
            # build the velocity commands here
            # CONTRACT: these velocities will be in [-1, 1] representing fraction of maximum velocity in each direction
            maxVel = 0x3FF//2
            self.command['thumbP'] =  math.floor(maxVel*velCom[0]) + self.zeroVel
            self.command['thumbY'] = math.floor(maxVel*velCom[1]) + self.zeroVel
            self.command['index'] = math.floor(maxVel*velCom[2]) + self.zeroVel
            self.command['mrp'] = math.floor(maxVel*velCom[3]) + self.zeroVel

            self.command['wristRot'] = math.floor(maxVel*velCom[4]) + self.zeroVel
            self.command['wristFlex'] = math.floor(maxVel*velCom[5]) + self.zeroVel
            self.command['humRot'] = math.floor(maxVel*velCom[6]) + self.zeroVel
            self.command['elbow'] = math.floor(maxVel*velCom[7]) + self.zeroVel

        elif posCom == None and velCom == None and gripCom != None:
            # build the grip commands here
            # CONTRACT: TODO
            self.command['handOC'] = gripCom[0]
            self.command['grip'] = gripCom[1]

        else:
            ValueError('buildCommand(): command type incorrect')

        if not self.isValidCommand():
            ValueError('buildCommand(): invalid command')

        # self.printCommand()
        # print(self.command['index'])

    def sendCommand(self):
        c = self.command
        ACI1 = [c['modeSelect'] >> 0x8, c['modeSelect'] & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ACI3 = [c['wristRot'] >> 0x8, c['wristRot'] & 0xFF, c['wristFlex'] >> 0x8, c['wristFlex'] & 0xFF, c['humRot'] >> 0x8, c['humRot'] & 0xFF, c['elbow'] >> 0x8, c['elbow'] & 0xFF]

        if self.commandDes == 'DF':
            # make the direct finger control commands here
            ACI2 = [c['thumbP'] >> 0x8, c['thumbP'] & 0xFF, c['thumbY'] >> 0x8, c['thumbY'] & 0xFF, c['index'] >> 0x8, c['index'] & 0xFF, c['mrp'] >> 0x8, c['mrp'] & 0xFF]
        elif self.commandDes == 'G':
            # make the grip control commands here
            ACI2 = [c['handOC'] >> 0x8, c['handOC'] & 0xFF, c['grip'] >> 0x8, c['grip'] & 0xFF, 0x00, 0x00, 0x00, 0x00]
        else:
            ValueError('sendCommand(): Incorrect command description %s' % self.commandDes)

        # print(ACI1, ACI2, ACI3)
        self.sendAck(ACI1, ACI2, ACI3)

    def sendAck(self, dACI1, dACI2, dACI3):
        ACI1 = can.Message(timestamp=time.time(), arbitration_id=0x210, data=dACI1, is_extended_id=False)
        ACI2 = can.Message(timestamp=time.time(), arbitration_id=0x211, data=dACI2, is_extended_id=False)
        ACI3 = can.Message(timestamp=time.time(), arbitration_id=0x212, data=dACI3, is_extended_id=False)
        self.bus.send(ACI1, timeout=0.5)
        self.bus.send(ACI2, timeout=0.5)
        self.bus.send(ACI3, timeout=0.5)
        print(ACI1)
        print(ACI2)
        print(ACI3, "\n")

    def startup(self):
        # send mode select as 0x000
        # wait > 500 ms (1 s to be safe)
        # send mode select 0x3FF for 100 ms, then send 0x000
        dACI1 = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        dACI2 = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        dACI3 = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        self.sendCommand() # at power on, send mode select at 0x000

        if self.commandType == 'P' and self.commandDes == 'DF':
            # [thumbP, thumbY, index, mrp, wristRot, wristFlex, humRot, elbow]
            posCom = [self.sensors['thumbPPos'], self.sensors['thumbYPos'], self.sensors['indexPos'], self.sensors['mrpPos'], self.sensors['wristRot'], self.sensors['wristFlex'], self.sensors['humPos'], self.sensors['elbowPos']]
            self.buildCommand(posCom=posCom)
        elif self.commandType == 'V' and self.commandDes == 'DF':
            # [thumbP, thumbY, index, mrp, wristRot, wristFlex, humRot, elbow]
            velCom = [0, 0, 0, 0, 0, 0, 0, 0]
            self.buildCommand(velCom=velCom)
        else:
            # [handOC, grip]
            gripCom = [0, 0]
            self.buildCommand(gripCom=gripCom)

        time.sleep(1)

        self.shortModeSwitch()

    def shortModeSwitch(self):
        origMode = self.mode # store original mode

        self.command['modeSelect'] = 0x3FF
        T = time.time()
        while (time.time() - T < 0.1):
            self.sendCommand()
            time.sleep(0.01)

        self.command['modeSelect'] = 0x000
        self.sendCommand()

        if origMode in [0, 1]: # initial power on, arm mode -> hand mode
            self.changeMode(2)
        elif origMode == 2: # hand mode -> arm mode
            self.changeMode(1)
        else:
            ValueError('shortModeSwitch(): wrong original mode for short mode switch')

    def longModeSwitch(self):
        self.command['modeSelect'] = 0x3FF
        T = time.time()
        while (time.time() - T < 1.1):
            self.sendCommand()
            time.sleep(0.01)

        self.command['modeSelect'] = 0x000
        self.sendCommand()

        self.changeMode(0) # put arm into standy

    def changeMode(self, modeSelect):
        self.mode = modeSelect
        # print("Mode: ", modeSelect)

    def shutdown(self):
        if self.mode == 1:
            self.shortModeSwitch()
        self.longModeSwitch()
        self.bus.shutdown()

    ## FOR TESTING BELOW
    def genIndexSinusoid(self, duration):
        return 45*(math.sin(2*math.pi*(self.startTimestamp - self.timestamp)/duration) + 1)

def main(alreadyOn):
    arm = LUKEArm(config='HC', hand='L', commandDes='DF', commandType='P')
    print('Going through startup.')
    arm.startup()
    print("Arm enabled")

    try:
        count = 0
        while(True):
            arm.recv()
            arm.messageCallback()

            if count % 10000 == 0:
                # arm.shortModeSwitch()
                continue

            count += 1

    except KeyboardInterrupt:
        print("\nProgram Exited")

    except can.CanError:
        print("\nCAN Error")

    arm.shutdown()
    print("Shutting down.")

if __name__ == '__main__':
    print("Running...\n")
    alreadyOn = False
    main(alreadyOn)