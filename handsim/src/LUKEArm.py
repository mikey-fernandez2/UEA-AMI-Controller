# Mikey Fernandez, 10/22/2021
#
# LUKEArm.py
# Handle communication with the LUKE arm

import can
import time
from can.interface import Bus
import math
import EMGClass

class LUKEArm:
    def __init__(self, config='HC', hand='L', commandDes='DF', commandType='P', interface='socketcan', channel='can0', bitrate=1000000, state=can.bus.BusState.PASSIVE):        
        # make sure valid inputs given
        assert config in ['HC', 'RC'], 'Invalid arm config %s given' % config
        assert hand in ['L', 'R'], 'Invalid handedness %s given' % hand
        assert commandDes in ['DF', 'G'], 'Invalid command description %s given' % commandDes
        assert commandType in ['P', 'V'], 'Invalid control type %s given' % commandType
        
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
        self.startTimestamp = math.inf # need really large number for initial setting of start timestamp

        self.gripRead = None # this is the grip read from the arm
        self.modeRead = None # this is the mode read from the arm
        self.modeSend = None # this is the mode sent to the arm

        self.syncID = 0x080
        self.sensorIDs = [0x4AA, 0x4BF, 0x1A0, 0x1A4, 0x241, 0x341, 0x4C2]
        self.messagesReceived = dict.fromkeys(self.sensorIDs + [self.syncID], 0) # for tracking

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

        self.ACICountsPerDegree = {'wristRot': 2.90, 'wristFlex': 8.47, 'index': 10.36, 'mrp': 10.36, 'thumbP': 9.32, 'thumbY': 12.43, 'humRot': 5, 'elbow': 6.84}
        self.zeroPos = {'index': 50, 'mrp': 50, 'thumbY': 50, 'thumbP': 50, 'wristFlex': 511, 'wristRot': 511, 'humRot': 511, 'elbow': 50} # the ACI command for a position of 0
        self.zeroVel = 512 # the ACI command for a velocity of 0

        if self.hand == "R":
            self.jointRoM = {'index': [0, 90], 'mrp': [0, 90], 'thumbY': [0, 75], 'thumbP': [0, 100], 'wristFlex': [-55, 55], 'wristRot': [-120, 175], 'humRot': [-95, 95], 'elbow': [0, 135]}
        elif self.hand == "L":
            self.jointRoM = {'index': [0, 90], 'mrp': [0, 90], 'thumbY': [0, 75], 'thumbP': [0, 75], 'wristFlex': [-55, 55], 'wristRot': [-120, 175], 'humRot': [-95, 95], 'elbow': [0, 135]}
        else:
            ValueError("LUKEArm(): invalid handedness")

    def recv(self):
        message = self.bus.recv()
        self.arbitration_id = message.arbitration_id
        self.data = message.data
        self.timestamp = message.timestamp
        self.message = message

        # I don't like checking this every time but I don't want to set a boolean flag
        if self.timestamp < self.startTimestamp:
            self.startTimestamp = self.timestamp

    def printSensors(self):
        s = self.sensors # to save me from typing
        print("\nRunning time: %f" % (self.timestamp - self.startTimestamp))
        print("\tCurrent grip: %i | current mode: %i" % (self.gripRead, self.modeRead))
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

    def posToCAN(self, pos, joint):
        return math.floor(self.ACICountsPerDegree[joint]*pos) + self.zeroPos[joint]

    def CANtoPos(self, CAN, joint):
        # convert a 2 byte CAN signal to the corresponding joint position using the joint ROM and scaling appropriately
        rom = self.jointRoM[joint]
        fullCAN = (CAN[0] << 0x8) + CAN[1]
        scaledCAN = fullCAN//2**6
        percent = scaledCAN/0x3FF
        # print(joint, percent)
        return rom[0] + percent*(rom[1] - rom[0])

    def syncAck(self):
        status = self.data[0]
        self.gripRead = status >> 2 & 111
        self.modeRead = (status & 11)
        # print("grip: ", self.gripRead, " | mode: ", self.modeRead)
        # print(self.message)

    def messageCallback(self):
        msg_id = self.arbitration_id
        if msg_id in self.sensorIDs + [self.syncID]:
            self.messagesReceived[msg_id] += 1
            data = list(self.data)
            scale = 2**6
            if msg_id == self.syncID:
                self.syncAck()
                self.sendCommand() # reply to sync message
            elif msg_id == 0x4AA:
                # print("0x4AA", ["%2x" % i for i in data])
                self.sensors['wristRot'] = self.CANtoPos(data[0:2], 'wristRot')
                self.sensors['wristFlex'] = self.CANtoPos(data[2:4], 'wristFlex')
                self.sensors['indexPos'] = self.CANtoPos(data[4:6], 'index')
                self.sensors['mrpPos'] = self.CANtoPos(data[6:8], 'mrp')
            elif msg_id == 0x4BF:
                # print("0x4BF", ["%2x" % i for i in data])
                self.sensors['thumbPPos'] = self.CANtoPos(data[0:2], 'thumbP')
                self.sensors['thumbYPos'] = self.CANtoPos(data[2:4], 'thumbY')
            elif msg_id == 0x1A0:
                self.sensors['humPos'] = self.CANtoPos(data[4:6], 'humRot')
            elif msg_id == 0x1A4:
                self.sensors['elbowPos'] = self.CANtoPos(data[4:6], 'elbow')
            elif msg_id == 0x241:
                self.sensors['indLatF'] = data[0] / 10
                self.sensors['indTipF'] = data[1] / 10
                self.sensors['midTipF'] = data[2] / 10
                self.sensors['ringTipF'] = data[3] / 10
                self.sensors['pinkTipF'] = data[4] / 10
                self.sensors['pinkTipStat'] = (data[5] >> 4) & 1
                self.sensors['ringTipStat'] = (data[5] >> 3) & 1
                self.sensors['midTipStat'] = (data[5] >> 2) & 1
                self.sensors['indTipStat'] = (data[5] >> 1) & 1
                self.sensors['indLatStat'] = data[5] & 1
            elif msg_id == 0x341:
                self.sensors['palmDistF'] = data[0] / 10
                self.sensors['palmProxF'] = data[1] / 10
                self.sensors['handEdgeF'] = data[2] / 10
                self.sensors['handDorsF'] = data[3] / 10
                self.sensors['handDorsStat'] = (data[4] >> 3) & 1
                self.sensors['handEdgeStat'] = (data[4] >> 2) & 1
                self.sensors['palmProxStat'] = (data[4] >> 1) & 1
                self.sensors['palmDistStat'] = data[4] & 1
            elif msg_id == 0x4C2:
                self.sensors['thumbUlF'] = data[0] / 10
                self.sensors['thumbRaF'] = data[1] / 10
                self.sensors['thumbTipF'] = data[2] / 10
                self.sensors['thumbDorsF'] = data[3] / 10
                self.sensors['thumbDorsStat'] = (data[4] >> 3) & 1
                self.sensors['thumbTipStat'] = (data[4] >> 2) & 1
                self.sensors['thumbRaStat'] = (data[4] >> 1) & 1
                self.sensors['thumbUlStat'] = data[4] & 1
            else:
                ValueError('messageCallback(): Invalid sensor message arbitration_id %X' % msg_id)

    def isValidCommand(self):
        # returns true if all commands in the list are between 0x000 and 0x3FF (0 to 1023)
        commandCAN = self.command.values()
        valid = all([com >= 0x000 and com <= 0x3FF for com in commandCAN])
        return valid

    def buildEmptyCommand(self):
        for i in self.command.keys():
            self.command[i] = 0x000

    def buildCommand(self, posCom=None, velCom=None, gripCom=None):
        if posCom != None and velCom == None and gripCom == None:
            # build the position commands here
            # CONTRACT: these positions will all be valid in the RoM for the given joint
            self.command['thumbP'] = self.posToCAN(posCom[0], 'thumbP')
            self.command['thumbY'] = self.posToCAN(posCom[1], 'thumbY')
            self.command['index'] = self.posToCAN(posCom[2], 'index')
            self.command['mrp'] = self.posToCAN(posCom[3], 'mrp')

            self.command['wristRot'] = self.posToCAN(posCom[4], 'wristRot')
            self.command['wristFlex'] = self.posToCAN(posCom[5], 'wristFlex')
            self.command['humRot'] = self.posToCAN(posCom[6], 'humRot')
            self.command['elbow'] = self.posToCAN(posCom[7], 'elbow')

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

    def sendCommand(self):
        if not self.isValidCommand():
            ValueError('sendCommand(): invalid command')

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

        self.send(ACI1, ACI2, ACI3)

    def send(self, dACI1, dACI2, dACI3):
        # print(["%x" % i for i in dACI2])
        ACI1 = can.Message(timestamp=time.time(), arbitration_id=0x210, data=dACI1, is_extended_id=False)
        ACI2 = can.Message(timestamp=time.time(), arbitration_id=0x211, data=dACI2, is_extended_id=False)
        ACI3 = can.Message(timestamp=time.time(), arbitration_id=0x212, data=dACI3, is_extended_id=False)
        self.bus.send(ACI1, timeout=0.5)
        self.bus.send(ACI2, timeout=0.5)
        self.bus.send(ACI3, timeout=0.5)
        # print(ACI1)
        # print(ACI2)
        # print(ACI3, "\n")

    def startup(self):
        # send mode select as 0x000
        # wait > 500 ms (1 s to be safe)
        print("Initializing sensor readings...")

        self.buildEmptyCommand() # initialize empty command
        self.changeMode(0x000) # make sure mode select set as 0x000
        missingReadings = self.sensorIDs.copy()
        while (time.time() - self.startTimestamp < 1 or len(missingReadings) > 0):
            self.recv()
            self.messageCallback()
            
            # populate sensors
            if self.arbitration_id in missingReadings:
                missingReadings.remove(self.arbitration_id)
        
        self.printSensors()

        # send mode select 0x3FF for 100 ms, then send 0x000
        self.enable()
        
    def enable(self):
        self.shortModeSwitch()

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

    def shortModeSwitch(self):
        self.buildEmptyCommand()
        self.changeMode(0x3FF)
        T = time.time()
        while (time.time() - T < 0.1):
            self.recv()
            if self.arbitration_id == self.syncID:
                self.syncAck()
            self.sendCommand()
            time.sleep(0.003)

        self.changeMode(0x000)
        self.sendCommand()

        if self.modeRead == 0: # initial power on
            pass
        elif self.modeRead == 1: # arm mode -> hand mode
            self.changeMode(2)
            print("In hand mode")
        elif self.modeRead == 2: # hand mode -> arm mode
            self.changeMode(1)
            print("In arm mode")
        else:
            ValueError('shortModeSwitch(): wrong original mode %i for short mode switch' % origMode)

    def longModeSwitch(self):
        self.buildEmptyCommand()
        self.changeMode(0x3FF)
        T = time.time()
        while (time.time() - T < 1):
            self.recv()
            if self.arbitration_id == self.syncID:
                self.syncAck()
            self.sendCommand()
            time.sleep(0.003)

        self.changeMode(0x000)
        self.sendCommand()

        self.changeMode(0) # put arm into standy
        print("In standby mode")

    def changeMode(self, modeSend):
        self.modeSend = modeSend
        self.command['modeSelect'] = modeSend # need to set this here, but I'm not happy about it

    def shutdown(self):
        self.longModeSwitch()
        self.bus.shutdown()

    ## FOR TESTING BELOW
    def genSinusoid(self, duration, joint):
        rom = self.jointRoM[joint]
        return 0.5*(rom[1] - rom[0])*(math.sin(2*math.pi*(self.startTimestamp - self.timestamp)/duration) + 1)

def main():
    arm = LUKEArm(config='HC', hand='L', commandDes='DF', commandType='P')
    print('Going through startup.')
    arm.startup()
    if arm.modeRead == arm.modeSend:
        print("Arm enabled")
    else:
        ValueError("startup(): Arm still in standby")

    try:
        count = 0
        while(True):
            arm.recv()
            arm.messageCallback()
            if not count % 1000:
                print(arm.modeRead, arm.modeSend)
                if arm.modeRead != 2:
                    arm.shortModeSwitch()

            if count > 5000:
                index = arm.genSinusoid(1, 'index')
                mrp = arm.genSinusoid(2, 'mrp')
                # elbow = arm.genSinusoid(10, 'elbow')
                thumbP = arm.genSinusoid(3, 'thumbP')
                # print(com)
                # [thumbP, thumbY, index, mrp, wristRot, wristFlex, humRot, elbow]
                posCom = [thumbP, arm.sensors['thumbYPos'], index, mrp, arm.sensors['wristRot'], arm.sensors['wristFlex'], arm.sensors['humPos'], arm.sensors['elbowPos']]
                arm.buildCommand(posCom=posCom)

            count += 1

    except KeyboardInterrupt:
        print("\nProgram Exited")

    except can.CanError:
        print("\nCAN Error")

    arm.shutdown()
    print("Shutting down.")

if __name__ == '__main__':
    print("Starting LUKEArm.py...\n")
    main()