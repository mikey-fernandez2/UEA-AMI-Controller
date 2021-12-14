# Mikey Fernandez, 10/22/2021
#
# LUKEArm.py
# Handle communication with the LUKE arm

import can
import time
from can.interface import Bus
import math
from EMGClass import EMG
from controllerClass import impedanceController
import sys
import struct

# interface='socketcan', channel='can0', bitrate=1000000, state=can.bus.BusState.PASSIVE

class LUKEArm:
    def __init__(self, config='HC', hand='L', commandDes='DF', commandType='P', commPath='/tmp/command', usingEMG=False):        
        # make sure valid inputs given
        assert config in ['HC', 'RC'], 'Invalid arm config %s given' % config
        assert hand in ['L', 'R'], 'Invalid handedness %s given' % hand
        assert commandDes in ['DF', 'G'], 'Invalid command description %s given' % commandDes
        assert commandType in ['P', 'V'], 'Invalid control type %s given' % commandType
        
        # setup arm state
        self.config = config
        self.hand = hand
        self.commandDes = commandDes
        self.commandType = commandType
        self.usingEMG = usingEMG
        self.commPath = commPath

        if config == 'HC':
            self.numMotors = 8
        else:
            self.numMotors = 6

        # setup communication with arm
        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = 'can0'
        can.rc['bitrate'] = 1000000
        can.rc['state'] = can.bus.BusState.PASSIVE
        self.bus = Bus()

        # setup message state
        self.message = None
        self.arbitration_id = 0
        self.data = None
        self.timestamp = 0
        self.startTimestamp = math.inf # need really large number for initial setting of start timestamp

        self.gripRead = -1 # this is the grip read from the arm
        self.modeRead = -1 # this is the mode read from the arm
        self.modeSend = -1 # this is the mode sent to the arm

        # constant message identifiers
        self.syncID = 0x080
        self.sensorIDs = [0x4AA, 0x4BF, 0x1A0, 0x1A4, 0x241, 0x341, 0x4C2]
        self.messagesReceived = dict.fromkeys(self.sensorIDs + [self.syncID], 0) # for tracking number of messages received

        # initial command and sensor settings
        self.sensorPositions = ['humPos', 'elbowPos', 'wristRot', 'wristFlex', 'thumbYPos', 'thumbPPos', 'indexPos', 'mrpPos']
        self.sensorForces = ['indLatF', 'indTipF', 'midTipF', 'ringTipF', 'pinkTipF', 'palmDistF', 'palmProxF', 'handEdgeF', 'handDorsF', 'thumbUlF', 'thumbRaF', 'thumbTipF', 'thumbDorsF']
        self.sensorStatus = ['indLatStat', 'indTipStat', 'midTipStat', 'ringTipStat', 'pinkTipStat', 'palmDistStat', 'palmProxStat', 'handEdgeStat', 'handDorsStat', 'thumbUlStat', 'thumbRaStat', 'thumbTipStat', 'thumbDorsStat']
        self.sensors = dict.fromkeys(self.sensorPositions + self.sensorForces + self.sensorStatus, -1)

        self.command = {'modeSelect': 0x000, 'thumbP': 0x000, 'thumbY': 0x000, 'index': 0x000, 'mrp': 0x000,
                        'handOC': 0x000, 'grip': 0x000,
                        'wristRot': 0x000, 'wristFlex': 0x000, 'humRot': 0x000, 'elbow': 0x000}

        # CAN controller settings
        self.ACICountsPerDegree = {'wristRot': 2.90, 'wristFlex': 8.47, 'index': 10.36, 'mrp': 10.36, 'thumbP': 9.32, 'thumbY': 12.43, 'humRot': 5, 'elbow': 6.84}
        self.zeroPos = {'index': 50, 'mrp': 50, 'thumbY': 50, 'thumbP': 50, 'wristFlex': 511, 'wristRot': 511, 'humRot': 511, 'elbow': 50} # the ACI command for a position of 0
        self.zeroVel = 512 # the ACI command for a velocity of 0

        # joint limits
        if self.hand == "R":
            self.jointRoM = {'indexPos': [0, 90], 'mrpPos': [0, 90], 'thumbYPos': [0, 75], 'thumbPPos': [0, 100], 'wristFlex': [-55, 55], 'wristRot': [-120, 175], 'humPos': [-95, 95], 'elbowPos': [0, 135]}
        elif self.hand == "L":
            self.jointRoM = {'indexPos': [0, 90], 'mrpPos': [0, 90], 'thumbYPos': [0, 75], 'thumbPPos': [0, 75], 'wristFlex': [-55, 55], 'wristRot': [-120, 175], 'humPos': [-95, 95], 'elbowPos': [0, 135]}
        else:
            ValueError("LUKEArm(): invalid handedness")

        ## FOR DEBUGGING
        self.replies = 0

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
        return rom[0] + percent*(rom[1] - rom[0])

    def syncAck(self):
        status = self.data[0]
        self.gripRead = status >> 2 & 111
        self.modeRead = (status & 11)

    def messageCallback(self):
        msg_id = self.arbitration_id
        if msg_id in self.sensorIDs + [self.syncID]:
            self.messagesReceived[msg_id] += 1
            data = list(self.data)
            if msg_id == self.syncID:
                self.syncAck()
                self.sendCommand() # reply to sync message
            elif msg_id == 0x4AA:
                self.sensors['wristRot'] = self.CANtoPos(data[0:2], 'wristRot')
                self.sensors['wristFlex'] = self.CANtoPos(data[2:4], 'wristFlex')
                self.sensors['indexPos'] = self.CANtoPos(data[4:6], 'indexPos')
                self.sensors['mrpPos'] = self.CANtoPos(data[6:8], 'mrpPos')
            elif msg_id == 0x4BF:
                self.sensors['thumbPPos'] = self.CANtoPos(data[0:2], 'thumbPPos')
                self.sensors['thumbYPos'] = self.CANtoPos(data[2:4], 'thumbYPos')
            elif msg_id == 0x1A0:
                self.sensors['humPos'] = self.CANtoPos(data[4:6], 'humPos')
            elif msg_id == 0x1A4:
                self.sensors['elbowPos'] = self.CANtoPos(data[4:6], 'elbowPos')
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

    def buildCurPosCommand(self):
        return [self.sensors['thumbPPos'], self.sensors['thumbYPos'], self.sensors['indexPos'], self.sensors['mrpPos'], self.sensors['wristRot'], self.sensors['wristFlex'], self.sensors['humPos'], self.sensors['elbowPos']]

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
        dACI1 = [c['modeSelect'] >> 0x8, c['modeSelect'] & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        dACI3 = [c['wristRot'] >> 0x8, c['wristRot'] & 0xFF, c['wristFlex'] >> 0x8, c['wristFlex'] & 0xFF, c['humRot'] >> 0x8, c['humRot'] & 0xFF, c['elbow'] >> 0x8, c['elbow'] & 0xFF]

        if self.commandDes == 'DF':
            # make the direct finger control commands here
            dACI2 = [c['thumbP'] >> 0x8, c['thumbP'] & 0xFF, c['thumbY'] >> 0x8, c['thumbY'] & 0xFF, c['index'] >> 0x8, c['index'] & 0xFF, c['mrp'] >> 0x8, c['mrp'] & 0xFF]
        elif self.commandDes == 'G':
            # make the grip control commands here
            dACI2 = [c['handOC'] >> 0x8, c['handOC'] & 0xFF, c['grip'] >> 0x8, c['grip'] & 0xFF, 0x00, 0x00, 0x00, 0x00]
        else:
            ValueError('sendCommand(): Incorrect command description %s' % self.commandDes)

        self.send(dACI1, dACI2, dACI3)

    def send(self, dACI1, dACI2, dACI3):
        # print(["%x" % i for i in dACI2])
        self.replies += 1
        # ACI1 = can.Message(timestamp=time.time(), arbitration_id=0x210, data=dACI1, is_extended_id=False)
        # ACI2 = can.Message(timestamp=time.time(), arbitration_id=0x211, data=dACI2, is_extended_id=False)
        # ACI3 = can.Message(timestamp=time.time(), arbitration_id=0x212, data=dACI3, is_extended_id=False)
        # self.bus.send(ACI1, timeout=None)
        # self.bus.send(ACI2, timeout=None)
        # self.bus.send(ACI3, timeout=None)
        # print(ACI1)
        # print(ACI2)
        # print(ACI3, "\n")

        # instead of sending directly, I will pass the commands to the communication process, which will send them
        commsPacked = bytearray(dACI1 + dACI2 + dACI3)
        with open(self.commPath, 'wb') as output:
            output.write(commsPacked)

    def initSensors(self):
        # send mode select as 0x000
        # wait > 500 ms (1 s to be safe)
        self.buildEmptyCommand() # initialize empty command
        # self.changeMode(0x000) # make sure mode select set as 0x000
        missingReadings = self.sensorIDs.copy()
        while (time.time() - self.startTimestamp < 1 or len(missingReadings) > 0):
            assert self.messagesReceived[0x080] == self.replies, "Missed reply"
            self.recv()
            self.messageCallback()
            
            # populate sensors
            if self.arbitration_id in missingReadings:
                missingReadings.remove(self.arbitration_id)

        self.printSensors()

    # def startup(self):
    #     # send mode select 0x3FF for 100 ms, then send 0x000
    #     self.enable()

        # hack to force startup
        # count = 0
        # while(True):
        #     self.recv()
        #     self.messageCallback()

        #     if not count % 300:
        #         self.shortModeSwitch(2)

        #     count += 1

        #     if self.modeRead == 2:
        #         break
        
    def startup(self):
        self.shortModeSwitch(2)

        if self.commandType == 'P' and self.commandDes == 'DF':
            # [thumbP, thumbY, index, mrp, wristRot, wristFlex, humRot, elbow]
            posCom = self.buildCurPosCommand()
            self.buildCommand(posCom=posCom)
        elif self.commandType == 'V' and self.commandDes == 'DF':
            # [thumbP, thumbY, index, mrp, wristRot, wristFlex, humRot, elbow]
            velCom = [0, 0, 0, 0, 0, 0, 0, 0]
            self.buildCommand(velCom=velCom)
        else:
            # [handOC, grip]
            gripCom = [0, 0]
            self.buildCommand(gripCom=gripCom)

    # def shortModeSwitch(self):
    #     self.buildEmptyCommand()
    #     self.changeMode(0x3FF)
    #     T = time.time()
    #     while (time.time() - T < 0.1):
    #         self.recv()
    #         if self.arbitration_id == self.syncID:
    #             self.syncAck()
    #         self.sendCommand()
    #         time.sleep(0.003)

    #     self.changeMode(0x000)
    #     self.sendCommand()

    #     if self.modeRead == 0: # initial power on
    #         pass
    #     elif self.modeRead == 1: # arm mode -> hand mode
    #         self.changeMode(2)
    #         # print("In hand mode")
    #     elif self.modeRead == 2: # hand mode -> arm mode
    #         self.changeMode(1)
    #         # print("In arm mode")
    #     else:
    #         ValueError('shortModeSwitch(): wrong mode for mode switch')

    def shortModeSwitch(self, newMode):
        # only do anything if arm mode is not correct?
        if self.modeRead != newMode:
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
        # elif self.modeRead == 1: # arm mode -> hand mode
        #     self.changeMode(2)
        #     # print("In hand mode")
        # elif self.modeRead == 2: # hand mode -> arm mode
        #     self.changeMode(1)
        #     # print("In arm mode")
        elif newMode in [1, 2, 3]:
            self.changeMode(newMode)
        else:
            ValueError('shortModeSwitch(): wrong mode for mode switch')

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

        self.changeMode(0) # put arm into standby
        print("In standby mode")

    def changeMode(self, modeSend):
        self.modeSend = modeSend
        self.command['modeSelect'] = modeSend # need to set this here, but I'm not happy about it

    def shutdown(self):
        self.longModeSwitch()
        # self.bus.shutdown()

    def mainControlLoop(self, emg=None, controller=None):
        try:
            count = 0
            while(True):
                # handle arm communication
                self.recv()
                self.messageCallback()

                if self.usingEMG:
                    # update EMG reading
                    emg.readEMG()
        
                    # now build control off this
                    emg.normEMG()
                    emg.muscleDynamics()
                    
                    posCom = controller.calculateEMGCommand()
                    print(posCom)

                else:
                    index = self.genSinusoid(3, 'indexPos')
                    mrp = self.genSinusoid(3, 'mrpPos')
                    thumbP = self.genSinusoid(3, 'thumbPPos')
                    wristR = self.genSinusoid(3, 'wristRot')
                    thumbY = self.genSinusoid(3, 'thumbYPos')
                    elbow = self.genSinusoid(10, 'elbowPos')
                    # humRot = self.genSinusoid(100, 'humPos')
                    # print(com)
                    # [thumbPPos, thumbYPos, indexPos, mrpPos, wristRot, wristFlex, humPos, elbowPos]
                    posCom = [thumbP, thumbY, index, mrp, wristR, self.sensors['wristFlex'], self.sensors['humPos'], elbow]

                if count == 2000:
                    print("Starting movement")
                if count > 2000:
                    self.buildCommand(posCom=posCom)

                count += 1

        except KeyboardInterrupt:
            print("\nProgram Exited")

        except can.CanError:
            print("\nCAN Error")


    ## FOR TESTING BELOW
    def genSinusoid(self, duration, joint):
        rom = self.jointRoM[joint]
        return (rom[1] - rom[0])*(0.4*math.sin(2*math.pi*(self.startTimestamp - self.timestamp)/duration) + 0.5)

###################################################################
def callback():
    run = ""
    # while run not in ["standby", "arm", "hand", "startup", "exit"]:
    #     run = input("Enter 'startup' to enable the arm.\nEnter 'standby' to put the arm in standby mode.\nEnter 'arm' to put the arm in arm mode.\nEnter 'hand' to put the arm in hand mode.\nEnter 'exit' to put the arm in standby mode and quit:\n")
    while run not in ["standby", "arm", "hand", "simul", "startup", "exit"]:
        run = input("\nEnter 'startup' to enable the arm.\nEnter 'standby' to put the arm in standby mode.\nEnter 'arm' to switch to arm mode.\nEnter 'hand' to switch to hand mode.\nEnter 'simul' to switch to simultaneous mode.\nEnter 'exit' to put the arm in standby mode and quit:\n")

    return run

def main(usingEMG, usingLogging):
    # instantiate arm class
    arm = LUKEArm(config='HC', hand='L', commandDes='DF', commandType='P', usingEMG=usingEMG)

    # connect to EMG board
    if usingEMG:
        print("Connecting to EMG board...")
        emg = EMG()
        print("Connected.")
        emg.readEMG() # need to get first signal to avoid errors

        # setup second order dynamics
        controller = impedanceController(arm.numMotors, 3, LUKEArm, emg)

    print("Initializing sensor readings...")
    arm.initSensors()
    print("Sensors initialized.")

    # set up case/switch for arm using an input/callback structure -- this allows the behavior of the arm to be controlled without stopping this code
    movementModes = ["arm", "hand", "simul"]
    try:
        while(1):
            run = callback()

            print(f"Original mode read: {arm.modeRead}")
            print(f"Original mode send: {arm.modeSend}")

            if run == "startup":
                print("\n\nGoing through startup...")
                arm.startup()
                print("Arm enabled\n\n")

            elif run in movementModes:
                print(f"\n\nSwitching to {run} mode...")
                arm.shortModeSwitch(movementModes.index(run) + 1)

                if usingEMG:
                    arm.mainControlLoop(emg, controller)
                else:
                    arm.mainControlLoop()

                print("\n\n")

            elif run == "standby":
                print("\n\nSwitching to standby mode...")
                arm.shutdown()
                print("\n\n")
            
            elif run == "exit":
                break

            else:
                print("Invalid command.")
            
            print(f"New mode send: {arm.modeSend}")
            print(f"New mode read: {arm.modeRead}")

    except KeyboardInterrupt:
        print("\n\nExiting.")

    arm.shutdown()
    print("Shutting down.")

if __name__ == '__main__':
    usingEMG = False
    usingLogging = False

    if len(sys.argv) == 1:
        print("Starting LUKEArm.py (no EMG, no logging)...\n")
    elif len(sys.argv) == 2:
        try:
            isNum = int(sys.argv[1])
            
            print("Starting LUKEArm.py (EMG, no logging)...\n")
            usingEMG = True
        except:
            print("Starting LUKEArm.py (no EMG, logging)...\n")
            usingLogging = True
    elif len(sys.argv) == 3:
        print("Starting LUKEArm.py (EMG, logging)...\n")
        usingEMG = True
        usingLogging = True
    else:
        ValueError("Wrong number of arguments (%i)" % len(sys.argv) - 1)

    main(usingEMG, usingLogging)