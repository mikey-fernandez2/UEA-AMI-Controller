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
import zmq

class LUKEArm:
    def __init__(self, config='HC', hand='L', commandDes='DF', commandType='P', socketAddr="tcp://127.0.0.1:1234", usingEMG=False):        
        # make sure valid inputs given
        assert config in ['HC', 'RC'], f'Invalid arm config {config} given'
        assert hand in ['L', 'R'], f'Invalid handedness {hand} given'
        assert commandDes in ['DF', 'G'], f'Invalid command description {commandDes} given'
        assert commandType in ['P', 'V'], f'Invalid control type {commandType} given'
        
        # setup arm state
        self.config = config
        self.hand = hand
        self.commandDes = commandDes
        self.commandType = commandType
        self.usingEMG = usingEMG

        # for IPC use a zmq socket
        self.socketAddr = socketAddr
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.bind(self.socketAddr)

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
        self.firstMessage = True

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
                        'wristRot': 0x000, 'wristFlex': 0x000, 'humPos': 0x000, 'elbow': 0x000}

        # CAN controller settings
        self.ACICountsPerDegree = {'wristRot': 2.90, 'wristFlex': 8.47, 'indexPos': 10.36, 'mrpPos': 10.36, 'thumbPPos': 9.32, 'thumbYPos': 12.43, 'humPos': 5, 'elbowPos': 6.84}
        self.zeroPos = {'indexPos': 50, 'mrpPos': 50, 'thumbYPos': 50, 'thumbPPos': 50, 'wristFlex': 511, 'wristRot': 511, 'humPos': 511, 'elbowPos': 50} # the ACI command for a position of 0
        self.zeroVel = 512 # the ACI command for a velocity of 0

        # joint limits
        if self.hand == "R":
            self.jointRoM = {'indexPos': [0, 90], 'mrpPos': [0, 90], 'thumbYPos': [0, 75], 'thumbPPos': [0, 75], 'wristFlex': [-55, 55], 'wristRot': [-120, 175], 'elbowPos': [0, 135], 'humPos': [-95, 95]}
        elif self.hand == "L":
            self.jointRoM = {'indexPos': [0, 90], 'mrpPos': [0, 90], 'thumbYPos': [0, 75], 'thumbPPos': [0, 75], 'wristFlex': [-55, 55], 'wristRot': [-175, 120], 'elbowPos': [0, 135], 'humPos': [-95, 95]}
        else:
            raise ValueError(f"LUKEArm(): invalid handedness {self.hand}")

        # command structure
        self.lastposCom = None
        self.lastvelCom = None
        self.lastgripCom = None

    def __del__(self):
        """ Garbage collection """
        try:
            self.bus.shutdown()
        except:
            print("__del__: Bus shutdown error")

        try:
            self.sock.unbind(self.socketAddr)
            self.sock.close()
            self.ctx.term()
        except:
            print("__del__: Socket closing error")

    def recv(self):
        message = self.bus.recv()
        self.arbitration_id = message.arbitration_id
        self.data = message.data
        self.timestamp = message.timestamp
        self.message = message

        if self.firstMessage:
            self.startTimestamp = self.timestamp
            self.firstMessage = False

    def printSensors(self):
        s = self.sensors # to save me from typing
        print(f"\nRunning time: {self.timestamp - self.startTimestamp:f}")
        print(f"\tCurrent grip: {self.gripRead} | current mode: {self.modeRead}")
        print("\tJoint positions:")
        print(f"\t\t  Hum rot: {s['humPos']:8.3f} |   Elbow pos: {s['elbowPos']:8.3f}")
        print(f"\t\tWrist rot: {s['wristRot']:8.3f} |  wrist flex: {s['wristFlex']:8.3f}")
        print(f"\t\tThumb yaw: {s['thumbYPos']:8.3f} | thumb pitch: {s['thumbPPos']:8.3f}")
        print(f"\t\tIndex pos: {s['indexPos']:8.3f} |     MRP pos: {s['mrpPos']:8.3f}")
        print("\n\tForce sensors (status):")
        print(f"\t\t  Index Lat: {s['indLatF']:8.3f} ({s['indLatStat']}) | index tip: {s['indTipF']:8.3f} ({s['indTipStat']}) |   mid tip: {s['midTipF']:8.3f} ({s['midTipStat']}) |   ring tip: {s['ringTipF']:8.3f} ({s['ringTipStat']}) | pinky tip: {s['pinkTipF']:8.3f} ({s['pinkTipStat']})")
        print(f"\t\t  Palm dist: {s['palmDistF']:8.3f} ({s['palmDistStat']}) | palm prox: {s['palmProxF']:8.3f} ({s['palmProxStat']}) | hand edge: {s['handEdgeF']:8.3f} ({s['handEdgeStat']}) |  hand dors: {s['handDorsF']:8.3f} ({s['handDorsStat']})")
        print(f"\t\tThumb ulnar: {s['thumbUlF']:8.3f} ({s['thumbUlStat']}) | thumb rad: {s['thumbRaF']:8.3f} ({s['thumbRaStat']}) | thumb tip: {s['thumbTipF']:8.3f} ({s['thumbTipStat']}) | thumb dors: {s['thumbDorsF']:8.3f} ({s['thumbDorsStat']})")
        print()

    def printCurPos(self):
        s = self.sensors # decrease typing
        print(f"Joint positions: [{s['thumbPPos']:8.3f}, {s['thumbYPos']:8.3f}, {s['indexPos']:8.3f}, {s['mrpPos']:8.3f}, {s['wristRot']:8.3f}, {s['wristFlex']:8.3f}, {s['humPos']:8.3f}, {s['elbowPos']:8.3f}]")

    def printCANPos(self):
        s = self.sensors # save me from typing
        for joint in self.jointRoM.keys():
            print(f"{joint}: {self.posToCAN(s[joint], joint):03x}")
        print()

    def printCommand(self):
        c = self.command # to save me from typing
        print(f"\nMode Select: {c['modeSelect']}")
        print("Joint positions:")
        print(f"\t  Hum rot: {c['humPos']:03x} |   Elbow pos: {c['elbow']:03x}")
        print(f"\tWrist rot: {c['wristRot']:03x} |  wrist flex: {c['wristFlex']:03x}")
        print(f"\tThumb yaw: {c['thumbY']:03x} | thumb pitch: {c['thumbP']:03x}")
        print(f"\tIndex pos: {c['index']:03x} |     MRP pos: {c['mrp']:03x}")
        print("Hand Mode:")
        print(f"\tHand OC: {c['handOC']} | Grip: {c['grip']}")
        print()

    def posToCAN(self, pos, joint):
        # convert a position command for a joint to its CAN command
        # need to convert the desired position in degrees to the number of degrees from the bottom of the range then scale by the ACI counts and add the zero offset
        canCom = math.floor(self.ACICountsPerDegree[joint]*pos) + self.zeroPos[joint]
        return canCom

    def CANtoPos(self, CAN):
        # convert a 2 byte CAN signal to the corresponding joint position
        fullCAN = (CAN[0] << 0x8) + CAN[1]
        scaledCAN = fullCAN/2**6
        pos = scaledCAN if scaledCAN <= 180 else scaledCAN - 1024    
        return pos

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
                self.sensors['wristRot'] = self.CANtoPos(data[0:2])
                self.sensors['wristFlex'] = self.CANtoPos(data[2:4])
                self.sensors['indexPos'] = self.CANtoPos(data[4:6])
                self.sensors['mrpPos'] = self.CANtoPos(data[6:8])
            elif msg_id == 0x4BF:
                self.sensors['thumbPPos'] = self.CANtoPos(data[0:2])
                self.sensors['thumbYPos'] = self.CANtoPos(data[2:4])
            elif msg_id == 0x1A0:
                self.sensors['humPos'] = self.CANtoPos(data[4:6])
            elif msg_id == 0x1A4:
                self.sensors['elbowPos'] = self.CANtoPos(data[4:6])
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
                raise ValueError(f'messageCallback(): Invalid sensor message arbitration_id {msg_id:03X}')

    def passiveUpdate(self, duration):
        # stupid little function to passively update sensors
        T = time.time()
        while time.time() - T < duration:
            self.recv()
            self.messageCallback()

    def isValidCommand(self):
        # returns true if all commands in the list are between 0x000 and 0x3FF (0 to 1023)
        commandCAN = self.command.values()
        vals = [(com >= 0x000 and com <= 0x3FF) for com in commandCAN]
        return all(vals)

    def getCurPos(self):
        return [self.sensors['thumbPPos'], self.sensors['thumbYPos'], self.sensors['indexPos'], self.sensors['mrpPos'], self.sensors['wristRot'], self.sensors['wristFlex'], self.sensors['humPos'], self.sensors['elbowPos']]

    def buildEmptyCommand(self):
        for i in self.command.keys():
            self.command[i] = 0x000

    def buildCommand(self, posCom=None, velCom=None, gripCom=None):
        if posCom != None and velCom == None and gripCom == None:
            # build the position commands here
            # CONTRACT: these positions will all be valid in the RoM for the given joint
            self.command['thumbP'] = self.posToCAN(posCom[0], 'thumbPPos')
            self.command['thumbY'] = self.posToCAN(posCom[1], 'thumbYPos')
            self.command['index'] = self.posToCAN(posCom[2], 'indexPos')
            self.command['mrp'] = self.posToCAN(posCom[3], 'mrpPos')

            self.command['wristRot'] = self.posToCAN(posCom[4], 'wristRot')
            self.command['wristFlex'] = self.posToCAN(posCom[5], 'wristFlex')
            self.command['humPos'] = self.posToCAN(posCom[6], 'humPos')
            self.command['elbow'] = self.posToCAN(posCom[7], 'elbowPos')

            self.lastposCom = posCom

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
            self.command['humPos'] = math.floor(maxVel*velCom[6]) + self.zeroVel
            self.command['elbow'] = math.floor(maxVel*velCom[7]) + self.zeroVel

            self.lastvelCom = velCom

        elif posCom == None and velCom == None and gripCom != None:
            # build the grip commands here
            # CONTRACT: TODO
            self.command['handOC'] = gripCom[0]
            self.command['grip'] = gripCom[1]

            self.lastgripCom = gripCom

        else:
            raise ValueError('buildCommand(): command type incorrect')

        if not self.isValidCommand():
            raise ValueError(f'buildCommand(): invalid command {self.command}')

    def sendCommand(self):
        if not self.isValidCommand():
            raise ValueError(f'sendCommand(): invalid command {self.command}')

        c = self.command
        dACI1 = [c['modeSelect'] >> 0x8, c['modeSelect'] & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        dACI3 = [c['wristRot'] >> 0x8, c['wristRot'] & 0xFF, c['wristFlex'] >> 0x8, c['wristFlex'] & 0xFF, c['humPos'] >> 0x8, c['humPos'] & 0xFF, c['elbow'] >> 0x8, c['elbow'] & 0xFF]

        if self.commandDes == 'DF':
            # make the direct finger control commands here
            dACI2 = [c['thumbP'] >> 0x8, c['thumbP'] & 0xFF, c['thumbY'] >> 0x8, c['thumbY'] & 0xFF, c['index'] >> 0x8, c['index'] & 0xFF, c['mrp'] >> 0x8, c['mrp'] & 0xFF]
        elif self.commandDes == 'G':
            # make the grip control commands here
            dACI2 = [c['handOC'] >> 0x8, c['handOC'] & 0xFF, c['grip'] >> 0x8, c['grip'] & 0xFF, 0x00, 0x00, 0x00, 0x00]
        else:
            raise ValueError('sendCommand(): Incorrect command description %s' % self.commandDes)

        self.send(dACI1, dACI2, dACI3)

    def send(self, dACI1, dACI2, dACI3):
        # instead of sending directly, I will pass the commands to the communication process, which will send them
        commsPacked = bytearray(dACI1 + dACI2 + dACI3)
        self.sock.send(commsPacked)

    def initSensors(self):
        # send mode select as 0x000
        # wait > 500 ms (1 s to be safe)
        self.buildEmptyCommand() # initialize empty command
        missingReadings = self.sensorIDs.copy()
        while (time.time() - self.startTimestamp < 1 or len(missingReadings) > 0):
            self.recv()
            self.messageCallback()
            
            # populate sensors
            if self.arbitration_id in missingReadings:
                missingReadings.remove(self.arbitration_id)

        self.printSensors()
        
    def startup(self):
        self.shortModeSwitch(2)

        if self.commandType == 'P' and self.commandDes == 'DF':
            # [thumbP, thumbY, index, mrp, wristRot, wristFlex, humPos, elbow]
            posCom = self.getCurPos()
            self.buildCommand(posCom=posCom)
        elif self.commandType == 'V' and self.commandDes == 'DF':
            # [thumbP, thumbY, index, mrp, wristRot, wristFlex, humPos, elbow]
            velCom = [0, 0, 0, 0, 0, 0, 0, 0]
            self.buildCommand(velCom=velCom)
        else:
            # [handOC, grip]
            gripCom = [0, 0]
            self.buildCommand(gripCom=gripCom)

    def shortModeSwitch(self, newMode):
        # only do anything if arm mode is not correct?
        while self.modeSend != newMode and self.modeRead != newMode:
            self.buildEmptyCommand()
            self.changeMode(0x3FF)
            T = time.time()
            while (time.time() - T < 0.1):
                self.recv()
                self.messageCallback()

                self.sendCommand()
                time.sleep(0.007)

            self.changeMode(0x000)
            self.sendCommand()

            if newMode in [1, 2, 3]:
                self.changeMode(newMode)
            else:
                raise ValueError(f'shortModeSwitch(): wrong mode {newMode} for mode switch')

    def longModeSwitch(self):
        while self.modeRead != 0:
            self.buildEmptyCommand()
            self.changeMode(0x3FF)
            T = time.time()
            while (time.time() - T < 1):
                self.recv()
                self.messageCallback()
    
                self.sendCommand()
                time.sleep(0.007)
    
            self.changeMode(0x000)
            self.sendCommand()

        print("In standby mode")

    def changeMode(self, modeSend):
        self.modeSend = modeSend
        self.command['modeSelect'] = modeSend # need to set this here, but I'm not happy about it

    def shutdown(self):
        self.longModeSwitch()

    def mainControlLoop(self, emg=None, controller=None):
        try:
            count = 0
            while(True):
                # handle arm communication
                self.recv()
                self.messageCallback()

                if self.usingEMG:                   
                    # posCom = controller.calculateEMGCommand()
                    diffCom = controller.differentialActCommand(threshold=0.01, gain=0.001)
                    posCom = self.getCurPos()
                    # try with index first - this is elemenet 2 of the lists
                    posCom[2] = diffCom[2]

                else:
                    # thumbP = self.sensors['thumbPPos']
                    thumbP = self.genSinusoid(3, 'thumbPPos')
                    # thumbY = self.sensors['thumbYPos']
                    thumbY = self.genSinusoid(3, 'thumbYPos')
                    # index = self.sensors['indexPos']
                    index = self.genSinusoid(4, 'indexPos')
                    # mrp = self.sensors['mrpPos']
                    mrp = self.genSinusoid(4, 'mrpPos')
                    # wristRot = self.sensors['wristRot']
                    wristRot = self.genSinusoid(6, 'wristRot')
                    # wristFlex = self.sensors['wristFlex']
                    wristFlex = self.genSinusoid(6, 'wristFlex')
                    # humPos = self.sensors['humPos']
                    humPos = self.genSinusoid(8, 'humPos')
                    # elbow = self.sensors['elbowPos']
                    elbow = self.genSinusoid(10, 'elbowPos')

                    # [thumbPPos, thumbYPos, indexPos, mrpPos, wristRot, wristFlex, humPos, elbowPos]
                    posCom = [thumbP, thumbY, index, mrp, wristRot, wristFlex, humPos, elbow]

                # wait a second or two to start moving for safety
                if count == 2000:
                    print("Starting movement")
                if count > 2000:
                    self.buildCommand(posCom=posCom)
                    if not count % 1000:
                        self.printSensors()

                count += 1

        except KeyboardInterrupt:
            print("\nControl ended.")

        except can.CanError:
            print("\nCAN Error")

    def goToZeroPos(self, period):
        self.shortModeSwitch(1) # make sure to switch to arm mode first!

        try:
            startPos = self.getCurPos()
            self.printCurPos()
            start = time.time()
            elapsedTime = 0
            while(elapsedTime < period):
                # handle arm communication
                self.recv()
                self.messageCallback()

                # [thumbPPos, thumbYPos, indexPos, mrpPos, wristRot, wristFlex, humPos, elbowPos]
                posCom = [(1 - elapsedTime/period)*pos for pos in startPos]

                self.buildCommand(posCom=posCom)

                # update time
                elapsedTime = time.time() - start

        except can.CanError:
            print("CAN Error")

        self.printCurPos()

        print("At zero position. Ending this movement.")

    def genSinusoid(self, period, joint):
        rom = self.jointRoM[joint]
        return 0.5*(rom[1] - rom[0])*(math.sin(2*math.pi*(self.startTimestamp - self.timestamp)/period) + 1) + rom[0]

###################################################################
def callback():
    run = ""
    while run not in ["standby", "arm", "hand", "simul", "startup", "zero", "exit"]:
        run = input("\nEnter 'startup' to enable the arm.\nEnter 'standby' to put the arm in standby mode.\nEnter 'arm' to switch to arm mode.\nEnter 'hand' to switch to hand mode.\nEnter 'simul' to switch to simultaneous mode.\nEnte 'zero' to return the arm to joint positions of 0.\nEnter 'exit' to put the arm in standby mode and quit:\n")

    return run

def main(usingEMG, usingLogging):
    # instantiate arm class
    arm = LUKEArm(config='HC', hand='L', commandDes='DF', commandType='P', socketAddr="tcp://127.0.0.1:1234", usingEMG=usingEMG)

    # connect to EMG board
    if usingEMG:
        print("Connecting to EMG board...")
        emg = EMG()
        print("Connected.")
        emg.readEMG() # need to get first signal to avoid errors

        # setup the controller class
        controller = impedanceController(numMotors=arm.numMotors, freq_n=3, LUKEArm=arm, emg=emg)

    print("Initializing sensor readings...")
    arm.initSensors()
    print("Sensors initialized.")

    # set up case/switch for arm using an input/callback structure -- this allows the behavior of the arm to be controlled without stopping this code
    movementModes = ["arm", "hand", "simul"]
    try:
        while True:
            run = callback()

            if run == "startup":
                print("\n\nGoing through startup...")
                arm.startup()
                arm.passiveUpdate(0.1)

            elif run in movementModes:
                # this is dumb but I have to make sure that you're in hand mode when you want to switch to arm from standby
                if (arm.modeSend in [-1, 0]):
                    print(f"\nInvalid command {run} from standby mode - run startup first")
                    continue

                print(f"\n\nSwitching to {run} mode...")
                arm.shortModeSwitch(movementModes.index(run) + 1)
                arm.mainControlLoop(emg, controller) if usingEMG else arm.mainControlLoop()

            elif run == "standby":
                print("\n\nSwitching to standby mode...")
                arm.shutdown()

            elif run == "zero":
                print("\n\nZeroing joints...")
                arm.goToZeroPos(5)
            
            elif run == "exit":
                break

            else:
                print("Invalid command.")

    except KeyboardInterrupt:
        print("Exiting.")

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
        raise ValueError(f"Wrong number of arguments ({len(sys.argv) - 1})")

    main(usingEMG, usingLogging)