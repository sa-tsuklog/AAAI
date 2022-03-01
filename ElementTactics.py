'''
Created on 2022/01/16

@author: sa
'''
from .MyUtil import *
from .Maneuver import *
from enum import IntEnum, auto
from .FireControl import *
from .IntegratedFireControl import *


ALT_AT_FIRST_SHOT = -15000
ALT_AT_IN = -9000
ALT_AT_HORIZONTAL_IN = -4000
ALT_AT_GUIDANCE = -3000
ALT_AT_OUT = -1000

M_FIRST_SHOT_RANGE_MAX = 100000
M_FIRST_SHOT_RANGE_MIN =  75000

M_LAUNCH_RANGE_HOT = 50000
M_SHOTBACK_RANGE = 40000
M_LAUNCH_RANGE_MWS = 40000
M_LAUNCH_RANGE_COLD = 17000
M_LAUNCH_RANGE_BACKWARD = 17000

M_CROSSFIRE_RANGE = 20000
M_SECONDARY_TARGET_LAUNCH_RANGE = 25000
RAD_CROSSFIRE_ANGLE_LIMIT = 60/180 * np.pi

MPS_MISSILE_ALIVE_CRITERIA = 400
MPS_MISSILE_ALIVE_CRITERIA_MWS = 200

S_OUT_KEEP_TIME = 15
S_MINIMUM_CRANKING_TIME = 45

# Coldターゲットへの接近方法。位置先読み
# S_COLD_TARGET_LEAD_AHEAD_TIME = 60    
# COLD_TARGET_OFFSET_RATIO = 0
# Coldターゲットへの接近方法。同じライン上の自陣側の位置 
S_COLD_TARGET_LEAD_AHEAD_TIME = 60
COLD_TARGET_OFFSET_RATIO = 0.5

M_ALT_RECOVER_CRITERIA = -1700
MPS_VEL_RECOVER_CRITERIA = 300
M_VERTICAL_IN_CRITERIA = -2000

RAD_CLIMB_ANGLE_LIMIT_NORMAL = 25/180*np.pi
RAD_DIVE_ANGLE_LIMIT_NORMAL = -30/180*np.pi #CLIMB_ANGLEより大きい角度であること。

RAD_CLIMB_ANGLE_LIMIT_MWS = 25/180*np.pi
RAD_DIVE_ANGLE_LIMIT_MWS = -45/180*np.pi #CLIMB_ANGLEより大きい角度であること。

PITCH_GAIN_NORMAL = (1/(90 /180 * np.pi))
PITCH_LIMIT_NORMAL =0.4

PITCH_GAIN_MWS = (1/(90 /180 * np.pi))
PITCH_LIMIT_MWS = 0.7

MPS_BANZAI_PUSH_SPEED = 75

class STATE(IntEnum):
    LAUNCH_AND_OUT_FROM_CRITICAL_RANGE = auto()
    OUT_FROM_MWS = auto()
    OUT_FROM_CRITICAL_RANGE = auto()
    OUT_KEEP = auto()
    EM_RECOVER1 = auto()
    EM_RECOVER2 = auto()
    HORIZONTAL_IN = auto()
    APPROACH_TARGET = auto()
    CRANKING = auto()
    IN = auto()
    PENETRATING = auto()
    
class TACTICS(IntEnum):
    CLIMB_AND_LAUNCH_FIRST = auto()
    ONE_ON_ONE_DEFENCE = auto()
    ESCORT_AND_PENETRATION = auto()
    BANZAI = auto()

class ElementTactics():
    def __init__(self):
        self.isAlive = [True,True]
        self.states = [STATE.IN,STATE.IN]
        self.primaryTargets = [None,None]
        self.fireControl = IntegratedFireControl()
        self.outFromCriticalRangeStartTimes = [-S_OUT_KEEP_TIME-1,-S_OUT_KEEP_TIME-1]
        self.outCountdowns = [np.inf,np.inf]
        self.descendStartCountdowns = [np.inf,np.inf]
        self.descendEndCountdowns = [0,0]
        self.crankingStartTime = [0.0,0.0]
        self.snakingStartTime = [-1,-1]
        self.snakingPolarity = [1,1]
        self.lastAspectOfTargets = [[0,0],[0,0]]
        self.isTargetsHot = [[True,True],[True,True]]
        self.util = MyUtil()
        self.tactics = TACTICS.CLIMB_AND_LAUNCH_FIRST
    
    def _calcAspectOfTargets(self,observableList):
        aspectOfTargets = [[0,0],[0,0]]
        for port in range(2):
            if(not observableList[port] is None):
                myPos = observableList[port]["motion"]["pos"]
                for i,val in enumerate(self.fireControl.trackFile.values()):
                    if(not val.track3D is None):
                        radAspectAngle = calcRadAspectAngle3D(myPos, val.track3D["pos"], val.track3D["vel"])
                        aspectOfTargets[port][i] = radAspectAngle
                
        return aspectOfTargets
                
    def RuleBasedDoctrine(self,time,observableList,names,primaryAxis=(0,1,0)):
        self.fireControl.updateTrack(time,observableList)
        primaryTargets = self.fireControl.assignPrimaryTarget(observableList,primaryAxis)
        aspectOfTargets = self._calcAspectOfTargets(observableList)
        
        targetTurnedCold = [[False,False],[False,False]]
        for port in range(2):
            for targetNum in range(2):
                if(self.isTargetsHot[port][targetNum] and aspectOfTargets[port][targetNum] < 80/180*np.pi and self.lastAspectOfTargets[port][targetNum] > 80/180*np.pi):
                    self.isTargetsHot[port][targetNum] = False
                    targetTurnedCold[port][targetNum] = True
                    #self.util.printLog(time, names[port], "aspect of target {:} turned cold".format(port,targetNum))
                if((not self.isTargetsHot[port][targetNum]) and aspectOfTargets[port][targetNum] > 90/180*np.pi and self.lastAspectOfTargets[port][targetNum] < 90/180*np.pi):
                    self.isTargetsHot[port][targetNum] = True
                    #self.util.printLog(time, names[port], "aspect of target {:} turned hot".format(port,targetNum))
        
        for port in range(2):
            if(observableList[port] is None):
                continue
            
            if(not primaryTargets[port] is None):
                primaryTrack = self.fireControl.getTrack(primaryTargets[port]).track3D
            else:
                primaryTrack = None
            
            secondaryTarget = self.fireControl.getAnotherLiveTruth(primaryTargets[port])
            if(not secondaryTarget is None):
                secondaryTrack = self.fireControl.getTrack(secondaryTarget).track3D
            else:
                secondaryTrack = None
            
            outCountdown,countdownUpdated,countdownUpdateSource,countdownDistance = updateOutCountdownForAllTrack(self.outCountdowns[port], observableList[port]["motion"]["pos"], primaryTrack,secondaryTrack)
            self.outCountdowns[port] = outCountdown
            if(countdownUpdated):
                self.util.printLog(time, names[port], " OutCountdown Updated to {:.1f}, source: {:} at {:.0f} m".format(outCountdown,countdownUpdateSource,countdownDistance))
        
        blueFrontline = 100000
        redFrontline = -100000
        
        for port in range(2):
            if(not observableList[port] is None):
                posY = float(observableList[port]["motion"]["pos"][1]) * -primaryAxis[1]
                if(posY < blueFrontline):
                    blueFrontline = posY
        for key,val in self.fireControl.trackFile.items():
            if(val.isAlive):
                posY = float(val.lastFoundPos[1]) * -primaryAxis[1]
                if(posY > redFrontline):
                    redFrontline = posY
        frontline = (blueFrontline + redFrontline)/2
        
        if(self.tactics == TACTICS.CLIMB_AND_LAUNCH_FIRST):
            commands,tacticsChangeRequest = self.climbAndLaunchFirst(time,observableList,names,primaryTargets,primaryAxis=primaryAxis)
            
            if(tacticsChangeRequest):
                if(self.fireControl.getNumTargetsFound() == 2):
                    nextTactics = TACTICS.ONE_ON_ONE_DEFENCE
                else:
                    nextTactics = TACTICS.ESCORT_AND_PENETRATION
                # nextTactics = TACTICS.BANZAI
            else:
                nextTactics = TACTICS.CLIMB_AND_LAUNCH_FIRST
            
        elif(self.tactics == TACTICS.ONE_ON_ONE_DEFENCE):
            if(redFrontline > 70000):
                nextTactics = TACTICS.BANZAI
            elif(frontline > 0 and frontline > (1000-float(time)) * MPS_BANZAI_PUSH_SPEED and self.fireControl.getTargetsRemain() == 2):
                nextTactics = TACTICS.BANZAI
            else:
                nextTactics = TACTICS.ONE_ON_ONE_DEFENCE
            
            commands = self.oneOnOneDefence(time,observableList,names,primaryTargets,primaryAxis=primaryAxis)
        
        elif(self.tactics == TACTICS.ESCORT_AND_PENETRATION):
            if(self.fireControl.getNumTargetsFound() == 2):
                nextTactics = TACTICS.ONE_ON_ONE_DEFENCE
            else:
                nextTactics = TACTICS.ESCORT_AND_PENETRATION
            
            commands = self.escortAndPenetration(time, observableList, names, primaryTargets, primaryAxis=primaryAxis)
        elif(self.tactics == TACTICS.BANZAI):
            if(frontline < -10000 or (self.fireControl.getTargetsRemain() == 1 and (not observableList[0] is None) and (not observableList[1] is None))):
                nextTactics = TACTICS.ONE_ON_ONE_DEFENCE
                # nextTactics = TACTICS.BANZAI
            else:
                nextTactics = TACTICS.BANZAI
            
            commands = self.banzai(time, observableList, names, primaryTargets, primaryAxis=primaryAxis)
        
        
        if(nextTactics != self.tactics):
            print("----------------------------------")
            self.util.printLog(time, names[0] ,"Tactics"+str(self.tactics)+"->"+str(nextTactics))
            print("----------------------------------")
        self.tactics = nextTactics
        
        for port in range(2):
            for targetNum in range(2):
                self.lastAspectOfTargets[port][targetNum] = aspectOfTargets[port][targetNum]
        
        return commands
    
    def climbAndLaunchFirst(self,time,observableList,names,primaryTargets,primaryAxis=(0,1,0)):
        mNsOffsets = [20000,-20000]
        commands = [None,None]
        
        numTargetsFound = self.fireControl.getNumTargetsFound()
        
        blue1Pos = observableList[0]["motion"]["pos"]
        blue2Pos = observableList[1]["motion"]["pos"]
        
        if(float(blue1Pos[0]) > float(blue2Pos[0])):
            northArmIndex = 0
            southArmIndex = 1
        else:
            northArmIndex = 1
            southArmIndex = 0
        
        if(not primaryTargets[0] is None):
            primaryTrack0 = self.fireControl.getTrack(primaryTargets[0]).track3D
            distanceToPrimaryTrack0 = calcDistance3d(blue1Pos,primaryTrack0["pos"])
        else: 
            distanceToPrimaryTrack0 = np.inf
            
        if(not primaryTargets[1] is None):
            primaryTrack1 = self.fireControl.getTrack(primaryTargets[1]).track3D
            distanceToPrimaryTrack1 = calcDistance3d(blue2Pos,primaryTrack1["pos"])
        else:
            distanceToPrimaryTrack1 = np.inf
        
        
        tacticsChangeRequest = False
        launches = [False,False]
        targets = [Track3D().to_json(),Track3D().to_json()]
        
        
        if(distanceToPrimaryTrack0 < M_FIRST_SHOT_RANGE_MIN):
            tacticsChangeRequest = True
            if(distanceToPrimaryTrack1 < M_FIRST_SHOT_RANGE_MAX):
                if(str(primaryTrack0["truth"]) != str(primaryTrack1["truth"])):
                    launches[0],targets[0] = self.fireControl.requestFire(time, observableList, 0, primaryTrack0)
                    launches[1],targets[1] = self.fireControl.requestFire(time, observableList, 1, primaryTrack1)
                else:
                    if(distanceToPrimaryTrack0 < distanceToPrimaryTrack1):
                        launches[0],targets[0] = self.fireControl.requestFire(time, observableList, 0, primaryTrack0)
                    else:
                        launches[1],targets[1] = self.fireControl.requestFire(time, observableList, 1, primaryTrack1)
            else:
                launches[0],targets[0] = self.fireControl.requestFire(time, observableList, 0, primaryTrack0)
            
        if(distanceToPrimaryTrack1 < M_FIRST_SHOT_RANGE_MIN):
            tacticsChangeRequest = True
            if(distanceToPrimaryTrack0 < M_FIRST_SHOT_RANGE_MAX):
                if(str(primaryTrack0["truth"]) != str(primaryTrack1["truth"])):
                    launches[0],targets[0] = self.fireControl.requestFire(time, observableList, 0, primaryTrack0)
                    launches[1],targets[1] = self.fireControl.requestFire(time, observableList, 1, primaryTrack1)
                else:
                    if(distanceToPrimaryTrack0 < distanceToPrimaryTrack1):
                        launches[0],targets[0] = self.fireControl.requestFire(time, observableList, 0, primaryTrack0)
                    else:
                        launches[1],targets[1] = self.fireControl.requestFire(time, observableList, 1, primaryTrack1)
            else:
                launches[1],targets[1] = self.fireControl.requestFire(time, observableList, 1, primaryTrack1)
        
        nextStates = [STATE.IN,STATE.IN]
        if(tacticsChangeRequest):
            nextStates[0] = STATE.CRANKING
            nextStates[1] = STATE.CRANKING
            self.crankingStartTime[0] = float(time)
            self.crankingStartTime[1] = float(time)
        elif(numTargetsFound == 0):
            nextStates[0] = STATE.IN
            nextStates[1] = STATE.IN
        elif(numTargetsFound == 1):
            targetTrack = list(self.fireControl.trackFile.values())[0]
            blue1ToTargetDistance = calcDistance3d(blue1Pos,targetTrack.lastFoundPos)
            blue2ToTargetDistance = calcDistance3d(blue2Pos,targetTrack.lastFoundPos)
            
            if(blue1ToTargetDistance < blue2ToTargetDistance):
                nextStates[0] = STATE.APPROACH_TARGET
                nextStates[1] = STATE.IN
            else:
                nextStates[0] = STATE.IN
                nextStates[1] = STATE.APPROACH_TARGET
            
        elif(numTargetsFound == 2):
            nextStates[0] = STATE.APPROACH_TARGET
            nextStates[1] = STATE.APPROACH_TARGET
        
        
        
        for port in range(2):
            if(observableList[port] is None):
                if(self.isAlive[port]):
                    self.isAlive[port] = False
                    print("{} : {} Down ".format(timeToStr(time),names[port]))
                continue
            
            obs = observableList[port]
            
            myPos = obs["motion"]["pos"]
            myVel = obs["motion"]["vel"]
            
            if(not primaryTargets[port] is None):
                primaryTrack = self.fireControl.getTrack(primaryTargets[port]).track3D
            else:
                primaryTrack = None
            
            if(not primaryTargets[port] is None):
                primaryTargetFoundInThisStep = self.fireControl.getTrack(primaryTargets[port]).foundInThisStep
                primaryTargetPos = self.fireControl.getTrack(primaryTargets[port]).lastFoundPos
                primaryTargetVel = self.fireControl.getTrack(primaryTargets[port]).lastFoundVel
                
                distanceToPrimaryTarget = calcDistance3d(myPos,primaryTargetPos)
                vecDirectionToPrimaryTarget = calcDirection3d(myPos,primaryTargetPos)
                radAspectAngle3DOfPrimaryTarget = calcRadAspectAngle3D(myPos, primaryTargetPos, primaryTargetVel)
                radDirectionToPrimaryTarget = calcRadAspectAngle3D(primaryTargetPos,myPos,myVel)
            else:
                primaryTargetFoundInThisStep = False
                distanceToPrimaryTarget = np.inf
                vecDirectionToPrimaryTarget = primaryAxis
                radAspectAngle3DOfPrimaryTarget = 180
                radDirectionToPrimaryTarget = 0
            
            #################################
            # calc maneuver from state
            #################################
            if(nextStates[port] == STATE.APPROACH_TARGET):
                if(primaryTargetFoundInThisStep and (radAspectAngle3DOfPrimaryTarget < -np.pi/2 or radAspectAngle3DOfPrimaryTarget > np.pi/2)):#敵機が見えており、Hotの場合
                    interceptPoint = calcInterceptPoint2D2(myPos,primaryTargetPos,primaryTargetVel)
                    vecTargetDir2D = calcDirection3d(myPos,interceptPoint)
                    vecTargetDir2D = clipDirection(vecTargetDir2D,vecDirectionToPrimaryTarget,85/180*np.pi)
                elif(primaryTargetFoundInThisStep): #敵機が見えており、Cold
                    vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                else:           #敵機の最終発見地点へ
                    vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                
                mTargetAlt = ALT_AT_FIRST_SHOT
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=10*np.pi/180, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextStates[port] == STATE.IN or nextStates[port] == STATE.CRANKING):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                
                if(northArmIndex == port):
                    mReferencePoint2D = (mNsOffsets[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[1],0,0)
                    
                
                
                if(float(time) < 30):
                    mTargetAlt = ALT_AT_IN
                else:
                    mTargetAlt = ALT_AT_FIRST_SHOT
                        
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 60/180*np.pi/3000,radDirectionDeviationLimit=20/180*np.pi,radClimbAngleLimit=10*np.pi/180, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
            
            
            if(self.states[port] != nextStates[port]):
                self.util.printLog(time, names[port], str(self.states[port])+"->"+str(nextStates[port])+"countdown = {:.1f}".format(self.outCountdowns[port]))
            
            self.states[port] = nextStates[port]
            
            command = {
                    "motion":{
                        "pitch":pitchCommand,
                        "roll":rollCommand,
                        "throttle":throttle
                    },
                    "weapon":{
                        "launch":launches[port],
                        "target":targets[port]
                    }
                }
            
            commands[port] = command
        
        return commands,tacticsChangeRequest
    
    def oneOnOneDefence(self,time,observableList,names,primaryTargets,primaryAxis=(0,1,0)):
        mNsOffsets = [20000,-20000]
        # mNsOffsets = [30000,-30000]
        
        commands = [None,None]
        
        
        for port in range(2):
            if(observableList[port] is None):
                if(self.isAlive[port]):
                    self.isAlive[port] = False
                    print("{} : {} Down ".format(timeToStr(time),names[port]))
                continue
            
            obs = observableList[port]
            
            myPos = obs["motion"]["pos"]
            myVel = obs["motion"]["vel"]
            radRoll,radPitch,radHeading =  QuaternionToRollPitchHeading(obs["motion"]["q"])
            
            if(not primaryTargets[port] is None):
                primaryTrack = self.fireControl.getTrack(primaryTargets[port]).track3D
            else:
                primaryTrack = None
            
            if(not primaryTargets[port] is None):
                primaryTargetFoundInThisStep = self.fireControl.getTrack(primaryTargets[port]).foundInThisStep
                primaryTargetPos = self.fireControl.getTrack(primaryTargets[port]).lastFoundPos
                primaryTargetVel = self.fireControl.getTrack(primaryTargets[port]).lastFoundVel
                
                distanceToPrimaryTarget = calcDistance3d(myPos,primaryTargetPos)
                vecDirectionToPrimaryTarget = calcDirection3d(myPos,primaryTargetPos)
                radAspectAngle3DOfPrimaryTarget = calcRadAspectAngle3D(myPos, primaryTargetPos, primaryTargetVel)
                radDirectionToPrimaryTarget = calcRadAspectAngle3D(primaryTargetPos,myPos,myVel)
            else:
                primaryTargetFoundInThisStep = False
                distanceToPrimaryTarget = np.inf
                vecDirectionToPrimaryTarget = primaryAxis
                radAspectAngle3DOfPrimaryTarget = 180
                radDirectionToPrimaryTarget = 0
            
            secondaryTarget = self.fireControl.getAnotherLiveTruth(primaryTargets[port])
            if(not secondaryTarget is None):
                secondaryTrack = self.fireControl.getTrack(secondaryTarget).track3D
            else:
                secondaryTrack = None
                
            if(not secondaryTarget is None):
                secondaryTargetFoundInThisStep = self.fireControl.getTrack(secondaryTarget).foundInThisStep
                secondaryTargetPos = self.fireControl.getTrack(secondaryTarget).lastFoundPos
                secondaryTargetVel = self.fireControl.getTrack(secondaryTarget).lastFoundVel
                
                distanceToSecondaryTarget = calcDistance3d(myPos,secondaryTargetPos)
                vecDirectionToSecondaryTarget = calcDirection3d(myPos, secondaryTargetPos)
                radAspectAngle3DOfSecnodaryTarget = calcRadAspectAngle3D(myPos, secondaryTargetPos, secondaryTargetVel)
                radDirectionToSecondaryTarget = calcRadAspectAngle3D(secondaryTargetPos,myPos,myVel)
            else:
                secondaryTargetFoundInThisStep = False
                distanceToSecondaryTarget = np.inf
                vecDirectionToSecondaryTarget = primaryAxis
                radAspectAngle3DOfSecondaryTarget = 180
                radDirectionToSecondaryTarget = 0
            
            
            # self.descendStartCountdowns[port],self.descendEndCountdowns[port] = updateDescenCountdownForAllTracks(self.descendStartCountdowns[port],self.descendEndCountdowns[port], myPos, primaryTrack, secondaryTrack)
            
            # if(self.descendEndCountdowns[port] > 0 and self.descendStartCountdowns[port]*MPS_MAXSPEED/2 < (-float(myPos[2])+ALT_AT_GUIDANCE)):
            #     altAtIn = ALT_AT_GUIDANCE     #9000mにいると危険なレンジで撃たれている可能性あり
            # else:
            #     altAtIn = ALT_AT_IN         #危険無し。9000mまで上昇
                
            ##############################
            # decide next state
            ##############################
            launch = False
            target = Track3D().to_json()
            
            if(self.states[port] == STATE.OUT_FROM_MWS):
                if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire when OUT_FROM_MWS, backward shot, target: {}".format(str(primaryTrack["truth"])))
                    self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                    self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                    launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                else:
                    self.snakingStartTime[port] = -1
                    nextState = STATE.EM_RECOVER1
                    
            elif(self.states[port] == STATE.OUT_FROM_CRITICAL_RANGE):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                else:
                    self.outFromCriticalRangeStartTimes[port] = float(time)
                    nextState = STATE.OUT_KEEP
                    
                    
            elif(self.states[port] == STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE):
                if((not primaryTrack is None) and self.fireControl.isLaunchAwaiting(observableList,port)):
                    nextState = STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE
                else:
                    nextState = STATE.OUT_FROM_CRITICAL_RANGE
                    
                    
            elif(self.states[port] == STATE.OUT_KEEP):
                if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire when OUT_KEEP, backward shot, target: {}".format(str(primaryTrack["truth"])))
                    self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                    self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                    launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(float(time)-self.outFromCriticalRangeStartTimes[port] < S_OUT_KEEP_TIME):
                    nextState = STATE.OUT_KEEP
                else:
                    nextState = STATE.EM_RECOVER1
                    
                    
            elif(self.states[port] == STATE.EM_RECOVER1):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(abs3d(obs["motion"]["vel"]) < MPS_VEL_RECOVER_CRITERIA or radPitch < -5/180*np.pi):
                    nextState = STATE.EM_RECOVER1
                else:
                    nextState = STATE.EM_RECOVER2
            elif(self.states[port] == STATE.EM_RECOVER2):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(float(obs["motion"]["pos"][2]) > M_ALT_RECOVER_CRITERIA):
                    nextState = STATE.EM_RECOVER2
                else:
                    if(float(obs["motion"]["pos"][2]) > M_VERTICAL_IN_CRITERIA and radPitch>2.0*np.pi/180): #高度低ければ垂直にIN。ピッチが下向きの場合もHORIZONTALとする
                        nextState = STATE.IN
                    else:                                                       #高度高ければ水平にIN
                        nextState = STATE.HORIZONTAL_IN
                    self.outCountdowns[port] = np.inf
            elif(self.states[port] == STATE.HORIZONTAL_IN):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 90*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                elif((not primaryTargets[port] is None) and (radDirectionToPrimaryTarget > 90/180*np.pi) or (radDirectionToPrimaryTarget < -90/180*np.pi)):
                    nextState = STATE.APPROACH_TARGET
                else:
                    nextState = STATE.HORIZONTAL_IN
            elif(self.states[port] == STATE.APPROACH_TARGET):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and (not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when APPROACH_TARGET->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when APPROACH_TARGET->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)

                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                    
                    
                elif((not primaryTrack is None) and self.outCountdowns[port] <= 3.0):
                    if(radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and (not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)) and distanceToPrimaryTarget < M_SHOTBACK_RANGE and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when APPROACH_TARGET->LAUNCH_AND_OUT_FROM_CRITICAL, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when APPROACH_TARGET->LAUNCH_AND_OUT_FROM_CRITICAL, target: {}".format(str(primaryTrack["truth"])))
                        nextState = STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                        
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                        nextState = STATE.OUT_FROM_CRITICAL_RANGE
                        
                        
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                        
                        
                elif((not primaryTrack is None) and isHot(radAspectAngle3DOfPrimaryTarget) and distanceToPrimaryTarget < M_LAUNCH_RANGE_HOT and float(myPos[2])<(ALT_AT_IN+1000) and radDirectionToPrimaryTarget > 100*np.pi/180):
                    nextState = STATE.CRANKING
                    self.crankingStartTime[port] = float(time)
                    
                    if(not self.fireControl.isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA) and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], "RequestFire when APPROACH_TARGET->CRANKING, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"RequestFire when APPROACH_TARGET->CRANKING, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: targetShot :{}".format(self.fireControl._isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"Launch abort: targetShot :{}".format(self.fireControl.isTargetShot(time,observableList, primaryTrack)))
                        
                        
                #elif(not isHot(radAspectAngle3DOfPrimaryTarget) and distanceToPrimaryTarget < M_LAUNCH_RANGE_COLD):
                elif(not isHot(radAspectAngle3DOfPrimaryTarget) and distanceToPrimaryTarget < calcLaunchRangeCold(radAspectAngle3DOfPrimaryTarget)):
                    nextState = STATE.APPROACH_TARGET
                    
                    if((not primaryTrack is None) and (not self.fireControl.isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)) and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], "RequestFire when APPROACH_TARGET->APPROACH_TARGET, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"RequestFire when APPROACH_TARGET->APPROACH_TARGET, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList,port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: targetShot :{}, PrimaryTrack :{}".format(self.fireControl._isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA),primaryTargets[port]))
                        #print(timeToStr(time),":",names[port],"Launch abort: targetShot :{}, PrimaryTrack :{}".format(self.fireControl.isTargetShot(time,observableList, primaryTrack),primaryTargets[port]))
                        
                        
                elif(not primaryTargets[port] is None):
                    nextState = STATE.APPROACH_TARGET
                    
                    
                else:
                    nextState = STATE.IN
                    
                    
            elif(self.states[port] == STATE.CRANKING):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    self.snakingStartTime[port] = -1
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when CRANKING->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when CRANKING->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.outCountdowns[port] <= 3.0):
                    nextState = STATE.OUT_FROM_CRITICAL_RANGE
                    self.snakingStartTime[port] = -1
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                    self.snakingStartTime[port] = -1
                elif(((not isHot(radAspectAngle3DOfPrimaryTarget)) or (not self.fireControl.isTargetShot(time, observableList, primaryTrack, 200))) and (float(time)-self.crankingStartTime[port]) > S_MINIMUM_CRANKING_TIME):
                    nextState = STATE.IN
                    self.snakingStartTime[port] = -1
                elif(not primaryTargets[port] is None):
                    nextState = STATE.CRANKING
                else:
                    nextState = STATE.IN
                    self.snakingStartTime[port] = -1
                    
                    
            elif(self.states[port] == STATE.IN):   #STATE_IN
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                elif(not primaryTargets[port] is None):
                    nextState = STATE.APPROACH_TARGET
                else:
                    nextState = STATE.IN
                    
                    
            elif(self.states[port] == STATE.PENETRATING):
                nextState = STATE.PENETRATING
            
            
            
            #################################
            # calc maneuver from state
            #################################
            if(nextState == STATE.OUT_FROM_MWS):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                
                if(radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD):
                    rollCommand,pitchCommand = outFromMws(time,obs,ALT_AT_OUT,vecTargetDir2D,ALT_AT_OUT,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000)
                else:
                    if(self.snakingStartTime[port] < 0):
                        if(isMwsActive(obs)):
                            mwsDir = meanMwsDir(obs)
                            self.snakingPolarity[port] = calcSnakingPolarity(-mwsDir, primaryAxis)
                        else:
                            self.snakingPolarity[port] = 1
                            
                        self.snakingStartTime[port] = float(time)
                    
                    snakingTime = float(time) - self.snakingStartTime[port]
                    #rollCommand,pitchCommand = outFromMws(time,observableList,ALT_AT_OUT,vecTargetDir2D,ALT_AT_OUT,radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000)
                    rollCommand,pitchCommand = outFromMwsWithSnaking(snakingTime,obs,ALT_AT_OUT,polarity=self.snakingPolarity[port],sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=vecTargetDir2D,mTargetAltAtIn=ALT_AT_IN,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                    #rollCommand,pitchCommand = outFromMwsWithBarrel(snakingTime,obs,ALT_AT_OUT,mAltAmplitude=-3000,polarity=self.snakingPolarity[port],sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=vecTargetDir2D,mTargetAltAtIn=ALT_AT_IN,radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-45/180*np.pi, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                
                throttle = 1.0
                
            elif(nextState == STATE.OUT_FROM_CRITICAL_RANGE):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_OUT
                
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=10/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                throttle = 1.0
                
            elif(nextState == STATE.OUT_KEEP):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_OUT
                
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=20/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                throttle = 1.0
                
            elif(nextState == STATE.EM_RECOVER1):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=30/180*np.pi,radClimbAngleLimit=2/180*np.pi, radDiveAngleLimit=-2/180*np.pi, altGain=1/100,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.EM_RECOVER2):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=30/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.HORIZONTAL_IN):
                RAD_OFF_ANGLE = 80*np.pi/180
                
                radDirection2DToPrimaryTarget = calcRadDirectionDifference2D(myVel,vecDirectionToPrimaryTarget)
                if(radDirection2DToPrimaryTarget > 0):
                    polarity = 1
                else:
                    polarity = -1
                
                r = R.from_quat([0, 0,np.sin(polarity*RAD_OFF_ANGLE/2),np.cos(polarity*RAD_OFF_ANGLE/2)])
                
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                vecTargetDir2D = r.apply(vecTargetDir2D)
                
                mTargetAlt = ALT_AT_HORIZONTAL_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=10*np.pi/180, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE):
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=0, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0            
            elif(nextState == STATE.APPROACH_TARGET):
                if(primaryTargetFoundInThisStep and (radAspectAngle3DOfPrimaryTarget < -np.pi/2 or radAspectAngle3DOfPrimaryTarget > np.pi/2)):#敵機が見えており、Hotの場合
                    interceptPoint = calcInterceptPoint2D2(myPos,primaryTargetPos,primaryTargetVel)
                    vecTargetDir2D = calcDirection3d(myPos,interceptPoint)
                    vecTargetDir2D = clipDirection(vecTargetDir2D,vecDirectionToPrimaryTarget,85/180*np.pi)
                elif(primaryTargetFoundInThisStep): #敵機が見えており、Cold
                    npPrimaryTargetPos = np.array([float(primaryTargetPos[0]),float(primaryTargetPos[1]),float(primaryTargetPos[2])])
                    npPrimaryTargetVel = np.array([float(primaryTargetVel[0]),float(primaryTargetVel[1]),float(primaryTargetVel[2])])
                    
                    #estimatedPrimaryTargetPos = npPrimaryTargetPos + S_COLD_TARGET_LEAD_AHEAD_TIME * npPrimaryTargetVel
                    primaryTargetPosOffset = np.array([primaryAxis[0],primaryAxis[1],0]) * distanceToPrimaryTarget*COLD_TARGET_OFFSET_RATIO
                    estimatedPrimaryTargetPos = npPrimaryTargetPos - primaryTargetPosOffset + S_COLD_TARGET_LEAD_AHEAD_TIME * npPrimaryTargetVel
                    
                    vecTargetDir2D = calcDirection3d(myPos ,estimatedPrimaryTargetPos)
                    vecTargetDir2D = clipDirection(vecTargetDir2D,vecDirectionToPrimaryTarget,85/180*np.pi)
                else:           #敵機の最終発見地点へ
                    vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                
                if(float(myPos[0]) > 73000 and vecTargetDir2D[0] > 0):
                    vecTargetDir2D[0] = 0
                
                if(float(myPos[0]) < -73000 and vecTargetDir2D[0] < 0):
                    vecTargetDir2D[0] = 0
                
                # if(self.outCountdowns[port] > 0 and self.outCountdowns[port]*MPS_MAXSPEED/2 < (-float(myPos[2])+ALT_AT_GUIDANCE)):
                #     mTargetAlt = ALT_AT_GUIDANCE
                # else:
                #     mTargetAlt = altAtIn
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.CRANKING):
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                mTargetAlt = ALT_AT_GUIDANCE
                
                if(self.snakingStartTime[port] < 0):
                    self.snakingStartTime[port] = float(time)
                    self.snakingPolarity[port] = calcSnakingPolarity(vecTargetDir2D, primaryAxis)
                
                snakingTime = float(time) - self.snakingStartTime[port]
                
                #rollCommand,pitchCommand = altDirHold(observableList, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000)
                rollCommand,pitchCommand = snaking(snakingTime,obs,vecTargetDir2D,mTargetAlt,self.snakingPolarity[port],sCycle=60.0,radSnakingAngle=75.0/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.IN):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 60/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
            elif(nextState == STATE.PENETRATING):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
            
            
            #Secondaryへの射撃可否判定
            if(not launch):
                if((not secondaryTrack is None) and distanceToSecondaryTarget < M_SECONDARY_TARGET_LAUNCH_RANGE and radAspectAngle3DOfSecnodaryTarget > 120*np.pi/180 and radDirectionToSecondaryTarget < 60*np.pi/180  and (not self.fireControl.isTargetShot(time,observableList,secondaryTrack,MPS_MISSILE_ALIVE_CRITERIA)) and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire to Secondary Target, target: {}".format(str(secondaryTrack["truth"])))
                    #print(timeToStr(time),":",names[port]," RequestFire to Secondary Target, target: {}".format(str(secondaryTrack["truth"])))
                    launch,target = self.fireControl.requestFire(time, observableList, port, secondaryTrack)
            
            
            if(self.primaryTargets[port] != primaryTargets[port]):
                self.util.printLog(time, names[port], "PrimaryTargetChanged "+str(self.primaryTargets[port])+"->"+str(primaryTargets[port]))
                #print("{} : {} PrimaryTargetChanged ".format(timeToStr(time),names[port]),self.primaryTargets[port],"->",primaryTargets[port])
            self.primaryTargets[port] = primaryTargets[port]
            
            
            if(self.states[port] != nextState):
                self.util.printLog(time, names[port], str(self.states[port])+"->"+str(nextState)+"countdown = {:.1f}".format(self.outCountdowns[port]))
                #print("{} : {}".format(timeToStr(time),names[port]),self.states[port],"->",nextState,"countdonw = {:.1f}".format(self.outCountdowns[port]))
            # if(self.states[port] != nextState and nextState == STATE.PENETRATING):
            #     print("{:.1f} : {}".format(time,names[port]),self.states[port],"->",nextState)
            self.states[port] = nextState
            
            
            
            
            # print("next missile: ",observableList["weapon"]["nextMsl"],observableList["weapon"]["launchable"])
            # print("missiles",observableList["weapon"]["missiles"])
            
            # if(launch):
            #     print(target)
            
            command = {
                    "motion":{
                        "pitch":pitchCommand,
                        "roll":rollCommand,
                        "throttle":throttle
                    },
                    "weapon":{
                        "launch":launch,
                        "target":target
                    }
                }
            
            commands[port] = command
        
        ##################################
        # Cross Fire 判定
        ##################################
        if((not observableList[0] is None) and (not observableList[1] is None)):
            blue1Pos = observableList[0]["motion"]["pos"]
            blue2Pos = observableList[1]["motion"]["pos"]
            
            for mainShooter,subShooter in ((0,1),(1,0)):
                if(commands[mainShooter]["weapon"]["launch"] == True and commands[subShooter]["weapon"]["launch"] == False):
                    crossfireTrack = commands[mainShooter]["weapon"]["target"]
                    crossfirePos = self.fireControl.getTrack(crossfireTrack["truth"]).lastFoundPos
                    blue1ToTargetDistance = calcDistance3d(blue1Pos,crossfirePos)
                    blue2ToTargetDistance = calcDistance3d(blue2Pos,crossfirePos)
                    radCrossfireAngle = calcRadAngle3Dof3Points(crossfirePos,blue1Pos,blue2Pos)
                    
                    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    # print("blue1ToTargetDistance:",blue1ToTargetDistance,", in range",blue1ToTargetDistance < M_CROSSFIRE_RANGE)
                    # print("blue2ToTargetDistance:",blue2ToTargetDistance,", in range",blue2ToTargetDistance < M_CROSSFIRE_RANGE)
                    # print("degCrossFireAngle:",radCrossfireAngle*180/np.pi,", in criteria",radCrossfireAngle > RAD_CROSSFIRE_ANGLE_LIMIT)
                    # print("shotBy:",self.fireControl.isTargetShotBy(time,subShooter,observableList,crossfireTrack))
                    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    
                    if(blue1ToTargetDistance < M_CROSSFIRE_RANGE and blue2ToTargetDistance < M_CROSSFIRE_RANGE and radCrossfireAngle > RAD_CROSSFIRE_ANGLE_LIMIT):
                        if(not self.fireControl.isTargetShotBy(time,subShooter,observableList,crossfireTrack,MPS_MISSILE_ALIVE_CRITERIA) and (observableList[subShooter]["weapon"]["launchable"])):
                            # print("")
                            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            self.util.printLog(time, names[subShooter], " RequestFire when CROSSFIRE, target: {}".format(str(crossfireTrack["truth"])))
                            #print(timeToStr(time),":",names[subShooter]," RequestFire when CROSSFIRE, target: {}".format(str(crossfireTrack["truth"])))
                            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            # print("")
                            launch,target = self.fireControl.requestFire(time, observableList, subShooter, crossfireTrack)
                            
                            commands[subShooter]["weapon"]["launch"] = launch
                            commands[subShooter]["weapon"]["target"] = target
        
        return commands
    
    def escortAndPenetration(self,time,observableList,names,primaryTargets,primaryAxis=(0,1,0)):
        mNsOffsets = [20000,-20000]
        # mNsOffsets = [30000,-30000]
        
        commands = [None,None]
        
        mPenetratorNsOffset = 0
        if((not observableList[0] is None) and  (not observableList[0] is None)):
            blue1Pos = observableList[0]["motion"]["pos"]
            blue2Pos = observableList[1]["motion"]["pos"]
            
            if(float(blue1Pos[0]) > float(blue2Pos[0])):
                northArmIndex = 0
                southArmIndex = 1
            else:
                northArmIndex = 1
                southArmIndex = 0
            
            if(not primaryTargets[0] is None):
                if(float(self.fireControl.getTrack(primaryTargets[0]).lastFoundPos[0]) < 0):
                    escortPort = southArmIndex
                    penetratorPort = northArmIndex
                    mPenetratorNsOffset = mNsOffsets[0]
                else:
                    escortPort = northArmIndex
                    penetratorPort = southArmIndex
                    mPenetratorNsOffset = mNsOffsets[1]
        elif(not observableList[0] is None):
            escortPort = 0
            penetratorPort = -1
        else:
            escortPort = 1
            penetratorPort = -1
        
        
        #Penetratorの動作
        if(penetratorPort >= 0):
            port = penetratorPort
            
            obs = observableList[port]
            
            myPos = obs["motion"]["pos"]
            myVel = obs["motion"]["vel"]
            radRoll,radPitch,radHeading =  QuaternionToRollPitchHeading(obs["motion"]["q"])
            
            if(not primaryTargets[port] is None):
                primaryTrack = self.fireControl.getTrack(primaryTargets[port]).track3D
            else:
                primaryTrack = None
            
            if(not primaryTargets[port] is None):
                primaryTargetFoundInThisStep = self.fireControl.getTrack(primaryTargets[port]).foundInThisStep
                primaryTargetPos = self.fireControl.getTrack(primaryTargets[port]).lastFoundPos
                primaryTargetVel = self.fireControl.getTrack(primaryTargets[port]).lastFoundVel
                
                distanceToPrimaryTarget = calcDistance3d(myPos,primaryTargetPos)
                vecDirectionToPrimaryTarget = calcDirection3d(myPos,primaryTargetPos)
                radAspectAngle3DOfPrimaryTarget = calcRadAspectAngle3D(myPos, primaryTargetPos, primaryTargetVel)
                radDirectionToPrimaryTarget = calcRadAspectAngle3D(primaryTargetPos,myPos,myVel)
            else:
                primaryTargetFoundInThisStep = False
                distanceToPrimaryTarget = np.inf
                vecDirectionToPrimaryTarget = primaryAxis
                radAspectAngle3DOfPrimaryTarget = 180
                radDirectionToPrimaryTarget = 0
            
            secondaryTarget = self.fireControl.getAnotherLiveTruth(primaryTargets[port])
            if(not secondaryTarget is None):
                secondaryTrack = self.fireControl.getTrack(secondaryTarget).track3D
            else:
                secondaryTrack = None
                
            if(not secondaryTarget is None):
                secondaryTargetFoundInThisStep = self.fireControl.getTrack(secondaryTarget).foundInThisStep
                secondaryTargetPos = self.fireControl.getTrack(secondaryTarget).lastFoundPos
                secondaryTargetVel = self.fireControl.getTrack(secondaryTarget).lastFoundVel
                
                distanceToSecondaryTarget = calcDistance3d(myPos,secondaryTargetPos)
                vecDirectionToSecondaryTarget = calcDirection3d(myPos, secondaryTargetPos)
                radAspectAngle3DOfSecnodaryTarget = calcRadAspectAngle3D(myPos, secondaryTargetPos, secondaryTargetVel)
                radDirectionToSecondaryTarget = calcRadAspectAngle3D(secondaryTargetPos,myPos,myVel)
            else:
                secondaryTargetFoundInThisStep = False
                distanceToSecondaryTarget = np.inf
                vecDirectionToSecondaryTarget = primaryAxis
                radAspectAngle3DOfSecondaryTarget = 180
                radDirectionToSecondaryTarget = 0
            
            
                
            ##############################
            # decide next state
            ##############################
            launch = False
            target = Track3D().to_json()
            
            if(self.states[port] == STATE.OUT_FROM_MWS):
                if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire when OUT_FROM_MWS, backward shot, target: {}".format(str(primaryTrack["truth"])))
                    self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                    self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                    launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                else:
                    self.snakingStartTime[port] = -1
                    nextState = STATE.EM_RECOVER1
                    
            elif(self.states[port] == STATE.OUT_FROM_CRITICAL_RANGE):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                else:
                    self.outFromCriticalRangeStartTimes[port] = float(time)
                    nextState = STATE.OUT_KEEP
                    
                    
            elif(self.states[port] == STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE):
                if((not primaryTrack is None) and self.fireControl.isLaunchAwaiting(observableList,port)):
                    nextState = STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE
                else:
                    nextState = STATE.OUT_FROM_CRITICAL_RANGE
                    
                    
            elif(self.states[port] == STATE.OUT_KEEP):
                if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire when OUT_KEEP, backward shot, target: {}".format(str(primaryTrack["truth"])))
                    self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                    self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                    launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(float(time)-self.outFromCriticalRangeStartTimes[port] < S_OUT_KEEP_TIME):
                    nextState = STATE.OUT_KEEP
                else:
                    nextState = STATE.EM_RECOVER1
                    
                    
            elif(self.states[port] == STATE.EM_RECOVER1):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(abs3d(obs["motion"]["vel"]) < MPS_VEL_RECOVER_CRITERIA or radPitch < -5/180*np.pi):
                    nextState = STATE.EM_RECOVER1
                else:
                    nextState = STATE.EM_RECOVER2
            elif(self.states[port] == STATE.EM_RECOVER2):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(float(obs["motion"]["pos"][2]) > M_ALT_RECOVER_CRITERIA):
                    nextState = STATE.EM_RECOVER2
                else:
                    nextState = STATE.IN
                    self.outCountdowns[port] = np.inf
            elif(self.states[port] == STATE.HORIZONTAL_IN):
                nextState = STATE.IN                    
            
            elif(self.states[port] == STATE.APPROACH_TARGET):
                nextState = STATE.IN
                    
            elif(self.states[port] == STATE.CRANKING):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    self.snakingStartTime[port] = -1
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when CRANKING->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when CRANKING->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.outCountdowns[port] <= 3.0):
                    nextState = STATE.OUT_FROM_CRITICAL_RANGE
                    self.snakingStartTime[port] = -1
                elif(((not isHot(radAspectAngle3DOfPrimaryTarget)) or (not self.fireControl.isTargetShot(time, observableList, primaryTrack, 200))) and (float(time)-self.crankingStartTime[port]) > S_MINIMUM_CRANKING_TIME):
                    nextState = STATE.IN
                    self.snakingStartTime[port] = -1
                elif(not primaryTargets[port] is None):
                    nextState = STATE.CRANKING
                else:
                    nextState = STATE.IN
                    self.snakingStartTime[port] = -1
                    
                    
            elif(self.states[port] == STATE.IN):   #STATE_IN
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                else:
                    nextState = STATE.IN
                    
            elif(self.states[port] == STATE.PENETRATING):
                nextState = STATE.IN
            
            #################################
            # calc maneuver from state
            #################################
            if(nextState == STATE.OUT_FROM_MWS):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                
                if(radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD):
                    rollCommand,pitchCommand = outFromMws(time,obs,ALT_AT_OUT,vecTargetDir2D,ALT_AT_OUT,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000)
                else:
                    if(self.snakingStartTime[port] < 0):
                        if(isMwsActive(obs)):
                            mwsDir = meanMwsDir(obs)
                            self.snakingPolarity[port] = calcSnakingPolarity(-mwsDir, primaryAxis)
                        else:
                            self.snakingPolarity[port] = 1
                            
                        self.snakingStartTime[port] = float(time)
                    
                    snakingTime = float(time) - self.snakingStartTime[port]
                    #rollCommand,pitchCommand = outFromMws(time,observableList,ALT_AT_OUT,vecTargetDir2D,ALT_AT_OUT,radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000)
                    rollCommand,pitchCommand = outFromMwsWithSnaking(snakingTime,obs,ALT_AT_OUT,polarity=self.snakingPolarity[port],sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=vecTargetDir2D,mTargetAltAtIn=ALT_AT_IN,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                    #rollCommand,pitchCommand = outFromMwsWithBarrel(snakingTime,obs,ALT_AT_OUT,mAltAmplitude=-3000,polarity=self.snakingPolarity[port],sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=vecTargetDir2D,mTargetAltAtIn=ALT_AT_IN,radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-45/180*np.pi, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                
                throttle = 1.0
                
            elif(nextState == STATE.OUT_FROM_CRITICAL_RANGE):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_OUT
                
                mReferencePoint2D = (mPenetratorNsOffset,0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=10/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                throttle = 1.0
                
            elif(nextState == STATE.OUT_KEEP):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_OUT
                
                mReferencePoint2D = (mPenetratorNsOffset,0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=20/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                throttle = 1.0
                
            elif(nextState == STATE.EM_RECOVER1):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                mReferencePoint2D = (mPenetratorNsOffset,0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=30/180*np.pi,radClimbAngleLimit=2/180*np.pi, radDiveAngleLimit=-2/180*np.pi, altGain=1/100,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.EM_RECOVER2):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                mReferencePoint2D = (mPenetratorNsOffset,0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=30/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.HORIZONTAL_IN):
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=10*np.pi/180, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                mReferencePoint2D = (mPenetratorNsOffset,0,0)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 60/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.APPROACH_TARGET):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                mReferencePoint2D = (mPenetratorNsOffset,0,0)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 60/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.CRANKING):
                mReferencePoint = (mPenetratorNsOffset*2,primaryAxis[1]*100000,0)
                vecTargetDir2D = calcDirection3d(myPos ,mReferencePoint)
                mTargetAlt = ALT_AT_GUIDANCE
                
                if(self.snakingStartTime[port] < 0):
                    self.snakingStartTime[port] = float(time)
                    self.snakingPolarity[port] = calcSnakingPolarity(vecTargetDir2D, primaryAxis)
                
                snakingTime = float(time) - self.snakingStartTime[port]
                
                #rollCommand,pitchCommand = altDirHold(observableList, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000)
                rollCommand,pitchCommand = snaking(snakingTime,obs,vecTargetDir2D,mTargetAlt,self.snakingPolarity[port],sCycle=60.0,radSnakingAngle=75.0/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.IN):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                mReferencePoint2D = (mPenetratorNsOffset,0,0)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 60/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
            elif(nextState == STATE.PENETRATING):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
            
            command = {
                    "motion":{
                        "pitch":pitchCommand,
                        "roll":rollCommand,
                        "throttle":throttle
                    },
                    "weapon":{
                        "launch":launch,
                        "target":target
                    }
                }
            
            commands[port] = command
        
        #ここからはOneOnOneDefenceと同一
        if(escortPort >= 0):
            port = escortPort
            
            obs = observableList[port]
            
            myPos = obs["motion"]["pos"]
            myVel = obs["motion"]["vel"]
            radRoll,radPitch,radHeading =  QuaternionToRollPitchHeading(obs["motion"]["q"])
            
            if(not primaryTargets[port] is None):
                primaryTrack = self.fireControl.getTrack(primaryTargets[port]).track3D
            else:
                primaryTrack = None
            
            if(not primaryTargets[port] is None):
                primaryTargetFoundInThisStep = self.fireControl.getTrack(primaryTargets[port]).foundInThisStep
                primaryTargetPos = self.fireControl.getTrack(primaryTargets[port]).lastFoundPos
                primaryTargetVel = self.fireControl.getTrack(primaryTargets[port]).lastFoundVel
                
                distanceToPrimaryTarget = calcDistance3d(myPos,primaryTargetPos)
                vecDirectionToPrimaryTarget = calcDirection3d(myPos,primaryTargetPos)
                radAspectAngle3DOfPrimaryTarget = calcRadAspectAngle3D(myPos, primaryTargetPos, primaryTargetVel)
                radDirectionToPrimaryTarget = calcRadAspectAngle3D(primaryTargetPos,myPos,myVel)
            else:
                primaryTargetFoundInThisStep = False
                distanceToPrimaryTarget = np.inf
                vecDirectionToPrimaryTarget = primaryAxis
                radAspectAngle3DOfPrimaryTarget = 180
                radDirectionToPrimaryTarget = 0
            
            secondaryTarget = self.fireControl.getAnotherLiveTruth(primaryTargets[port])
            if(not secondaryTarget is None):
                secondaryTrack = self.fireControl.getTrack(secondaryTarget).track3D
            else:
                secondaryTrack = None
                
            if(not secondaryTarget is None):
                secondaryTargetFoundInThisStep = self.fireControl.getTrack(secondaryTarget).foundInThisStep
                secondaryTargetPos = self.fireControl.getTrack(secondaryTarget).lastFoundPos
                secondaryTargetVel = self.fireControl.getTrack(secondaryTarget).lastFoundVel
                
                distanceToSecondaryTarget = calcDistance3d(myPos,secondaryTargetPos)
                vecDirectionToSecondaryTarget = calcDirection3d(myPos, secondaryTargetPos)
                radAspectAngle3DOfSecnodaryTarget = calcRadAspectAngle3D(myPos, secondaryTargetPos, secondaryTargetVel)
                radDirectionToSecondaryTarget = calcRadAspectAngle3D(secondaryTargetPos,myPos,myVel)
            else:
                secondaryTargetFoundInThisStep = False
                distanceToSecondaryTarget = np.inf
                vecDirectionToSecondaryTarget = primaryAxis
                radAspectAngle3DOfSecondaryTarget = 180
                radDirectionToSecondaryTarget = 0
            
            
            # self.descendStartCountdowns[port],self.descendEndCountdowns[port] = updateDescenCountdownForAllTracks(self.descendStartCountdowns[port],self.descendEndCountdowns[port], myPos, primaryTrack, secondaryTrack)
            
            # if(self.descendEndCountdowns[port] > 0 and self.descendStartCountdowns[port]*MPS_MAXSPEED/2 < (-float(myPos[2])+ALT_AT_GUIDANCE)):
            #     altAtIn = ALT_AT_GUIDANCE     #9000mにいると危険なレンジで撃たれている可能性あり
            # else:
            #     altAtIn = ALT_AT_IN         #危険無し。9000mまで上昇
                
            ##############################
            # decide next state
            ##############################
            launch = False
            target = Track3D().to_json()
            
            if(self.states[port] == STATE.OUT_FROM_MWS):
                if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire when OUT_FROM_MWS, backward shot, target: {}".format(str(primaryTrack["truth"])))
                    self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                    self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                    launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                else:
                    self.snakingStartTime[port] = -1
                    nextState = STATE.EM_RECOVER1
                    
            elif(self.states[port] == STATE.OUT_FROM_CRITICAL_RANGE):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                else:
                    self.outFromCriticalRangeStartTimes[port] = float(time)
                    nextState = STATE.OUT_KEEP
                    
                    
            elif(self.states[port] == STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE):
                if((not primaryTrack is None) and self.fireControl.isLaunchAwaiting(observableList,port)):
                    nextState = STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE
                else:
                    nextState = STATE.OUT_FROM_CRITICAL_RANGE
                    
                    
            elif(self.states[port] == STATE.OUT_KEEP):
                if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire when OUT_KEEP, backward shot, target: {}".format(str(primaryTrack["truth"])))
                    self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                    self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                    launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(float(time)-self.outFromCriticalRangeStartTimes[port] < S_OUT_KEEP_TIME):
                    nextState = STATE.OUT_KEEP
                else:
                    nextState = STATE.EM_RECOVER1
                    
                    
            elif(self.states[port] == STATE.EM_RECOVER1):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(abs3d(obs["motion"]["vel"]) < MPS_VEL_RECOVER_CRITERIA or radPitch < -5/180*np.pi):
                    nextState = STATE.EM_RECOVER1
                else:
                    nextState = STATE.EM_RECOVER2
            elif(self.states[port] == STATE.EM_RECOVER2):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(float(obs["motion"]["pos"][2]) > M_ALT_RECOVER_CRITERIA):
                    nextState = STATE.EM_RECOVER2
                else:
                    if(float(obs["motion"]["pos"][2]) > M_VERTICAL_IN_CRITERIA and radPitch>2.0*np.pi/180): #高度低ければ垂直にIN
                        nextState = STATE.IN
                    else:                                                       #高度高ければ水平にIN
                        nextState = STATE.HORIZONTAL_IN
                    self.outCountdowns[port] = np.inf
            elif(self.states[port] == STATE.HORIZONTAL_IN):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 90*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                elif((not primaryTargets[port] is None) and (radDirectionToPrimaryTarget > 90/180*np.pi) or (radDirectionToPrimaryTarget < -90/180*np.pi)):
                    nextState = STATE.APPROACH_TARGET
                else:
                    nextState = STATE.HORIZONTAL_IN
            elif(self.states[port] == STATE.APPROACH_TARGET):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and (not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when APPROACH_TARGET->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when APPROACH_TARGET->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)

                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                    
                    
                elif((not primaryTrack is None) and self.outCountdowns[port] <= 3.0):
                    if(radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and (not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)) and distanceToPrimaryTarget < M_SHOTBACK_RANGE and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when APPROACH_TARGET->LAUNCH_AND_OUT_FROM_CRITICAL, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when APPROACH_TARGET->LAUNCH_AND_OUT_FROM_CRITICAL, target: {}".format(str(primaryTrack["truth"])))
                        nextState = STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                        
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                        nextState = STATE.OUT_FROM_CRITICAL_RANGE
                        
                        
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                        
                        
                elif((not primaryTrack is None) and isHot(radAspectAngle3DOfPrimaryTarget) and distanceToPrimaryTarget < M_LAUNCH_RANGE_HOT and float(myPos[2])<(ALT_AT_IN+1000) and radDirectionToPrimaryTarget > 100*np.pi/180):
                    nextState = STATE.CRANKING
                    self.crankingStartTime[port] = float(time)
                    
                    if(not self.fireControl.isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA) and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], "RequestFire when APPROACH_TARGET->CRANKING, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"RequestFire when APPROACH_TARGET->CRANKING, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: targetShot :{}".format(self.fireControl._isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"Launch abort: targetShot :{}".format(self.fireControl.isTargetShot(time,observableList, primaryTrack)))
                        
                        
                #elif(not isHot(radAspectAngle3DOfPrimaryTarget) and distanceToPrimaryTarget < M_LAUNCH_RANGE_COLD):
                elif(not isHot(radAspectAngle3DOfPrimaryTarget) and distanceToPrimaryTarget < calcLaunchRangeCold(radAspectAngle3DOfPrimaryTarget)):
                    nextState = STATE.APPROACH_TARGET
                    
                    if((not primaryTrack is None) and (not self.fireControl.isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)) and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], "RequestFire when APPROACH_TARGET->APPROACH_TARGET, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"RequestFire when APPROACH_TARGET->APPROACH_TARGET, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList,port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: targetShot :{}, PrimaryTrack :{}".format(self.fireControl._isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA),primaryTargets[port]))
                        #print(timeToStr(time),":",names[port],"Launch abort: targetShot :{}, PrimaryTrack :{}".format(self.fireControl.isTargetShot(time,observableList, primaryTrack),primaryTargets[port]))
                        
                        
                elif(not primaryTargets[port] is None):
                    nextState = STATE.APPROACH_TARGET
                    
                    
                else:
                    nextState = STATE.IN
                    
                    
            elif(self.states[port] == STATE.CRANKING):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    self.snakingStartTime[port] = -1
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when CRANKING->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when CRANKING->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.outCountdowns[port] <= 3.0):
                    nextState = STATE.OUT_FROM_CRITICAL_RANGE
                    self.snakingStartTime[port] = -1
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                    self.snakingStartTime[port] = -1
                elif(((not isHot(radAspectAngle3DOfPrimaryTarget)) or (not self.fireControl.isTargetShot(time, observableList, primaryTrack, 200))) and (float(time)-self.crankingStartTime[port]) > S_MINIMUM_CRANKING_TIME):
                    nextState = STATE.IN
                    self.snakingStartTime[port] = -1
                elif(not primaryTargets[port] is None):
                    nextState = STATE.CRANKING
                else:
                    nextState = STATE.IN
                    self.snakingStartTime[port] = -1
                    
                    
            elif(self.states[port] == STATE.IN):   #STATE_IN
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        self.util.printLog(time, names[port], "aspect of primaryTarget: {:.1f}, distance: {:.1f}, launchable: {:}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi, distanceToPrimaryTarget,(obs["weapon"]["launchable"])))
                        self.util.printLog(time, names[port], "aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port]," RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                elif(not primaryTargets[port] is None):
                    nextState = STATE.APPROACH_TARGET
                else:
                    nextState = STATE.IN
                    
                    
            elif(self.states[port] == STATE.PENETRATING):
                nextState = STATE.PENETRATING
            
            
            
            #################################
            # calc maneuver from state
            #################################
            if(nextState == STATE.OUT_FROM_MWS):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                
                if(radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD):
                    rollCommand,pitchCommand = outFromMws(time,obs,ALT_AT_OUT,vecTargetDir2D,ALT_AT_OUT,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000)
                else:
                    if(self.snakingStartTime[port] < 0):
                        if(isMwsActive(obs)):
                            mwsDir = meanMwsDir(obs)
                            self.snakingPolarity[port] = calcSnakingPolarity(-mwsDir, primaryAxis)
                        else:
                            self.snakingPolarity[port] = 1
                            
                        self.snakingStartTime[port] = float(time)
                    
                    snakingTime = float(time) - self.snakingStartTime[port]
                    #rollCommand,pitchCommand = outFromMws(time,observableList,ALT_AT_OUT,vecTargetDir2D,ALT_AT_OUT,radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000)
                    rollCommand,pitchCommand = outFromMwsWithSnaking(snakingTime,obs,ALT_AT_OUT,polarity=self.snakingPolarity[port],sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=vecTargetDir2D,mTargetAltAtIn=ALT_AT_IN,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                    #rollCommand,pitchCommand = outFromMwsWithBarrel(snakingTime,obs,ALT_AT_OUT,mAltAmplitude=-3000,polarity=self.snakingPolarity[port],sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=vecTargetDir2D,mTargetAltAtIn=ALT_AT_IN,radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-45/180*np.pi, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                
                throttle = 1.0
                
            elif(nextState == STATE.OUT_FROM_CRITICAL_RANGE):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_OUT
                
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=10/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                throttle = 1.0
                
            elif(nextState == STATE.OUT_KEEP):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_OUT
                
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=20/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                throttle = 1.0
                
            elif(nextState == STATE.EM_RECOVER1):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=30/180*np.pi,radClimbAngleLimit=2/180*np.pi, radDiveAngleLimit=-2/180*np.pi, altGain=1/100,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.EM_RECOVER2):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=30/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.HORIZONTAL_IN):
                RAD_OFF_ANGLE = 80*np.pi/180
                
                radDirection2DToPrimaryTarget = calcRadDirectionDifference2D(myVel,vecDirectionToPrimaryTarget)
                if(radDirection2DToPrimaryTarget > 0):
                    polarity = 1
                else:
                    polarity = -1
                
                r = R.from_quat([0, 0,np.sin(polarity*RAD_OFF_ANGLE/2),np.cos(polarity*RAD_OFF_ANGLE/2)])
                
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                vecTargetDir2D = r.apply(vecTargetDir2D)
                
                mTargetAlt = ALT_AT_HORIZONTAL_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=10*np.pi/180, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE):
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=0, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0            
            elif(nextState == STATE.APPROACH_TARGET):
                if(primaryTargetFoundInThisStep and (radAspectAngle3DOfPrimaryTarget < -np.pi/2 or radAspectAngle3DOfPrimaryTarget > np.pi/2)):#敵機が見えており、Hotの場合
                    interceptPoint = calcInterceptPoint2D2(myPos,primaryTargetPos,primaryTargetVel)
                    vecTargetDir2D = calcDirection3d(myPos,interceptPoint)
                    vecTargetDir2D = clipDirection(vecTargetDir2D,vecDirectionToPrimaryTarget,85/180*np.pi)
                elif(primaryTargetFoundInThisStep): #敵機が見えており、Cold
                    npPrimaryTargetPos = np.array([float(primaryTargetPos[0]),float(primaryTargetPos[1]),float(primaryTargetPos[2])])
                    npPrimaryTargetVel = np.array([float(primaryTargetVel[0]),float(primaryTargetVel[1]),float(primaryTargetVel[2])])
                    
                    #estimatedPrimaryTargetPos = npPrimaryTargetPos + S_COLD_TARGET_LEAD_AHEAD_TIME * npPrimaryTargetVel
                    primaryTargetPosOffset = np.array([primaryAxis[0],primaryAxis[1],0]) * distanceToPrimaryTarget*COLD_TARGET_OFFSET_RATIO
                    estimatedPrimaryTargetPos = npPrimaryTargetPos - primaryTargetPosOffset + S_COLD_TARGET_LEAD_AHEAD_TIME * npPrimaryTargetVel
                    
                    vecTargetDir2D = calcDirection3d(myPos ,estimatedPrimaryTargetPos)
                    vecTargetDir2D = clipDirection(vecTargetDir2D,vecDirectionToPrimaryTarget,85/180*np.pi)
                else:           #敵機の最終発見地点へ
                    vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                
                if(float(myPos[0]) > 73000 and vecTargetDir2D[0] > 0):
                    vecTargetDir2D[0] = 0
                
                if(float(myPos[0]) < -73000 and vecTargetDir2D[0] < 0):
                    vecTargetDir2D[0] = 0
                
                # if(self.outCountdowns[port] > 0 and self.outCountdowns[port]*MPS_MAXSPEED/2 < (-float(myPos[2])+ALT_AT_GUIDANCE)):
                #     mTargetAlt = ALT_AT_GUIDANCE
                # else:
                #     mTargetAlt = altAtIn
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.CRANKING):
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                mTargetAlt = ALT_AT_GUIDANCE
                
                if(self.snakingStartTime[port] < 0):
                    self.snakingStartTime[port] = float(time)
                    self.snakingPolarity[port] = calcSnakingPolarity(vecTargetDir2D, primaryAxis)
                
                snakingTime = float(time) - self.snakingStartTime[port]
                
                #rollCommand,pitchCommand = altDirHold(observableList, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000)
                rollCommand,pitchCommand = snaking(snakingTime,obs,vecTargetDir2D,mTargetAlt,self.snakingPolarity[port],sCycle=60.0,radSnakingAngle=75.0/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.IN):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                if(not primaryTargets[port] is None):
                    mReferencePoint2D = (primaryTargetPos[0],0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 60/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
            elif(nextState == STATE.PENETRATING):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
            
            
            #Secondaryへの射撃可否判定
            if(not launch):
                if((not secondaryTrack is None) and distanceToSecondaryTarget < M_SECONDARY_TARGET_LAUNCH_RANGE and radAspectAngle3DOfSecnodaryTarget > 120*np.pi/180 and radDirectionToSecondaryTarget < 60*np.pi/180  and (not self.fireControl.isTargetShot(time,observableList,secondaryTrack,MPS_MISSILE_ALIVE_CRITERIA)) and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire to Secondary Target, target: {}".format(str(secondaryTrack["truth"])))
                    #print(timeToStr(time),":",names[port]," RequestFire to Secondary Target, target: {}".format(str(secondaryTrack["truth"])))
                    launch,target = self.fireControl.requestFire(time, observableList, port, secondaryTrack)
            
            
            if(self.primaryTargets[port] != primaryTargets[port]):
                self.util.printLog(time, names[port], "PrimaryTargetChanged "+str(self.primaryTargets[port])+"->"+str(primaryTargets[port]))
                #print("{} : {} PrimaryTargetChanged ".format(timeToStr(time),names[port]),self.primaryTargets[port],"->",primaryTargets[port])
            self.primaryTargets[port] = primaryTargets[port]
            
            
            if(self.states[port] != nextState):
                self.util.printLog(time, names[port], str(self.states[port])+"->"+str(nextState)+"countdown = {:.1f}".format(self.outCountdowns[port]))
                #print("{} : {}".format(timeToStr(time),names[port]),self.states[port],"->",nextState,"countdonw = {:.1f}".format(self.outCountdowns[port]))
            # if(self.states[port] != nextState and nextState == STATE.PENETRATING):
            #     print("{:.1f} : {}".format(time,names[port]),self.states[port],"->",nextState)
            self.states[port] = nextState
            
            command = {
                    "motion":{
                        "pitch":pitchCommand,
                        "roll":rollCommand,
                        "throttle":throttle
                    },
                    "weapon":{
                        "launch":launch,
                        "target":target
                    }
                }
            
            commands[port] = command
        
        ##################################
        # Cross Fire 判定
        ##################################
        if((not observableList[0] is None) and (not observableList[1] is None)):
            blue1Pos = observableList[0]["motion"]["pos"]
            blue2Pos = observableList[1]["motion"]["pos"]
            
            for mainShooter,subShooter in ((0,1),(1,0)):
                if(commands[mainShooter]["weapon"]["launch"] == True and commands[subShooter]["weapon"]["launch"] == False):
                    crossfireTrack = commands[mainShooter]["weapon"]["target"]
                    crossfirePos = self.fireControl.getTrack(crossfireTrack["truth"]).lastFoundPos
                    blue1ToTargetDistance = calcDistance3d(blue1Pos,crossfirePos)
                    blue2ToTargetDistance = calcDistance3d(blue2Pos,crossfirePos)
                    radCrossfireAngle = calcRadAngle3Dof3Points(crossfirePos,blue1Pos,blue2Pos)
                    
                    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    # print("blue1ToTargetDistance:",blue1ToTargetDistance,", in range",blue1ToTargetDistance < M_CROSSFIRE_RANGE)
                    # print("blue2ToTargetDistance:",blue2ToTargetDistance,", in range",blue2ToTargetDistance < M_CROSSFIRE_RANGE)
                    # print("degCrossFireAngle:",radCrossfireAngle*180/np.pi,", in criteria",radCrossfireAngle > RAD_CROSSFIRE_ANGLE_LIMIT)
                    # print("shotBy:",self.fireControl.isTargetShotBy(time,subShooter,observableList,crossfireTrack))
                    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    
                    if(blue1ToTargetDistance < M_CROSSFIRE_RANGE and blue2ToTargetDistance < M_CROSSFIRE_RANGE and radCrossfireAngle > RAD_CROSSFIRE_ANGLE_LIMIT):
                        if(not self.fireControl.isTargetShotBy(time,subShooter,observableList,crossfireTrack,MPS_MISSILE_ALIVE_CRITERIA) and (observableList[subShooter]["weapon"]["launchable"])):
                            # print("")
                            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            self.util.printLog(time, names[subShooter], " RequestFire when CROSSFIRE, target: {}".format(str(crossfireTrack["truth"])))
                            #print(timeToStr(time),":",names[subShooter]," RequestFire when CROSSFIRE, target: {}".format(str(crossfireTrack["truth"])))
                            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            # print("")
                            launch,target = self.fireControl.requestFire(time, observableList, subShooter, crossfireTrack)
                            
                            commands[subShooter]["weapon"]["launch"] = launch
                            commands[subShooter]["weapon"]["target"] = target
        
        return commands
    
    def banzai(self,time,observableList,names,primaryTargets,primaryAxis=(0,1,0)):
        mNsOffsets = [20000,-20000]
        # mNsOffsets = [30000,-30000]
        
        commands = [None,None]
        
        
        for port in range(2):
            if(observableList[port] is None):
                if(self.isAlive[port]):
                    self.isAlive[port] = False
                    print("{} : {} Down ".format(timeToStr(time),names[port]))
                continue
            
            obs = observableList[port]
            
            myPos = obs["motion"]["pos"]
            myVel = obs["motion"]["vel"]
            radRoll,radPitch,radHeading =  QuaternionToRollPitchHeading(obs["motion"]["q"])
            
            if(not primaryTargets[port] is None):
                primaryTrack = self.fireControl.getTrack(primaryTargets[port]).track3D
            else:
                primaryTrack = None
            
            if(not primaryTargets[port] is None):
                primaryTargetFoundInThisStep = self.fireControl.getTrack(primaryTargets[port]).foundInThisStep
                primaryTargetPos = self.fireControl.getTrack(primaryTargets[port]).lastFoundPos
                primaryTargetVel = self.fireControl.getTrack(primaryTargets[port]).lastFoundVel
                
                distanceToPrimaryTarget = calcDistance3d(myPos,primaryTargetPos)
                vecDirectionToPrimaryTarget = calcDirection3d(myPos,primaryTargetPos)
                radAspectAngle3DOfPrimaryTarget = calcRadAspectAngle3D(myPos, primaryTargetPos, primaryTargetVel)
                radDirectionToPrimaryTarget = calcRadAspectAngle3D(primaryTargetPos,myPos,myVel)
            else:
                primaryTargetFoundInThisStep = False
                distanceToPrimaryTarget = np.inf
                vecDirectionToPrimaryTarget = primaryAxis
                radAspectAngle3DOfPrimaryTarget = 180
                radDirectionToPrimaryTarget = 0
            
            secondaryTarget = self.fireControl.getAnotherLiveTruth(primaryTargets[port])
            if(not secondaryTarget is None):
                secondaryTrack = self.fireControl.getTrack(secondaryTarget).track3D
            else:
                secondaryTrack = None
                
            if(not secondaryTarget is None):
                secondaryTargetFoundInThisStep = self.fireControl.getTrack(secondaryTarget).foundInThisStep
                secondaryTargetPos = self.fireControl.getTrack(secondaryTarget).lastFoundPos
                secondaryTargetVel = self.fireControl.getTrack(secondaryTarget).lastFoundVel
                
                distanceToSecondaryTarget = calcDistance3d(myPos,secondaryTargetPos)
                vecDirectionToSecondaryTarget = calcDirection3d(myPos, secondaryTargetPos)
                radAspectAngle3DOfSecnodaryTarget = calcRadAspectAngle3D(myPos, secondaryTargetPos, secondaryTargetVel)
                radDirectionToSecondaryTarget = calcRadAspectAngle3D(secondaryTargetPos,myPos,myVel)
            else:
                secondaryTargetFoundInThisStep = False
                distanceToSecondaryTarget = np.inf
                vecDirectionToSecondaryTarget = primaryAxis
                radAspectAngle3DOfSecondaryTarget = 180
                radDirectionToSecondaryTarget = 0
            
                
            ##############################
            # decide next state
            ##############################
            launch = False
            target = Track3D().to_json()
            
            if(self.states[port] == STATE.OUT_FROM_MWS):
                if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire when OUT_FROM_MWS, backward shot, target: {}".format(str(primaryTrack["truth"])))
                    launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                else:
                    self.snakingStartTime[port] = -1
                    nextState = STATE.EM_RECOVER1
                    
                    
            elif(self.states[port] == STATE.OUT_FROM_CRITICAL_RANGE):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                else:
                    self.outFromCriticalRangeStartTimes[port] = float(time)
                    nextState = STATE.OUT_KEEP
                    
                    
            elif(self.states[port] == STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE):
                if((not primaryTrack is None) and self.fireControl.isLaunchAwaiting(observableList,port)):
                    nextState = STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE
                else:
                    nextState = STATE.OUT_FROM_CRITICAL_RANGE
                    
                    
            elif(self.states[port] == STATE.OUT_KEEP):
                if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire when OUT_KEEP, backward shot, target: {}".format(str(primaryTrack["truth"])))
                    launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(float(time)-self.outFromCriticalRangeStartTimes[port] < S_OUT_KEEP_TIME):
                    nextState = STATE.OUT_KEEP
                else:
                    nextState = STATE.IN
                    
                    
            elif(self.states[port] == STATE.EM_RECOVER1):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(abs3d(obs["motion"]["vel"]) < MPS_VEL_RECOVER_CRITERIA or radPitch < -5/180*np.pi):
                    nextState = STATE.EM_RECOVER1
                else:
                    nextState = STATE.EM_RECOVER2
            elif(self.states[port] == STATE.EM_RECOVER2):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                elif(float(obs["motion"]["pos"][2]) > M_ALT_RECOVER_CRITERIA):
                    nextState = STATE.EM_RECOVER2
                else:
                    if(float(obs["motion"]["pos"][2]) > M_VERTICAL_IN_CRITERIA and radPitch>2.0*np.pi/180): #高度低ければ垂直にIN
                        nextState = STATE.IN
                    else:                                                       #高度高ければ水平にIN
                        nextState = STATE.HORIZONTAL_IN
                    self.outCountdowns[port] = np.inf
            elif(self.states[port] == STATE.HORIZONTAL_IN):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        #print(timeToStr(time),":",names[port]," RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                elif((not primaryTargets[port] is None) and (radDirectionToPrimaryTarget > 90/180*np.pi) or (radDirectionToPrimaryTarget < -90/180*np.pi)):
                    nextState = STATE.APPROACH_TARGET
                else:
                    nextState = STATE.HORIZONTAL_IN
                    
            elif(self.states[port] == STATE.APPROACH_TARGET):
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when APPROACH_TARGET->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        #print(timeToStr(time),":",names[port]," RequestFire when APPROACH_TARGET->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)

                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                    
                    
                elif((not primaryTrack is None) and self.outCountdowns[port] <= 3.0):
                    if(not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA) and distanceToPrimaryTarget < M_SHOTBACK_RANGE and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when APPROACH_TARGET->LAUNCH_AND_OUT_FROM_CRITICAL, target: {}".format(str(primaryTrack["truth"])))
                        #print(timeToStr(time),":",names[port]," RequestFire when APPROACH_TARGET->LAUNCH_AND_OUT_FROM_CRITICAL, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                        
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                    self.crankingStartTime[port] = float(time)
                    nextState = STATE.CRANKING
                        
                        
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                        
                        
                elif((not primaryTrack is None) and isHot(radAspectAngle3DOfPrimaryTarget) and distanceToPrimaryTarget < M_LAUNCH_RANGE_HOT and float(myPos[2])<(ALT_AT_IN+1000) and radDirectionToPrimaryTarget > 100*np.pi/180):
                    if(not self.fireControl.isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA) and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], "RequestFire when APPROACH_TARGET->CRANKING, target: {}".format(str(primaryTrack["truth"])))
                        #print(timeToStr(time),":",names[port],"RequestFire when APPROACH_TARGET->CRANKING, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                        
                        nextState = STATE.CRANKING
                        self.crankingStartTime[port] = float(time)
                    else:
                        #self.util.printLog(time, names[port], "Launch abort: targetShot :{}".format(self.fireControl._isTargetShot(time,observableList, primaryTrack)))
                        #print(timeToStr(time),":",names[port],"Launch abort: targetShot :{}".format(self.fireControl.isTargetShot(time,observableList, primaryTrack)))
                        
                        nextState = STATE.APPROACH_TARGET
                        
                        
                elif(not isHot(radAspectAngle3DOfPrimaryTarget) and distanceToPrimaryTarget < M_LAUNCH_RANGE_COLD):
                    nextState = STATE.APPROACH_TARGET
                    
                    if((not primaryTrack is None) and (not self.fireControl.isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)) and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], "RequestFire when APPROACH_TARGET->APPROACH_TARGET, target: {}".format(str(primaryTrack["truth"])))
                        #print(timeToStr(time),":",names[port],"RequestFire when APPROACH_TARGET->APPROACH_TARGET, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList,port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: targetShot :{}, PrimaryTrack :{}".format(self.fireControl._isTargetShot(time,observableList, primaryTrack,MPS_MISSILE_ALIVE_CRITERIA),primaryTargets[port]))
                        #print(timeToStr(time),":",names[port],"Launch abort: targetShot :{}, PrimaryTrack :{}".format(self.fireControl.isTargetShot(time,observableList, primaryTrack),primaryTargets[port]))
                        
                        
                elif(not primaryTargets[port] is None):
                    nextState = STATE.APPROACH_TARGET
                    
                    
                else:
                    nextState = STATE.IN
                    
            elif(self.states[port] == STATE.CRANKING):
                # if(isHot(radAspectAngle3DOfPrimaryTarget)):
                #     self.crankingStartTime[port] = float(time)
                
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    self.snakingStartTime[port] = -1
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when CRANKING->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        #print(timeToStr(time),":",names[port]," RequestFire when CRANKING->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.outCountdowns[port] <= -S_MINIMUM_CRANKING_TIME or self.outCountdowns[port] > 60):
                    nextState = STATE.IN
                    self.snakingStartTime[port] = -1
                    self.outCountdowns[port] = np.inf
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                    self.snakingStartTime[port] = -1
                elif((not isHot(radAspectAngle3DOfPrimaryTarget)) and (float(time)-self.crankingStartTime[port]) > S_MINIMUM_CRANKING_TIME):
                    nextState = STATE.IN
                    self.outCountdowns[port] = np.inf
                    self.snakingStartTime[port] = -1
                    self.outCountdowns[port] = np.inf
                elif(not primaryTargets[port] is None):
                    nextState = STATE.CRANKING
                else:
                    nextState = STATE.IN
                    self.snakingStartTime[port] = -1
                    
                    
            elif(self.states[port] == STATE.IN):   #STATE_IN
                if(isMwsActive(obs)):
                    nextState = STATE.OUT_FROM_MWS
                    if((not primaryTrack is None) and radAspectAngle3DOfPrimaryTarget > 60*np.pi/180 and not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS) and distanceToPrimaryTarget < M_LAUNCH_RANGE_MWS and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        #print(timeToStr(time),":",names[port]," RequestFire when IN->OUT_FROM_MWS, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA_MWS)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                elif(self.fireControl.isPenetratable(time, obs, primaryAxis)):
                    nextState = STATE.PENETRATING
                elif(not primaryTargets[port] is None):
                    if(int(obs["weapon"]["remMsls"]) > 0):
                        nextState = STATE.APPROACH_TARGET
                    else:
                        nextState = STATE.IN
                elif((not primaryTrack is None) and self.outCountdowns[port] <= 3.0):
                    if(not self.fireControl.isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA) and distanceToPrimaryTarget < M_SHOTBACK_RANGE and (obs["weapon"]["launchable"])):
                        self.util.printLog(time, names[port], " RequestFire when IN->CRANKING, target: {}".format(str(primaryTrack["truth"])))
                        #print(timeToStr(time),":",names[port]," RequestFire when APPROACH_TARGET->LAUNCH_AND_OUT_FROM_CRITICAL, target: {}".format(str(primaryTrack["truth"])))
                        launch,target = self.fireControl.requestFire(time, observableList, port, primaryTrack)
                    else:
                        self.util.printLog(time, names[port], "Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl._isTargetShot(time,observableList,primaryTrack,MPS_MISSILE_ALIVE_CRITERIA)))
                        #print(timeToStr(time),":",names[port],"Launch abort: aspect of target: {:.1f}, targetShot: {}".format(radAspectAngle3DOfPrimaryTarget*180/np.pi,self.fireControl.isTargetShot(time,observableList,primaryTrack,600)))
                    self.crankingStartTime[port] = float(time)
                    nextState = STATE.CRANKING
                else:
                    nextState = STATE.IN
                    
                    
            elif(self.states[port] == STATE.PENETRATING):
                nextState = STATE.PENETRATING
            
            
            
            #################################
            # calc maneuver from state
            #################################
            if(nextState == STATE.OUT_FROM_MWS):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                
                if(radAspectAngle3DOfPrimaryTarget > 120*np.pi/180 and distanceToPrimaryTarget < M_LAUNCH_RANGE_BACKWARD):
                    rollCommand,pitchCommand = outFromMws(time,obs,ALT_AT_OUT,vecTargetDir2D,ALT_AT_OUT,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000)
                else:
                    if(self.snakingStartTime[port] < 0):
                        if(isMwsActive(obs)):
                            mwsDir = meanMwsDir(obs)
                            self.snakingPolarity[port] = calcSnakingPolarity(-mwsDir, primaryAxis)
                        else:
                            self.snakingPolarity[port] = 1
                            
                        self.snakingStartTime[port] = float(time)
                    
                    snakingTime = float(time) - self.snakingStartTime[port]
                    #rollCommand,pitchCommand = outFromMws(time,observableList,ALT_AT_OUT,vecTargetDir2D,ALT_AT_OUT,radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000)
                    rollCommand,pitchCommand = outFromMwsWithSnaking(snakingTime,obs,ALT_AT_OUT,polarity=self.snakingPolarity[port],sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=vecTargetDir2D,mTargetAltAtIn=ALT_AT_IN,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                    #rollCommand,pitchCommand = outFromMwsWithBarrel(snakingTime,obs,ALT_AT_OUT,mAltAmplitude=-3000,polarity=self.snakingPolarity[port],sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=vecTargetDir2D,mTargetAltAtIn=ALT_AT_IN,radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-45/180*np.pi, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                
                throttle = 1.0
                
            elif(nextState == STATE.OUT_FROM_CRITICAL_RANGE):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_OUT
                
                # if(not primaryTargets[port] is None):
                #     if(float(myPos[0]) > float(primaryTargetPos[0])):
                #         mReferencePoint2D = (65000,0,0)
                #     else:
                #         mReferencePoint2D = (-65000,0,0)
                # else:
                #     mReferencePoint2D = (mNsOffsets[port],0,0)
                #
                # rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                RAD_OFF_ANGLE = 80*np.pi/180
                
                vecTargetDir = calcDirection3d(myPos,primaryTargetPos)
                radDirection2DToPrimaryTarget = calcRadDirectionDifference2D(vecTargetDir,primaryAxis)
                if(radDirection2DToPrimaryTarget > 0):
                    polarity = -1
                else:
                    polarity = 1
                
                r = R.from_quat([0, 0,np.sin(polarity*RAD_OFF_ANGLE/2),np.cos(polarity*RAD_OFF_ANGLE/2)])
                
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                vecTargetDir2D = r.apply(vecTargetDir2D)
                
                mTargetAlt = ALT_AT_GUIDANCE
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=10*np.pi/180, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                
                throttle = 1.0
                
            elif(nextState == STATE.OUT_KEEP):
                RAD_OFF_ANGLE = 80*np.pi/180
                
                vecTargetDir = calcDirection3d(myPos,primaryTargetPos)
                radDirection2DToPrimaryTarget = calcRadDirectionDifference2D(vecTargetDir,primaryAxis)
                if(radDirection2DToPrimaryTarget > 0):
                    polarity = -1
                else:
                    polarity = 1
                
                r = R.from_quat([0, 0,np.sin(polarity*RAD_OFF_ANGLE/2),np.cos(polarity*RAD_OFF_ANGLE/2)])
                
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                vecTargetDir2D = r.apply(vecTargetDir2D)
                
                mTargetAlt = ALT_AT_GUIDANCE
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=10*np.pi/180, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                
                throttle = 1.0
                
            elif(nextState == STATE.EM_RECOVER1):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                if(not primaryTargets[port] is None):
                    if(float(myPos[0]) > float(primaryTargetPos[0])):
                        mReferencePoint2D = (65000,0,0)
                    else:
                        mReferencePoint2D = (-65000,0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=2/180*np.pi, radDiveAngleLimit=-2/180*np.pi, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.EM_RECOVER2):
                vecTargetDir2D = (-primaryAxis[0],-primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                if(not primaryTargets[port] is None):
                    if(float(myPos[0]) > float(primaryTargetPos[0])):
                        mReferencePoint2D = (65000,0,0)
                    else:
                        mReferencePoint2D = (-65000,0,0)
                else:
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                
                rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 45/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.HORIZONTAL_IN):
                RAD_OFF_ANGLE = 80*np.pi/180
                
                radDirection2DToPrimaryTarget = calcRadDirectionDifference2D(myVel,vecDirectionToPrimaryTarget)
                if(radDirection2DToPrimaryTarget > 0):
                    polarity = 1
                else:
                    polarity = -1
                
                r = R.from_quat([0, 0,np.sin(polarity*RAD_OFF_ANGLE/2),np.cos(polarity*RAD_OFF_ANGLE/2)])
                
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                vecTargetDir2D = r.apply(vecTargetDir2D)
                
                mTargetAlt = ALT_AT_HORIZONTAL_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=10*np.pi/180, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.LAUNCH_AND_OUT_FROM_CRITICAL_RANGE):
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=0, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                       
            elif(nextState == STATE.APPROACH_TARGET):
                if(primaryTargetFoundInThisStep and (radAspectAngle3DOfPrimaryTarget < -np.pi/2 or radAspectAngle3DOfPrimaryTarget > np.pi/2)):#敵機が見えており、Hotの場合
                    interceptPoint = calcInterceptPoint2D2(myPos,primaryTargetPos,primaryTargetVel)
                    vecTargetDir2D = calcDirection3d(myPos,interceptPoint)
                    vecTargetDir2D = clipDirection(vecTargetDir2D,vecDirectionToPrimaryTarget,85/180*np.pi)
                elif(primaryTargetFoundInThisStep): #敵機が見えており、Cold
                    npPrimaryTargetPos = np.array([float(primaryTargetPos[0]),float(primaryTargetPos[1]),float(primaryTargetPos[2])])
                    npPrimaryTargetVel = np.array([float(primaryTargetVel[0]),float(primaryTargetVel[1]),float(primaryTargetVel[2])])
                    
                    #estimatedPrimaryTargetPos = npPrimaryTargetPos + S_COLD_TARGET_LEAD_AHEAD_TIME * npPrimaryTargetVel
                    primaryTargetPosOffset = np.array([primaryAxis[0],primaryAxis[1],0]) * distanceToPrimaryTarget*COLD_TARGET_OFFSET_RATIO
                    estimatedPrimaryTargetPos = npPrimaryTargetPos - primaryTargetPosOffset + S_COLD_TARGET_LEAD_AHEAD_TIME * npPrimaryTargetVel
                    
                    vecTargetDir2D = calcDirection3d(myPos ,estimatedPrimaryTargetPos)
                    vecTargetDir2D = clipDirection(vecTargetDir2D,vecDirectionToPrimaryTarget,85/180*np.pi)
                else:           #敵機の最終発見地点へ
                    vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                
                if(float(myPos[0]) > 73000 and vecTargetDir2D[0] > 0):
                    vecTargetDir2D[0] = 0
                
                if(float(myPos[0]) < -73000 and vecTargetDir2D[0] < 0):
                    vecTargetDir2D[0] = 0
                
                # if(self.outCountdowns[port] > 0 and self.outCountdowns[port]*MPS_MAXSPEED/2 < (-float(myPos[2])+ALT_AT_GUIDANCE)):
                #     mTargetAlt = ALT_AT_GUIDANCE
                # else:
                #     mTargetAlt = altAtIn
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
                
            elif(nextState == STATE.CRANKING):
                vecTargetDir2D = calcDirection3d(myPos ,primaryTargetPos)
                mTargetAlt = ALT_AT_GUIDANCE
                
                if(self.snakingStartTime[port] < 0):
                    self.snakingStartTime[port] = float(time)
                    self.snakingPolarity[port] = calcSnakingPolarity(vecTargetDir2D, primaryAxis)
                
                snakingTime = float(time) - self.snakingStartTime[port]
                
                #rollCommand,pitchCommand = altDirHold(observableList, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000)
                rollCommand,pitchCommand = snaking(snakingTime,obs,vecTargetDir2D,mTargetAlt,self.snakingPolarity[port],sCycle=60.0,radSnakingAngle=60.0/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_MWS, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_MWS, altGain=1/3000,pitchGain=PITCH_GAIN_MWS,pitchLimit=PITCH_LIMIT_MWS)
                throttle = 1.0
                
            elif(nextState == STATE.IN):
                if(not primaryTargets[port] is None):
                    if(int(obs["weapon"]["remMsls"]) > 0):
                        vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                        mReferencePoint2D = (primaryTargetPos[0],0,0)
                        rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,ALT_AT_IN,radPerMCourseGain = 60/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                    else:
                        M_ISOLATION_RANGE = 40000
                        if(np.abs(float(primaryTargetPos[0]) - float(myPos[0])) > M_ISOLATION_RANGE): #南北に十分に離れている場合
                            vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                        else:
                            if(float(primaryTargetPos[0]) < float(myPos[0])):
                                isolatePos = np.array([float(primaryTargetPos[0])+M_ISOLATION_RANGE,float(primaryTargetPos[1]),float(primaryTargetPos[2])])
                                if(isolatePos[0] > 70000):
                                    isolatePos[0] = 70000
                                
                                vecTargetDir2D = calcDirection3d(myPos ,isolatePos)
                            else:
                                isolatePos = np.array([float(primaryTargetPos[0])-M_ISOLATION_RANGE,float(primaryTargetPos[1]),float(primaryTargetPos[2])])
                                if(isolatePos[0] < -70000):
                                    isolatePos[0] = -70000
                                
                                vecTargetDir2D = calcDirection3d(myPos ,isolatePos)
                        
                        rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, ALT_AT_IN, radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                else:
                    vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                    mReferencePoint2D = (mNsOffsets[port],0,0)
                    rollCommand,pitchCommand = courseAltHold(obs,vecTargetDir2D,mReferencePoint2D,ALT_AT_IN,radPerMCourseGain = 60/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
            elif(nextState == STATE.PENETRATING):
                vecTargetDir2D = (primaryAxis[0],primaryAxis[1],0)
                mTargetAlt = ALT_AT_IN
                
                rollCommand,pitchCommand = altDirHold(obs, vecTargetDir2D, mTargetAlt, radClimbAngleLimit=RAD_CLIMB_ANGLE_LIMIT_NORMAL, radDiveAngleLimit=RAD_DIVE_ANGLE_LIMIT_NORMAL, altGain=1/3000,pitchGain=PITCH_GAIN_NORMAL,pitchLimit=PITCH_LIMIT_NORMAL)
                throttle = 1.0
            
            
            #Secondaryへの射撃可否判定
            if(not launch):
                if((not secondaryTrack is None) and distanceToSecondaryTarget < M_SECONDARY_TARGET_LAUNCH_RANGE and radAspectAngle3DOfSecnodaryTarget > 120*np.pi/180 and radDirectionToSecondaryTarget < 60*np.pi/180  and (not self.fireControl.isTargetShot(time,observableList,secondaryTrack,MPS_MISSILE_ALIVE_CRITERIA)) and (obs["weapon"]["launchable"])):
                    self.util.printLog(time, names[port], " RequestFire to Secondary Target, target: {}".format(str(secondaryTrack["truth"])))
                    #print(timeToStr(time),":",names[port]," RequestFire to Secondary Target, target: {}".format(str(secondaryTrack["truth"])))
                    launch,target = self.fireControl.requestFire(time, observableList, port, secondaryTrack)
            
            
            if(self.primaryTargets[port] != primaryTargets[port]):
                self.util.printLog(time, names[port], "PrimaryTargetChanged "+str(self.primaryTargets[port])+"->"+str(primaryTargets[port]))
                #print("{} : {} PrimaryTargetChanged ".format(timeToStr(time),names[port]),self.primaryTargets[port],"->",primaryTargets[port])
            self.primaryTargets[port] = primaryTargets[port]
            
            
            if(self.states[port] != nextState):
                self.util.printLog(time, names[port], str(self.states[port])+"->"+str(nextState)+"countdown = {:.1f}".format(self.outCountdowns[port]))
                
            self.states[port] = nextState
            
            
            command = {
                    "motion":{
                        "pitch":pitchCommand,
                        "roll":rollCommand,
                        "throttle":throttle
                    },
                    "weapon":{
                        "launch":launch,
                        "target":target
                    }
                }
            
            commands[port] = command
        
        ##################################
        # Cross Fire 判定
        ##################################
        if((not observableList[0] is None) and (not observableList[1] is None)):
            blue1Pos = observableList[0]["motion"]["pos"]
            blue2Pos = observableList[1]["motion"]["pos"]
            
            for mainShooter,subShooter in ((0,1),(1,0)):
                if(commands[mainShooter]["weapon"]["launch"] == True and commands[subShooter]["weapon"]["launch"] == False):
                    crossfireTrack = commands[mainShooter]["weapon"]["target"]
                    crossfirePos = self.fireControl.getTrack(crossfireTrack["truth"]).lastFoundPos
                    blue1ToTargetDistance = calcDistance3d(blue1Pos,crossfirePos)
                    blue2ToTargetDistance = calcDistance3d(blue2Pos,crossfirePos)
                    radCrossfireAngle = calcRadAngle3Dof3Points(crossfirePos,blue1Pos,blue2Pos)
                    
                    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    # print("blue1ToTargetDistance:",blue1ToTargetDistance,", in range",blue1ToTargetDistance < M_CROSSFIRE_RANGE)
                    # print("blue2ToTargetDistance:",blue2ToTargetDistance,", in range",blue2ToTargetDistance < M_CROSSFIRE_RANGE)
                    # print("degCrossFireAngle:",radCrossfireAngle*180/np.pi,", in criteria",radCrossfireAngle > RAD_CROSSFIRE_ANGLE_LIMIT)
                    # print("shotBy:",self.fireControl.isTargetShotBy(time,subShooter,observableList,crossfireTrack))
                    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    
                    if(blue1ToTargetDistance < M_CROSSFIRE_RANGE and blue2ToTargetDistance < M_CROSSFIRE_RANGE and radCrossfireAngle > RAD_CROSSFIRE_ANGLE_LIMIT):
                        if(not self.fireControl.isTargetShotBy(time,subShooter,observableList,crossfireTrack,MPS_MISSILE_ALIVE_CRITERIA) and (observableList[subShooter]["weapon"]["launchable"])):
                            # print("")
                            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            self.util.printLog(time, names[subShooter], " RequestFire when CROSSFIRE, target: {}".format(str(crossfireTrack["truth"])))
                            #print(timeToStr(time),":",names[subShooter]," RequestFire when CROSSFIRE, target: {}".format(str(crossfireTrack["truth"])))
                            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            # print("")
                            launch,target = self.fireControl.requestFire(time, observableList, subShooter, crossfireTrack)
                            
                            commands[subShooter]["weapon"]["launch"] = launch
                            commands[subShooter]["weapon"]["target"] = target
        
        return commands