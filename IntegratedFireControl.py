'''
Created on 2022/01/23

@author: sa
'''
import numpy as np
from .MyUtil import *
from ASRCAISim1.libCore import *

S_TIMEOUT = 1200
MPS_MINIMUM_LIVE_MISSILE_VELOCITY = 500
S_MISSILE_ACL_TIME = 4.0

MPS_MAXSPEED = 500
M_RADAR_RANGE = 100000
M_SPLASH_RANGE = 300

M_ABREAST_CRITERIA = 10000

class Track():
    def __init__(self,foundBy,lastFoundTime,lastFoundPos,lastFoundVel,isAlive,lastShotTime=[0,0],track3D = Track3D().to_json(), foundInThisStep = False):
        self.foundBy = foundBy
        self.lastFoundTime = lastFoundTime
        self.lastFoundPos = lastFoundPos
        self.lastFoundVel = lastFoundVel
        self.isAlive = isAlive
        self.lastShotTime = [0,0]
        self.track3D = track3D
        self.foundInThisStep = foundInThisStep
        
class IntegratedFireControl():
    def __init__(self):
        self.trackFile = {}
        self.launchAwaitingId = [-1,-1]
        self.launchAwaitingTargets = [
                [None]*10,
                [None]*10
            ]
        self.launchAwaitingTime = [
                [-1]*10,
                [-1]*10
            ]
    
    def reset(self):
        self.trackFile = {}
        self.launchAwaitingId = [-1,-1]
    
    def _updateFoundTarget(self,time,port,track3D):
        if(str(track3D["truth"]) in self.trackFile):
            tmpTrack = self.trackFile[str(track3D["truth"])]
            tmpTrack.isAlive = True
            tmpTrack.foundBy = port
            tmpTrack.lastFoundTime = time
            tmpTrack.lastFoundPos = track3D["pos"]
            tmpTrack.lastFoundVel = track3D["vel"]
            tmpTrack.track3D = track3D
            tmpTrack.foundInThisStep = True
        else:
            tmpTrack = Track(port,time,track3D["pos"],track3D["vel"],True,track3D=track3D,foundInThisStep=True)
            
        self.trackFile[str(track3D["truth"])] = tmpTrack 
    
    def updateTrack(self,time,observableList): #observableList: array of observableList. if not available, None
        for key,track in self.trackFile.items():
            track.foundInThisStep = False
            track.track3D = None
        
        for port in range(2):
            if(not observableList[port] is None):
                for track3D in observableList[port]["sensor"]["radar"]["track"]:
                    self._updateFoundTarget(time,port,track3D)
                    
        #撃墜確認
        for truth,target in self.trackFile.items():
            if(target.isAlive and target.lastFoundTime != time):    #alive, but lost
                for port in range(2):
                    if(not observableList[port] is None):
                        myPos = observableList[port]["motion"]["pos"]
                        myVel = observableList[port]["motion"]["vel"]
                        targetPos = target.lastFoundPos
                        radAspectToTarget = calcRadAspectAngle3D(targetPos,myPos,myVel)
                        mDistanceToTarget = calcDistance3d(myPos,targetPos)
                        mTargetMovableRange = MPS_MAXSPEED * (float(time)-float(target.lastFoundTime))
                        
                        if(radAspectToTarget > np.pi/2 and  #sensor is directed to target
                           mDistanceToTarget + mTargetMovableRange < M_RADAR_RANGE and #target is in radar range
                           mTargetMovableRange < mDistanceToTarget * np.sin(radAspectToTarget-np.pi/2)): #target is in radar covarage
                            target.isAlive = False
        
        #ミサイルが目標と十分接近したか？
        for port in range(2):
            if(not observableList[port] is None):
                for missile in observableList[port]["weapon"]["missiles"]:
                    if(missile["isAlive"] and missile["hasLaunched"]):
                        npMissilePos = np.array([float(missile["motion"]["pos"][0]),float(missile["motion"]["pos"][1]),float(missile["motion"]["pos"][2])])
                        npTargetPos = np.array([float(missile["target"]["pos"][0]),float(missile["target"]["pos"][1]),float(missile["target"]["pos"][2])])
                        npMissileVel = np.array([float(missile["motion"]["vel"][0]),float(missile["motion"]["vel"][1]),float(missile["motion"]["vel"][2])])
                        npTargetVel = np.array([float(missile["target"]["vel"][0]),float(missile["target"]["vel"][1]),float(missile["target"]["vel"][2])])
                        
                        missilePosInNextFrame = npMissilePos + npMissileVel*0.1
                        targetPosInNextFrame = npTargetPos + npTargetVel*0.1
                        
                        mMissDistance = calcDistance3d(missilePosInNextFrame,targetPosInNextFrame)
                        if(mMissDistance < M_SPLASH_RANGE):
                            #print("Splash ",str(truth)," confirmed by downlink",mMissDistance)
                            self.trackFile[str(missile["target"]["truth"])].isAlive=False
            
    def _updateFireControl(self,time,port,track3D):
        if(str(track3D["truth"]) in self.trackFile):
            tmpTrack = self.trackFile[str(track3D["truth"])]
            tmpTrack.lastShotTime[port] = time
            
            self.trackFile[str(track3D["truth"])] = tmpTrack
        else:
            print("!!!!!!!Debug launched before found!!!!!!")
            print("port:",port)
            print("track3D",track3D)
            
            
    def requestFire(self,time,observableList,port,targetTrack):
        if(observableList[port] is None):
            launch = False
            target = Track3D().to_json()
            
            return launch,target
        
        nextMissileId = int(observableList[port]["weapon"]["nextMsl"])
        
        if(not observableList[port]["weapon"]["launchable"] or nextMissileId >= 10):
            launch = False
            target = Track3D().to_json()
            
            return launch,target
        
        launch = True
        target = targetTrack
        self._updateFireControl(time, port, target)
        
        self.launchAwaitingId[port] = nextMissileId
        self.launchAwaitingTargets[port][nextMissileId] = target["truth"]
        self.launchAwaitingTime[port][nextMissileId] = float(time)
        
        return launch,target
    
    def getTimeFromLastLaunch(self,time,truth):#time from launch by any ally
        truth = str(truth)
        if(truth in self.trackFile):
            t0 = float(self.trackFile[truth].lastShotTime[0])
            t1 = float(self.trackFile[truth].lastShotTime[1])
            
            lastShotTime = np.max(t0,t1)
            
            return float(time) - lastShotTime
        
        else:
            return S_TIMEOUT
    
    def getTimeFromLastLaunchBy(self,time,port,truth):
        truth = str(truth)
        if(truth in self.trackFile):
            return float(time) - self.trackFile[truth].lastShotTime[port]
        
        else:
            return S_TIMEOUT
    
    def isLaunchAwaiting(self,observableList,port):
        if(self.launchAwaitingId[port] < 0):
            return False
        elif(self.launchAwaitingId[port] >= 10):
            return False
        else:
            nextMissile = observableList[port]["weapon"]["missiles"][self.launchAwaitingId[port]]
            if(nextMissile["isAlive"] and nextMissile["hasLaunched"]):
                return False
            elif(not nextMissile["isAlive"]):
                return False
            else:
                return True
            
    def _isTargetShotBy_bak(self,time,port,observableList,targetTrack3D,mpsMinimumLiveMissileVelecity = MPS_MINIMUM_LIVE_MISSILE_VELOCITY):
        if(targetTrack3D is None):  #不正なターゲット
            return False,"invalid target",0,0,0.0
        
        if(observableList[port] is None):   #対象味方機が被撃墜済
            return False,"element not alive",0,0,0.0
        
        for missileId in range(10):
            missiles = observableList[port]["weapon"]["missiles"]
            if(not self.launchAwaitingTargets[port][missileId] is None): #ターゲット割当済
                if(missiles[missileId]["isAlive"] and (not missiles[missileId]["hasLaunched"])):#未発射かつ発射割当済
                    if((float(time) - self.launchAwaitingTime[port][missileId]) < 3.0 and (str(self.launchAwaitingTargets[port][missileId]) == str(targetTrack3D["truth"]))):    #現在時刻が発射要求時刻より3秒以上経過していれば発射失敗->割当済ではない
                        return True,"assigned, not launched",port,missileId,float(missiles[missileId]["launchedT"])
                elif(missiles[missileId]["isAlive"] and missiles[missileId]["hasLaunched"]):#発射済かつ残速度あり
                    if(str(missiles[missileId]["target"]["truth"]) == str(targetTrack3D["truth"])):
                        npVel = np.array([float(missiles[missileId]["motion"]["vel"][0]),float(missiles[missileId]["motion"]["vel"][1]),float(missiles[missileId]["motion"]["vel"][2])])
                        velMag = np.linalg.norm(npVel,ord=2)
                        if(velMag > mpsMinimumLiveMissileVelecity):
                            return True,"launched, has speed",port,missileId,float(missiles[missileId]["launchedT"])
                        if((float(time)-float(missiles[missileId]["launchedT"])) < S_MISSILE_ACL_TIME):
                            return True,"launched, accel",port,missileId,float(missiles[missileId]["launchedT"])
                        
        
        return False,"no matching",0,0,0.0
    def _isTargetShotBy(self,time,port,observableList,targetTrack3D,mpsMinimumLiveMissileVelecity = MPS_MINIMUM_LIVE_MISSILE_VELOCITY):
        if(targetTrack3D is None):  #不正なターゲット
            return False,"invalid target",0,0,0.0
        
        if(observableList[port] is None):   #対象味方機が被撃墜済
            return False,"element not alive",0,0,0.0
        
        for missileId in range(10):
            missiles = observableList[port]["weapon"]["missiles"]
            if(not self.launchAwaitingTargets[port][missileId] is None): #ターゲット割当済
                if(missiles[missileId]["isAlive"] and (not missiles[missileId]["hasLaunched"])):#未発射かつ発射割当済
                    if((float(time) - self.launchAwaitingTime[port][missileId]) < 3.0 and (str(self.launchAwaitingTargets[port][missileId]) == str(targetTrack3D["truth"]))):    #現在時刻が発射要求時刻より3秒以上経過していれば発射失敗->割当済ではない
                        return True,"assigned, not launched",port,missileId,float(missiles[missileId]["launchedT"])
                elif(missiles[missileId]["isAlive"] and missiles[missileId]["hasLaunched"]):#発射済かつ残速度あり
                    if(str(missiles[missileId]["target"]["truth"]) == str(targetTrack3D["truth"])):
                        npVel = np.array([float(missiles[missileId]["motion"]["vel"][0]),float(missiles[missileId]["motion"]["vel"][1]),float(missiles[missileId]["motion"]["vel"][2])])
                        velMag = np.linalg.norm(npVel,ord=2)
                        if(velMag > mpsMinimumLiveMissileVelecity):
                            return True,"launched, has speed",port,missileId,float(missiles[missileId]["launchedT"])
                        if((float(time)-float(missiles[missileId]["launchedT"])) < S_MISSILE_ACL_TIME):
                            return True,"launched, accel",port,missileId,float(missiles[missileId]["launchedT"])
                        
        #DEBUG FOR UNNECESSARY LAUNCH
        # print("target {:}".format(str(targetTrack3D["truth"])))
        # print("missile alive criteria: {:.1f}".format(mpsMinimumLiveMissileVelecity))
        # for missileId in range(10):
        #     if(not missiles[missileId]["isAlive"]):
        #         print("{:}-{:} alive: {:}".format(port,missileId,missiles[missileId]["isAlive"]))
        #     else:
        #         npVel = np.array([float(missiles[missileId]["motion"]["vel"][0]),float(missiles[missileId]["motion"]["vel"][1]),float(missiles[missileId]["motion"]["vel"][2])])
        #         velMag = np.linalg.norm(npVel,ord=2)
        #         print("{:}-{:} alive: {:}, hasLaunched {:}, target {:}, match {:}, vel {:.1f},vel alive {:}, awaiting {:}".format(port,missileId,missiles[missileId]["isAlive"],missiles[missileId]["hasLaunched"],str(missiles[missileId]["target"]["truth"]),str(missiles[missileId]["target"]["truth"]) == str(targetTrack3D["truth"]),velMag,velMag > mpsMinimumLiveMissileVelecity, self.launchAwaitingTargets[port][missileId]))
        
        return False,"no matching",0,0,0.0
    
    def _isTargetShot(self,time,observableList,targetTrack3D,mpsMinimumLiveMissileVelecity = MPS_MINIMUM_LIVE_MISSILE_VELOCITY):
        for port in [0,1]:
            targetShot,shotInfo,shotPort,missileId,launchTime = self._isTargetShotBy(time,port,observableList,targetTrack3D,mpsMinimumLiveMissileVelecity)
            if(targetShot):
                return True,shotInfo,shotPort,missileId,launchTime
        
        return False,"no matching",0,0,0.0
    
    def isTargetShotBy(self,time,port,observableList,targetTrack3D,mpsMinimumLiveMissileVelecity = MPS_MINIMUM_LIVE_MISSILE_VELOCITY):
        return self._isTargetShotBy(time, port, observableList, targetTrack3D, mpsMinimumLiveMissileVelecity)[0]
    
    def isTargetShot(self,time,observableList,targetTrack3D,mpsMinimumLiveMissileVelecity = MPS_MINIMUM_LIVE_MISSILE_VELOCITY):
        for port in [0,1]:
            if(self.isTargetShotBy(time,port,observableList,targetTrack3D,mpsMinimumLiveMissileVelecity)):
                return True
        
        return False
                        
    def isPenetratable(self,time,observable,primaryAxis):
        M_MARGIN = 27000
        PENETRATION_RANGE = 100000
        
        if(len(self.trackFile) != 2):   #敵が両方見つかっていない場合は突破不可
            return False
        
        
        for key,track in self.trackFile.items():
            if(track.isAlive):
                myPos = np.array([float(observable["motion"]["pos"][0]),float(observable["motion"]["pos"][1]),float(observable["motion"]["pos"][2])])
                npPrimaryAxis = np.array(primaryAxis)
                myVel = npPrimaryAxis * MPS_MAXSPEED    #全力で敵陣に向かう想定
                targetPos = np.array([float(track.lastFoundPos[0]),float(track.lastFoundPos[1]),float(track.lastFoundPos[2])])
                movableRange = MPS_MAXSPEED * (float(time) - float(track.lastFoundTime))
                
                #自分の方が敵陣に近いか判定
                estimatedTargetPos = targetPos - movableRange * npPrimaryAxis #最も突破方向に移動された場合
                if(myPos[1]*primaryAxis[1] < estimatedTargetPos[1]*primaryAxis[1]*(-1)):
                    return False
                
                
                #逃げ切れるか判定
                estimatedTargetPos = targetPos + movableRange * npPrimaryAxis #最も接近された場合
                
                myPosWithMargin = myPos - M_MARGIN * npPrimaryAxis
                
                hotAspect = isHot(calcRadAspectAngle2D(estimatedTargetPos,myPosWithMargin,myVel))
                if(not hotAspect):  #ミサイルが追いつける距離の補正入れた状態でcoldならpass
                    continue
                
                interceptPoint = calcInterceptPoint2D2(estimatedTargetPos,myPosWithMargin,myVel)
                if(interceptPoint[1]*primaryAxis[1] < PENETRATION_RANGE):   #敵陣手前でインターセプトされる可能性あり。
                    return False
                    
        return True
    
    def getTrack(self,truth):
        if(truth is None):
            return None
        else:
            return self.trackFile[str(truth)]
    
    def getAnotherLiveTruth(self,truth):
        if(truth is None):
            return None
        elif(len(self.trackFile) == 1):
            return None
        
        for key,val in self.trackFile.items():
            if(str(key) != str(truth) and val.isAlive):
                return key

        return None
    
    def getTargetsRemain(self):#未発見のターゲットは生存とみなす
        count = 2
        for key,val in self.trackFile.items():
            if(not val.isAlive):
                count -= 1
        return count
    
    def getNumTargetsFound(self):
        return len(self.trackFile)
    
    def _assign2v2_northSouth(self,observableList): #敵味方どちらも2機生存を前提とする
        truth0 = list(self.trackFile.keys())[0]
        truth1 = list(self.trackFile.keys())[1]
        if(float(self.trackFile[truth0].lastFoundPos[0]) > float(self.trackFile[truth1].lastFoundPos[0])):
            northTarget = truth0
            southTarget = truth1
        else:
            northTarget = truth1
            southTarget = truth0
        
        if(float(observableList[0]["motion"]["pos"][0]) > float(observableList[1]["motion"]["pos"][0])): #flight1 is north
            return northTarget,southTarget
        else:
            return southTarget,northTarget
        
    def _assign2v2_nearestFirst(self,observableList): #敵味方どちらも2機生存を前提とする
        distance_blue1_red1 = calcDistance3d(observableList[0]["motion"]["pos"],list(self.trackFile.values())[0].lastFoundPos)
        distance_blue1_red2 = calcDistance3d(observableList[0]["motion"]["pos"],list(self.trackFile.values())[1].lastFoundPos)
        distance_blue2_red1 = calcDistance3d(observableList[1]["motion"]["pos"],list(self.trackFile.values())[0].lastFoundPos)
        distance_blue2_red2 = calcDistance3d(observableList[1]["motion"]["pos"],list(self.trackFile.values())[1].lastFoundPos)
        
        distanceList = np.array([distance_blue1_red1,distance_blue1_red2,distance_blue2_red1,distance_blue2_red2])
        minIndex = np.argmin(distanceList)
        
        truth0 = list(self.trackFile.keys())[0]
        truth1 = list(self.trackFile.keys())[1]
        if(minIndex == 0):
            return truth0,truth1
        elif(minIndex == 1):
            return truth1,truth0
        elif(minIndex == 2):
            return truth1,truth0
        else:
            return truth0,truth1
    
    def _assign2v2_LeadingTargetFirst(self,observableList,primaryAxis): #敵味方どちらも2機生存を前提とする。
        truth0 = list(self.trackFile.keys())[0]
        truth1 = list(self.trackFile.keys())[1]
        
        if(float(self.trackFile[truth0].lastFoundPos[1]) > float(self.trackFile[truth1].lastFoundPos[1])):
            eastTarget = truth0
            westTarget = truth1
        else:
            eastTarget = truth1
            westTarget = truth0
            
        if(primaryAxis[1] > 0): #RED
            leadingTarget = westTarget
            trailingTarget = eastTarget
        else:   #BLUE
            leadingTarget = eastTarget
            trailingTarget = westTarget
        
        targetPos = self.trackFile[truth0].lastFoundPos
        port0Pos = observableList[0]["motion"]["pos"]
        port0Vel = observableList[0]["motion"]["vel"]
        
        port0Distance = calcDistance3d(port0Pos,targetPos)
        port0Hot = isHot(calcRadAspectAngle3D(targetPos,port0Pos,port0Vel))
        
        port1Pos = observableList[1]["motion"]["pos"]
        port1Vel = observableList[1]["motion"]["vel"]
        
        port1Distance = calcDistance3d(port1Pos,targetPos)
        port1Hot = isHot(calcRadAspectAngle3D(targetPos,port1Pos,port1Vel))
        
        if(port0Hot and (not port1Hot)):
            return leadingTarget,trailingTarget
        elif((not port0Hot) and port1Hot):
            return trailingTarget,leadingTarget
        else:   #both ally hot or both ally cold
            if(port0Distance < port1Distance):
                return leadingTarget,trailingTarget
            else:
                return trailingTarget,leadingTarget
    
    def _assign2v2_trail_abreast(self,observableList,primaryAxis):
        truth0 = list(self.trackFile.keys())[0]
        truth1 = list(self.trackFile.keys())[1]
        if(np.abs(float(self.trackFile[truth0].lastFoundPos[0]) - float(self.trackFile[truth1].lastFoundPos[0])) < M_ABREAST_CRITERIA):
            #return self._assign2v2_nearestFirst(observableList)
            return self._assign2v2_LeadingTargetFirst(observableList,primaryAxis)
        else:
            return self._assign2v2_northSouth(observableList)
            
    def _assign2v2_minimamTotalDistance(self,observableList):
        distance_blue1_red1 = calcDistance3d(observableList[0]["motion"]["pos"],list(self.trackFile.values())[0].lastFoundPos)
        distance_blue1_red2 = calcDistance3d(observableList[0]["motion"]["pos"],list(self.trackFile.values())[1].lastFoundPos)
        distance_blue2_red1 = calcDistance3d(observableList[1]["motion"]["pos"],list(self.trackFile.values())[0].lastFoundPos)
        distance_blue2_red2 = calcDistance3d(observableList[1]["motion"]["pos"],list(self.trackFile.values())[1].lastFoundPos)
        
        straightMatchupDistance = np.sqrt(distance_blue1_red1) + np.sqrt(distance_blue2_red2)
        crossMathupDistance     = np.sqrt(distance_blue1_red2) + np.sqrt(distance_blue2_red1)
        
        
        truth0 = list(self.trackFile.keys())[0]
        truth1 = list(self.trackFile.keys())[1]
        if(straightMatchupDistance < crossMathupDistance):
            return truth0,truth1
        else:
            return truth1,truth0
    
    def assignPrimaryTarget(self,observableList,primaryAxis):
        if(observableList[1] is None): #ally is down
            if(len(self.trackFile) == 0):   #目標未発見
                return None,None
            elif(len(self.trackFile) == 1): #目標1機のみ発見
                truth = list(self.trackFile.keys())[0]
                if(self.trackFile[truth].isAlive):
                    return truth,None
                else:
                    return None,None
            else:   #目標2機発見済
                truth0 = list(self.trackFile.keys())[0]
                truth1 = list(self.trackFile.keys())[1]
                if(not self.trackFile[truth0].isAlive): #target0 down
                    return truth1,None
                elif(not self.trackFile[truth1].isAlive): #target1 down
                    return truth0,None
                else: #both targets alive
                    distanceToTarget0 = calcDistance3d(observableList[0]["motion"]["pos"],self.trackFile[truth0].lastFoundPos)
                    distanceToTarget1 = calcDistance3d(observableList[0]["motion"]["pos"],self.trackFile[truth1].lastFoundPos)
                    
                    if(distanceToTarget0 < distanceToTarget1):
                        return truth0,None
                    else:
                        return truth1,None
                    
        if(observableList[0] is None): #ally is down
            if(len(self.trackFile) == 0):   #目標未発見
                return None,None
            elif(len(self.trackFile) == 1): #目標1機のみ発見
                truth = list(self.trackFile.keys())[0]
                if(self.trackFile[truth].isAlive):
                    return None,truth
                else:
                    return None,None
            else:   #目標2機発見済
                truth0 = list(self.trackFile.keys())[0]
                truth1 = list(self.trackFile.keys())[1]
                if(not self.trackFile[truth0].isAlive): #target0 down
                    return None,truth1
                elif(not self.trackFile[truth1].isAlive): #target1 down
                    return None,truth0
                else: #both targets alive
                    distanceToTarget0 = calcDistance3d(observableList[1]["motion"]["pos"],self.trackFile[truth0].lastFoundPos)
                    distanceToTarget1 = calcDistance3d(observableList[1]["motion"]["pos"],self.trackFile[truth1].lastFoundPos)
                    
                    if(distanceToTarget0 < distanceToTarget1):
                        return None,truth0
                    else:
                        return None,truth1
                
        else:   #all ally are alive
            if(len(self.trackFile) == 0):       #目標未発見
                return None,None
            elif(len(self.trackFile) == 1):     #目標1機のみ発見
                truth = list(self.trackFile.keys())[0]
                if(self.trackFile[truth].isAlive):
                    return truth,truth
                else:
                    return None,None
            else:   #2機発見済
                truth0 = list(self.trackFile.keys())[0]
                truth1 = list(self.trackFile.keys())[1]
                if(not self.trackFile[truth0].isAlive): #target0 down
                    return truth1,truth1
                elif(not self.trackFile[truth1].isAlive): #target1 down
                    return truth0,truth0
                else:   #both targets alive
                    #return self._assign2v2_minimamTotalDistance(observableList)
                    return self._assign2v2_trail_abreast(observableList,primaryAxis)
                    #return self._assign2v2_northSouth(observableList)
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    