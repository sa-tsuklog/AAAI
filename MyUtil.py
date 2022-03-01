'''
Created on 2022/01/10

@author: sa
'''

import numpy as np
from scipy.spatial.transform import Rotation as R

class MyUtil():
    DEBUG_LOG = True
    def printLog(self,time,name,message):
        if(self.DEBUG_LOG):
            print(timeToStr(time),":",name,message)

def QuaternionToRollPitchHeading(q):
    r = R.from_quat([float(q[1]), float(q[2]),float(q[3]),float(q[0])])
    roll,pitch,heading = r.as_euler('xyz', degrees=False)
    return roll,pitch,heading

def ObservationToRollPitchHeading(observables):
    return QuaternionToRollPitchHeading(observables["motion"]["q"])

def abs3d(vec):
    npVec = np.array([float(vec[0]),float(vec[1]),float(vec[2])])
    return np.linalg.norm(npVec,ord=2)

def calcDistance3d(pos1,pos2):
    npPos1 = np.array([float(pos1[0]),float(pos1[1]),float(pos1[2])])
    npPos2 = np.array([float(pos2[0]),float(pos2[1]),float(pos2[2])])
    
    return np.linalg.norm((npPos1-npPos2),ord=2)
    
def calcDirection3d(pos1,pos2):
    npPos1 = np.array([float(pos1[0]),float(pos1[1]),float(pos1[2])])
    npPos2 = np.array([float(pos2[0]),float(pos2[1]),float(pos2[2])])
    
    return npPos2 - npPos1

def calcRadAngleOfDirections3D(direction1,direction2):
    npDirection1 = np.array([float(direction1[0]),float(direction1[1]),float(direction1[2])])
    npDirection2 = np.array([float(direction2[0]),float(direction2[1]),float(direction2[2])])
    
    npDirection1 = npDirection1 / np.linalg.norm(npDirection1,ord=2)
    npDirection2 = npDirection2 / np.linalg.norm(npDirection2,ord=2)
    
    
    cosAngle = np.dot(npDirection1,npDirection2)
    if(cosAngle < -1.0):
        cosAngle = -1.0
    elif(cosAngle > 1.0):
        cosAngle = 1.0
    
    radAngle = np.arccos(cosAngle)
    
    return radAngle

def calcRadAngle3Dof3Points(refPoint,p1,p2):
    dir1 = calcDirection3d(refPoint,p1)
    dir2 = calcDirection3d(refPoint,p2)
    
    return calcRadAngleOfDirections3D(dir1,dir2)

def calcInterceptPoint2D2(myPos,targetPos,targetVel):
    targetX = float(targetPos[0])
    targetY = float(targetPos[1])
    
    targetVecX = float(targetVel[0])
    targetVecY = float(targetVel[1])
    
    myX = float(myPos[0])
    myY = float(myPos[1])
    
    if(targetVecX != 0):
        mergePointX = (targetVecY*targetX*(targetY-myY) - targetVecX*targetY*(targetY-myY) + targetVecX/2*(targetX*targetX - myX*myX + targetY*targetY - myY*myY))  /  (targetVecX*(targetX-myX)+targetVecY*(targetY-myY))
        mergePointY = targetVecY/targetVecX * (mergePointX - targetX) + targetY
    else:
        mergePointX = targetX
        mergePointY = (-targetX*(targetX-myX) + 0.5*(targetX*targetX - myX*myX + targetY*targetY - myY*myY) ) / (targetY-myY)
    
    return np.array([mergePointX,mergePointY,0])

def calcInterceptPoint2D(observation,track):
    #calcRadAspectAngle2Dの結果が91〜180 or -91〜-180であること。
    
    targetX = float(track["pos"][0])
    targetY = float(track["pos"][1])
    
    targetVecX = float(track["vel"][0])
    targetVecY = float(track["vel"][1])
    
    myX = float(observation["motion"]["pos"][0])
    myY = float(observation["motion"]["pos"][1])
    
    if(targetVecX != 0):
        mergePointX = (targetVecY*targetX*(targetY-myY) - targetVecX*targetY*(targetY-myY) + targetVecX/2*(targetX*targetX - myX*myX + targetY*targetY - myY*myY))  /  (targetVecX*(targetX-myX)+targetVecY*(targetY-myY))
        mergePointY = targetVecY/targetVecX * (mergePointX - targetX) + targetY
    else:
        mergePointX = targetX
        mergePointY = (-targetX*(targetX-myX) + 0.5*(targetX*targetX - myX*myX + targetY*targetY - myY*myY) ) / (targetY-myY)
    
    #print(mergePointX,",",mergePointY)
    
    #a = np.array([targetX-mergePointX,targetY-mergePointY])
    #b = np.array([myX-mergePointX,myY-mergePointY])
    #print(np.linalg.norm(a,ord=2),",",np.linalg.norm(b,ord=2))
    
    return np.array([mergePointX,mergePointY,0])
    
def calcRadAspectAngle2D(myPos,targetPos,targetVel):
    #head on = 180deg, cold = 0deg
    npTargetVec = np.array([float(targetVel[0]),float(targetVel[1])])
    npDirection = np.array([float(targetPos[0])-float(myPos[0]),float(targetPos[1])-float(myPos[1])])
    
    npTargetVec = npTargetVec / np.linalg.norm(npTargetVec,ord=2)
    npDirection = npDirection / np.linalg.norm(npDirection,ord=2)
    
    radAspectAngle2D = np.arctan2(-np.cross(npTargetVec,npDirection),np.dot(npTargetVec,npDirection))
    
    #print("aspect angle",radAspectAngle2D*180/np.pi)
    return radAspectAngle2D

def calcRadAspectAngle3D(myPos,targetPos,targetVel):
    #head on = 180deg, cold = 0deg. no sign
    npTargetVec = np.array([float(targetVel[0]),float(targetVel[1]),float(targetVel[2])])
    npDirection = np.array([float(targetPos[0])-float(myPos[0]),float(targetPos[1])-float(myPos[1]),float(targetPos[2])-float(myPos[2])])
    
    npTargetVec = npTargetVec / np.linalg.norm(npTargetVec,ord=2)
    npDirection = npDirection / np.linalg.norm(npDirection,ord=2)
    
    radAspectAngle3D = np.arccos(np.dot(npTargetVec,npDirection))
    
    return radAspectAngle3D

def calcRadDirectionDifference2D(direction2D,referenceDirection2D):    
    npDirection2D = np.array([float(direction2D[0]),float(direction2D[1])])
    npReferenceDirection2D = np.array([float(referenceDirection2D[0]),float(referenceDirection2D[1])])
    
    npDirection2D = npDirection2D / np.linalg.norm(npDirection2D,ord=2)
    npReferenceDirection2D = npReferenceDirection2D / np.linalg.norm(npReferenceDirection2D,ord=2)
    
    radAspectAngle2D = np.arctan2(-np.cross(npDirection2D,npReferenceDirection2D),np.dot(npDirection2D,npReferenceDirection2D))
    
    return radAspectAngle2D

def clipDirection(direction2D,referenceDirection2D,radAngleLimit):
    radAngle = calcRadDirectionDifference2D(direction2D,referenceDirection2D)
    
    npReferenceDirection2D = np.array([float(referenceDirection2D[0]),float(referenceDirection2D[1]),float(referenceDirection2D[2])])
    if(radAngle < -radAngleLimit):
        rotation = R.from_quat([0,0,np.sin(-radAngleLimit/2),np.cos(-radAngleLimit/2)])
        direction2D = rotation.apply(npReferenceDirection2D)
        
    elif(radAngle > radAngleLimit):
        rotation = R.from_quat([0,0,np.sin(radAngleLimit/2),np.cos(radAngleLimit/2)])
        direction2D = rotation.apply(npReferenceDirection2D)

    return direction2D

def isHot(radAspectAngle,radCriteria=np.pi/2):
    if(-radCriteria < radAspectAngle and radAspectAngle < radCriteria):
        return False
    else:
        return True
    
def isMwsActive(observables):
    mwsTracks = observables["sensor"]["mws"]["track"]
    
    return len(mwsTracks) != 0

def meanMwsDir(observables):
    mwsTracks = observables["sensor"]["mws"]["track"]
    
    if(len(mwsTracks) == 0):
        print("!!!!Warning meanMwsDir called when MWS is not active")
    
    dir = np.array([0,0,0])
    for mwsTrack in mwsTracks:
        dir = dir + np.array([float(mwsTrack["dir"][0]),float(mwsTrack["dir"][1]),float(mwsTrack["dir"][2])])
        
    return dir

def _getCriticalRangeAspectCorrection(radAspectAngle3D,targetPos):
    height = -float(targetPos[2])
    heightCoeff = 1.70 + height * 0.25/3000
    
    correctionCoeff = np.cos((np.pi-radAspectAngle3D)/heightCoeff)
    if(correctionCoeff < 0.6):
        correctionCoeff = 0.6
    
    return correctionCoeff
    #return 1

def isInCriticalRange(myPos,targetPos,radAspectAngle3D): #自機高度3000mのみ対応. 1機のみ対応
    M_MARGIN = 10000
    CRITICAL_RANGE_SLOPE = 2.14
    CRITICAL_RANGE_OFFSET = 24800
    
    distance = calcDistance3d(myPos,targetPos)
    
    criticalRange = CRITICAL_RANGE_SLOPE * (-float(targetPos[2])) + CRITICAL_RANGE_OFFSET + M_MARGIN
    
    criticalRange = criticalRange * _getCriticalRangeAspectCorrection(radAspectAngle3D,targetPos)
    
    if(distance < criticalRange):
        return True
    else:
        return False

def tAAtCriticalRange(myPos,targetPos,sMargin=-3.0): #自機高度3000mのみ対応
    TA_TIME_SLOPE = 6.27e-4
    TA_TIME_OFFSET = -4.24
    TA_ALT_SLOPE = -2.0/9000
    
    distance = calcDistance3d(myPos,targetPos)
    
    tA = TA_TIME_SLOPE * distance + TA_ALT_SLOPE*(-float(targetPos[2])) + TA_TIME_OFFSET + sMargin
    
    return tA

def calcLaunchRangeCold(radAspectAngleOfTarget):
    M_OFFSET = 18000
    M_PER_RAD_SLOPE = 100 *180 / np.pi
    
    return M_OFFSET + M_PER_RAD_SLOPE * radAspectAngleOfTarget

def updateOutCountdown(countdown,myPos,closestTrack):
    countdown -= 0.1
    
    
    if(not closestTrack is None):
        radAspectAngle3D = calcRadAspectAngle3D(myPos,closestTrack["pos"],closestTrack["vel"])
        if(isInCriticalRange(myPos,closestTrack["pos"],radAspectAngle3D) and isHot(calcRadAspectAngle3D(myPos, closestTrack["pos"], closestTrack["vel"]),radCriteria=90*np.pi/180)):
            newTA = tAAtCriticalRange(myPos, closestTrack["pos"])
            if(newTA < countdown):
                countdown = newTA
    
    return countdown

def updateOutCountdownForAllTrack(countdown,myPos,primaryTrack,secondaryTrack):
    countdown -= 0.1
    updated = False
    updateSource = None
    distance = np.inf
    
    if(not primaryTrack is None):
        radAspectAngle3D = calcRadAspectAngle3D(myPos,primaryTrack["pos"],primaryTrack["vel"])
        if(isInCriticalRange(myPos,primaryTrack["pos"],radAspectAngle3D) and isHot(calcRadAspectAngle3D(myPos, primaryTrack["pos"], primaryTrack["vel"]),radCriteria=90*np.pi/180)):
            newTA = tAAtCriticalRange(myPos, primaryTrack["pos"])
            if(newTA < countdown):
                countdown = newTA
                updated = True
                updateSource = "primary target"
                distance = calcDistance3d(myPos,primaryTrack["pos"])
    
    if(not secondaryTrack is None):
        radAspectAngle3D = calcRadAspectAngle3D(myPos,secondaryTrack["pos"],secondaryTrack["vel"])
        if(isInCriticalRange(myPos,secondaryTrack["pos"],radAspectAngle3D) and isHot(calcRadAspectAngle3D(myPos, secondaryTrack["pos"], secondaryTrack["vel"]),radCriteria=90*np.pi/180)):
            newTA = tAAtCriticalRange(myPos, secondaryTrack["pos"])
            if(newTA < countdown):
                countdown = newTA
                updated = True
                updateSource = "secondary target"
                distance = calcDistance3d(myPos,secondaryTrack["pos"])
    
    return countdown,updated,updateSource,distance

def _updateDescendCountdown(descendStartCountdown,descendEndCountdown,myPos,track):
    M_DESCEND_START_RANGE = 65000
    
    if(not track is None):
        distance = calcDistance3d(myPos,track["pos"])
        
        if(distance < M_DESCEND_START_RANGE):
            newTA = tAAtCriticalRange(myPos, track["pos"])
            if(newTA > descendEndCountdown):
                descendEndCountdown = newTA
                
            if(newTA < descendStartCountdown):
                descendStartCountdown = newTA
    
    return descendStartCountdown,descendEndCountdown

def updateDescenCountdownForAllTracks(descendStartCountdown,descendEndCountdown,myPos,primaryTrack,secondaryTrack):
    if(descendEndCountdown > 0):
        descendEndCountdown -= 0.1
        descendStartCountdown -= 0.1
    else:
        descendStartCountdown = np.inf
    
    descendStartCountdown,descendEndCountdown = _updateDescendCountdown(descendStartCountdown,descendEndCountdown,myPos,primaryTrack)
    descendStartCountdown,descendEndCountdown = _updateDescendCountdown(descendStartCountdown,descendEndCountdown,myPos,secondaryTrack)
    
    return descendStartCountdown,descendEndCountdown

def getClosestTargetTrack(observables):
    tracks = observables["sensor"]["track"]
    myPos = observables["motion"]["pos"]
    
    minimumDistance = np.inf
    closestTargetTrack = None
    for track in tracks:
        distance = calcDistance3d(myPos, track["pos"])
        if(distance < minimumDistance):
            minimumDistance = distance
            closestTargetTrack = track

    return closestTargetTrack

def timeToStr(time):
    time = float(time)
    minute = int(time)//60
    sec = time - minute*60
    
    return "{:}:{:02.1f}".format(minute,sec)
    
    
    

 