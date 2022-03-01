'''
Created on 2022/01/10

@author: sa
'''
import numpy as np
from ASRCAISim1.libCore import *
from .MyUtil import *
from scipy.spatial.transform import Rotation as R

def observationToAttitude(observables):
    attitude = R.from_quat([float(observables["motion"]["q"][1]),float(observables["motion"]["q"][2]),float(observables["motion"]["q"][3]),float(observables["motion"]["q"][0])])
    return attitude

def observationToVelocity(observables):
    vel = np.array([float(observables["motion"]["vel"][0]),float(observables["motion"]["vel"][1]),float(observables["motion"]["vel"][2])])
    return vel
    
def courseHold(observables,vecTargetDir,rollGain=(1 / (300 / 180 * np.pi)),pitchGain=(1/(90 /180 * np.pi)),rollLimit=1.0,pitchLimit=1.0):
    
    vel = observationToVelocity(observables)
    attitude = observationToAttitude(observables)
    
    roll,pitch,heading = ObservationToRollPitchHeading(observables)
    # print("roll,pitch,heading",roll*180/np.pi,pitch*180/np.pi,heading*180/np.pi)
    # print("vel",vel)
    # print("targetDir",vecTargetDir)
    
    vel = vel / np.linalg.norm(vel,ord=2)
    vecTargetDir = vecTargetDir / np.linalg.norm(vecTargetDir,ord=2)
    
    turnAxis = np.cross(vel,vecTargetDir)
    turnAngle = np.arccos(np.dot(vel,vecTargetDir))
    
    bodyFrameTurnAxis = attitude.inv().apply(turnAxis)
    
    bodyFrameVel = attitude.inv().apply(vel)
    bodyFrameTargetDir = attitude.inv().apply(vecTargetDir)
    
    bodyFrameTurnAxis = np.cross(bodyFrameVel,bodyFrameTargetDir)
    
    radRollDiff =  np.arctan2(bodyFrameTurnAxis[2],bodyFrameTurnAxis[1])
    
    radVelPitch = -np.arctan2(bodyFrameVel[2],bodyFrameVel[0])
    radTargetDirPitch = -np.arctan2(bodyFrameTargetDir[2],bodyFrameTargetDir[0])
    
    rollCommand = rollGain * radRollDiff
    pitchCommand = pitchGain * (radTargetDirPitch - radVelPitch) 
    
    if(rollCommand > 1.0):
        rollCommand = 1.0
    elif(rollCommand < -1.0):
        rollCommand = -1.0
    
    if(pitchCommand > 1.0):
        pitchCommand = 1.0
    elif(pitchCommand < -1.0):
        pitchCommand = -1.0
    
    if(rollCommand > rollLimit):
        rollCommand = rollLimit
    elif(rollCommand < -rollLimit):
        rollCommand = -rollLimit
    
    if(pitchCommand > pitchLimit):
        pitchCommand = pitchLimit
    elif(pitchCommand < -pitchLimit):
        pitchCommand = -pitchLimit
    
    # print("turn axis",turnAxis)
    # print("roll diff",radRollDiff*180/np.pi)
    # print("turnAxis in body frame",bodyFrameTurnAxis)
    # print("body frame vel",bodyFrameVel)
    # print("vel pitch",radVelPitch*180/np.pi)
    # print("target Pitch",radTargetDirPitch*180/np.pi)
    # print("pitchCommand",pitchCommand * 180/np.pi)
    
    return rollCommand,pitchCommand

def altDirHold(observables,vecTargetDir,mTargetAlt,radClimbAngleLimit=35/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000,rollGain=(1 / (360 / 180 * np.pi)),pitchGain=(1/(90 /180 * np.pi)),rollLimit=1.0,pitchLimit=1.0):
    heightDiff = float(observables["motion"]["pos"][2]) - mTargetAlt
    radDiveAngle = altGain * heightDiff
    
    # print("height:",observables["motion"]["pos"][2],",",mTargetAlt,",",heightDiff)
    # print("raw dive angle:", radDiveAngle *180/np.pi)
    
    if(radDiveAngle < radDiveAngleLimit):
        radDiveAngle = radDiveAngleLimit
    elif(radDiveAngle > radClimbAngleLimit):
        radDiveAngle = radClimbAngleLimit
    
    vec3DDir = np.array([vecTargetDir[0],vecTargetDir[1],0]).astype(np.float64)
    norm = np.linalg.norm(vec3DDir,ord=2)
    
    vec3DDir[2] = norm * np.tan(-radDiveAngle)
    vec3DDir = vec3DDir / np.linalg.norm(vec3DDir,ord=2)
    
    # print("3d dir:",vec3DDir)
    
    return courseHold(observables, vec3DDir,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)

def courseAltHold(observables,vecTargetDir2D,mReferencePoint2D,mTargetAlt,radPerMCourseGain = 60/180*np.pi/3000,radDirectionDeviationLimit=60/180*np.pi,radClimbAngleLimit=30/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000,rollGain=(1 / (360 / 180 * np.pi)),pitchGain=(1/(90 /180 * np.pi)),rollLimit=1.0,pitchLimit=1.0):
    currentPos = np.array([float(observables["motion"]["pos"][0]),float(observables["motion"]["pos"][2]),float(observables["motion"]["pos"][2])])
    npMReferencePoint2D = np.array([float(mReferencePoint2D[0]),float(mReferencePoint2D[1]),float(mReferencePoint2D[2])])
    ###################################
    # 目標方向に対して何度ずらすかを計算
    ###################################    
    mDeviationToCourse = np.abs((vecTargetDir2D[1] * (currentPos[0] - npMReferencePoint2D[0]) - vecTargetDir2D[0] * (currentPos[1] - npMReferencePoint2D[1])) / np.sqrt(vecTargetDir2D[0]*vecTargetDir2D[0] + vecTargetDir2D[1]*vecTargetDir2D[1]))
    radDirectionToCourse = radPerMCourseGain * mDeviationToCourse
    radDirectionToCourse = np.clip(radDirectionToCourse , -radDirectionDeviationLimit,radDirectionDeviationLimit)
    
    
    if(mDeviationToCourse == 0):
        radDirectionToCourse = 0    #現在のコースが完全に一致している場合の例外処理(nan避け)
    else:
        ###################################
        # ずらす向きを計算(1,-1)
        ###################################
        vecCurrentPointToRefPoint = np.array([npMReferencePoint2D[0]-currentPos[0],npMReferencePoint2D[1]-currentPos[1],npMReferencePoint2D[2]-currentPos[2]])
        
        directionSign = np.cross(vecTargetDir2D,vecCurrentPointToRefPoint)[2]
        directionSign = directionSign / np.abs(directionSign)
        
        ###################################
        # 目標方向に対して何度ずらすかを計算
        ###################################
        radDirectionToCourse = radDirectionToCourse * directionSign
    
    ###################################
    # 目標方向と方向ずらしを合成
    ###################################
    vecTargetDir2D = np.array([vecTargetDir2D[0],vecTargetDir2D[1],0])
    rotation = R.from_quat([0,0,np.sin(radDirectionToCourse/2),np.cos(radDirectionToCourse/2)])
    
    vecInstantaneousTargetDir2D = rotation.apply(vecTargetDir2D)
    
    return altDirHold(observables,vecInstantaneousTargetDir2D,mTargetAlt,radClimbAngleLimit, radDiveAngleLimit, altGain,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)
    
def snaking(snakingTime,observables,vecTargetDir,mTargetAlt,polarity=1,sCycle=30.0,radSnakingAngle=60.0/180*np.pi,radClimbAngleLimit=35/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000,rollGain=(1 / (360 / 180 * np.pi)),pitchGain=(1/(90 /180 * np.pi)),rollLimit=1.0,pitchLimit=1.0):
    radAngleOff = polarity * radSnakingAngle * np.sin(2*np.pi*snakingTime/sCycle)
    
    r = R.from_quat([0, 0,np.sin(radAngleOff/2),np.cos(radAngleOff/2)])
    vecTargetDir2D = np.array([vecTargetDir[0],vecTargetDir[1],0])
    vecTargetDir2D = r.apply(vecTargetDir2D)
    
    return altDirHold(observables,vecTargetDir2D,mTargetAlt,radClimbAngleLimit, radDiveAngleLimit, altGain,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)

def calcSnakingPolarity(vecTargetDir,primaryAxis):
    npPrimaryAxis2D = np.array([primaryAxis[0],primaryAxis[1],primaryAxis[2]])
    npVecTargetDir2D = np.array([float(vecTargetDir[0]),float(vecTargetDir[1]),float(vecTargetDir[2])])
    
    headingToPrimaryAxis = np.cross(npPrimaryAxis2D,npVecTargetDir2D)[2]
    
    if(headingToPrimaryAxis > 0):
        return 1
    else:
        return -1

def barrelRoll(barrelTime,observables,vecTargetDir,mTargetAlt,mAltAmplitude=-2000,polarity=1,sCycle=30.0,radSnakingAngle=60.0/180*np.pi,radClimbAngleLimit=35/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000,rollGain=(1 / (360 / 180 * np.pi)),pitchGain=(1/(90 /180 * np.pi)),rollLimit=1.0,pitchLimit=1.0):
    radAngleOff = polarity * radSnakingAngle * np.sin(2*np.pi*barrelTime/sCycle)
    mBarrelAlt =  mTargetAlt + mAltAmplitude * (1 - np.cos(2*np.pi*barrelTime/sCycle)/2)
    
    r = R.from_quat([0, 0,np.sin(radAngleOff/2),np.cos(radAngleOff/2)])
    vecTargetDir2D = np.array([vecTargetDir[0],vecTargetDir[1],0])
    vecTargetDir2D = r.apply(vecTargetDir2D)
    
    return altDirHold(observables,vecTargetDir2D,mBarrelAlt,radClimbAngleLimit, radDiveAngleLimit, altGain,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)

def outFromMws(time,observables,mTargetAltAtOut,vecTargetDirAtIn,mTargetAltAtIn,radClimbAngleLimit=35/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000,rollGain=(1 / (360 / 180 * np.pi)),pitchGain=(1/(90 /180 * np.pi)),rollLimit=1.0,pitchLimit=1.0):
    tracks = observables["sensor"]["mws"]["track"]
    
    mwsActive = isMwsActive(observables)
    
    if(mwsActive):
        mwsDir = meanMwsDir(observables)
        return altDirHold(observables,-mwsDir,mTargetAltAtOut,radClimbAngleLimit, radDiveAngleLimit, altGain,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)
    else:
        return altDirHold(observables,vecTargetDirAtIn,mTargetAltAtIn,radClimbAngleLimit, radDiveAngleLimit, altGain,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)

def outFromMwsWithSnaking(snakingTime,observables,mTargetAltAtOut,polarity=1,sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=(1,0,0),mTargetAltAtIn=-6000,radClimbAngleLimit=35/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000,rollGain=(1 / (360 / 180 * np.pi)),pitchGain=(1/(90 /180 * np.pi)),rollLimit=1.0,pitchLimit=1.0):
    tracks = observables["sensor"]["mws"]["track"]
    
    mwsActive = isMwsActive(observables)
    
    if(mwsActive):
        mwsDir = meanMwsDir(observables)
        return snaking(snakingTime,observables,-mwsDir,mTargetAltAtOut,polarity=polarity,sCycle=sCycle,radSnakingAngle=radSnakingAngle,radClimbAngleLimit=radClimbAngleLimit, radDiveAngleLimit=radDiveAngleLimit,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)
    else:
        return altDirHold(observables,vecTargetDirAtIn,mTargetAltAtIn,radClimbAngleLimit, radDiveAngleLimit, altGain,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)

def outFromMwsWithBarrel(barrelTime,observables,mTargetAltAtOut,mAltAmplitude=-3000,polarity=1,sCycle=30.0,radSnakingAngle=45.0/180*np.pi,vecTargetDirAtIn=(1,0,0),mTargetAltAtIn=-6000,radClimbAngleLimit=35/180*np.pi, radDiveAngleLimit=-30/180*np.pi, altGain=1/3000,rollGain=(1 / (360 / 180 * np.pi)),pitchGain=(1/(90 /180 * np.pi)),rollLimit=1.0,pitchLimit=1.0):
    tracks = observables["sensor"]["mws"]["track"]
    
    mwsActive = isMwsActive(observables)
    
    if(mwsActive):
        mwsDir = meanMwsDir(observables)
        return barrelRoll(barrelTime,observables,-mwsDir,mTargetAltAtOut,mAltAmplitude=mAltAmplitude,polarity=polarity,sCycle=sCycle,radSnakingAngle=radSnakingAngle,radClimbAngleLimit=radClimbAngleLimit, radDiveAngleLimit=radDiveAngleLimit,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)
    else:
        return altDirHold(observables,vecTargetDirAtIn,mTargetAltAtIn,radClimbAngleLimit, radDiveAngleLimit, altGain,rollGain=rollGain,pitchGain=pitchGain,rollLimit=rollLimit,pitchLimit=pitchLimit)
