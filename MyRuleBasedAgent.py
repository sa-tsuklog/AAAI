'''
Created on 2022/01/09

@author: sa
'''

from math import *
from gym import spaces
import numpy as np
import sys
from ASRCAISim1.libCore import *
from OriginalModelSample.libOriginalModelSample import *
from .MyUtil import *
from .Maneuver import *
from .IndividualInOutTactics import IndividualInOutTactics
from .ElementTactics import ElementTactics

class MyRuleBasedAgent(Agent):
    def __init__(self,modelConfig,instanceConfig):
        super().__init__(modelConfig,instanceConfig)
        if(self.isDummy):
            return
        
        self.tactics = ElementTactics()
        
    def validate(self):
        for port,parent in self.parents.items(): 
            parent.setFlightControllerMode("direct")
            
        self.tactics = ElementTactics()
        
    def observation_space(self):
        return spaces.Box(low=0.0,high=1.0,shape=(1,))
    
    def makeObs(self):
        return np.array([0.0,])
    
    def action_space(self):
        return spaces.MultiDiscrete(np.array([1,]))
    def deploy(self,actions):
        for port,parent in self.parents.items():
            if(parent.isAlive()):
                self.observables[parent.getFullName()]["decision"]={
                    "Roll":("Don't care"),
                    "Horizontal":("Az_BODY",90),
                    "Vertical":("El",0),
                    "Throttle":("Vel",300.0),
                    "Fire":(False,Track3D().to_json())
                }
        self.lastActions=actions[:]
    def control(self):
        
        names = ["",""]
        observations = [None,None]
        primaryAxis = (0,1,0)
        time = self.manager.getTime()
        
        for port,parent in self.parents.items():
            port = int(port)
            if(parent.isAlive()):
                if(parent.isinstance(CoordinatedFighter)):
                    observations[port] = parent.observables
                    
                    if(parent.getFullName() == "Blue/Blue1"):
                        names[port] = "Blue1"
                        names[not port] = "Blue2"
                        primaryAxis = (0,-1,0)
                    elif(parent.getFullName() == "Blue/Blue2"):
                        names[not port] = "Blue1"
                        names[port] = "Blue2"
                        primaryAxis = (0,-1,0)
                    elif(parent.getFullName() == "Red/Red1"):
                        names[port] = "Red1"
                        names[not port] = "Red2"
                        primaryAxis = (0,1,0)
                    elif(parent.getFullName() == "Red/Red2"):
                        names[not port] = "Red1"
                        names[port] = "Red2"
                        primaryAxis = (0,1,0)
                    
        
        commands = self.tactics.RuleBasedDoctrine(time, observations, names, primaryAxis)
        
        for port,parent in self.parents.items():
            port = int(port)
            #if(parent.isAlive()):
            if(parent.isinstance(CoordinatedFighter)):
                self.commands[parent.getFullName()]= commands[port] 
        
    def reportMissileRemaining(self):
        for port,parent in self.parents.items():
            if(parent.isAlive()):
                if(parent.isinstance(CoordinatedFighter)):
                    print("{}: {:} missiles remaining.".format(parent.getFullName(),parent.observables["weapon"]["remMsls"]))
                
                
                
                