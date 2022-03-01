#-*-coding:utf-8 -*-


import os,json
from ASRCAISim1.policy.StandalonePolicy import StandalonePolicy

def getUserAgentClass():
    from . import MyRuleBasedAgent
    return MyRuleBasedAgent.MyRuleBasedAgent
    
def getUserAgentModelConfig():
    return json.load(open(os.path.join(os.path.dirname(__file__),"config.json"),"r"))

def isUserAgentSingleAsset():
    return False

class DummyPolicy(StandalonePolicy):
    def step(self,observation,reward,done,info,agentFullName,observation_space,action_space):
        return action_space.sample()

def getUserPolicy():
    return DummyPolicy()