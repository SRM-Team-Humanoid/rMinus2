import random
import pypot.dynamixel
import time
import numpy as np
from pprint import pprint
import xml.etree.cElementTree as ET
from collections import Counter
from copy import deepcopy


class Dxl(object):
    def __init__(self,port_id=0, scan_limit=25, lock=-1):
        # Initializes Dynamixel Object
        # Port ID is zero by default
        ports = pypot.dynamixel.get_available_ports()
        if not ports:
            raise IOError('no port found!')

        print('ports found', ports)
        print('connecting on the first available port:', ports[port_id])
        dxl_io = pypot.dynamixel.DxlIO(ports[port_id])
        ids = dxl_io.scan(range(25))
        print(ids)

        if lock > 0:
            if len(ids) < lock:
                raise RuntimeError("Couldn't detect all motors.")

        self.dxl_io = dxl_io
        self.ids = ids
    def setPos(self,pose):
        '''
        for k in pose.keys():
            if k not in self.ids:
                del pose[k]
        '''
        writ = {key: value for key, value in pose.items() if key in self.ids}
        #print writ
        self.dxl_io.set_goal_position(writ)

    def getPos(self):
        return Motion(1," ".join(map(str,self.dxl_io.get_present_position(self.ids))),0)



class Motion(object):
    def __init__(self,frame,pose,prev_frame):
        self.frame = int(frame)
        self.pose = {}
        self.delay = self.frame-int(prev_frame)
        for i,p in enumerate(pose.split()):
            self.pose[i+1] =float(p)

    def __str__(self):
        return "Frame:"+str(self.frame) + "      Delay:"+str(self.delay) + "     Pose:"+" ".join(map(str,self.pose.values()))

    def updatePose(self,offset,add=True):
        if add:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] += offset[k]
        else:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] -= offset[k]


    def write(self,state, speed,exclude=[],offset={}):
        begpos = state.pose
        endpos = self.pose
        frames = []
        ids = []
        for k in endpos.keys():
            if begpos[k]!=endpos[k] and k not in exclude:
                frames.append(np.linspace(begpos[k],endpos[k],self.delay))
                ids.append(k)

        frames = zip(*frames)
        for f in frames:
            writ = dict(zip(ids, f))
            #dxl.setPos(writ)
            time.sleep(0.008 / speed)
            #print writ



class MotionSet(object):
    def __init__(self,motions,speed=1,exclude =[],offsets=[]):
        self.motions = motions
        self.speed = speed
        self.exclude = exclude
        '''
        sum = Counter({})
        for offset in offsets:
            sum += Counter(offset)
            print "-----------------"
            print dict(Counter(offset))
        self.offset = dict(sum)
        '''
        sum = {}
        for offset in offsets:
            for k in offset.keys():
                if k in sum:
                    if offset[k]
                    sum[k]+=offset[k]
                else:
                    sum[k]=offset[k]

        self.offset = sum
        print self.offset

    def setExclude(self,list):
        self.exclude = list

    def setSpeed(self,speed):
        self.speed = speed

    def execute(self,speed=-1,iter=1):
        if speed<0:
            speed = self.speed

        global state
        while iter>0:
            for motion in self.motions:
                print motion
                print self.offset
                motion.updatePose(self.offset)
                print motion
                motion.write(state,speed,self.exclude)
                state = deepcopy(motion)
                motion.updatePose(self.offset,add=False)
            iter-=1

def parsexml(text, tree):
    find = "PageRoot/Page[@name='" + text + "']/steps/step"
    motions = []
    prev_frame = 0
    steps = [x for x in tree.findall(find)]
    for step in steps:
        motion = Motion(step.attrib['frame'],step.attrib['pose'],prev_frame)
        prev_frame = step.attrib['frame']
        motions.append(motion)

    return motions

'''
#Bioloid2Darwin
motion[1] += 90
motion[2] += -90
motion[3] += 67.5
motion[4] += -67.5
motion[7] += 45
motion[8] += -45
motion[9] = (-1) * motion[9]
motion[10] = (-1) * motion[10]
motion[13] = (-1) * motion[13]
motion[14] = (-1) * motion[14]
motion[17] = (-1) * motion[17]
motion[18] = (-1) * motion[18]

#Aditya_Bhaiya_Math
motion[11] += 15
motion[12] += -15
motion[13] += -10
motion[14] += 10
motion[15] += -5
motion[16] += 5
'''

dxl = Dxl(lock=18)
tree = ET.ElementTree(file='data2.xml')
state = dxl.getPos()
darwin = {1:90,2:-90,3:67.5,4:-67.5,7:45,8:-45,9:'i',10:'i',13:'i',14:'i',17:'i',18:'i'}
abmath = {11:15,12:-15,13:-10,14:10,15:-5,16:5}
offsets = [darwin]
#state  = parsexml("152 Balance",tree)[0]
balance = MotionSet(parsexml("152 Balance",tree),offsets=offsets)
#bravo = MotionSet(parsexml("21 L attack",tree),exclude=[1,2,3,4,5,6],offsets=offsets)
#kick = MotionSet(parsexml("18 L kick",tree),2,exclude=[1,2,3,4,5,6])

print state

x = raw_input("Proceed (y/n)?")
if x == 'y':
    balance.execute()
    #time.sleep(1)
    #kick.execute()






