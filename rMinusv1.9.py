import random
import pypot.dynamixel
import time
import numpy as np
from pprint import pprint
import xml.etree.cElementTree as ET


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

    def write(self,state, speed,exclude=[],offset={}):
        begpos = state.pose
        endpos = self.pose
        frames = []
        ids = []
        for k in endpos.keys():
            print begpos
            if begpos[k]!=endpos[k] and k not in exclude:
                frames.append(np.linspace(begpos[k],endpos[k],self.delay))
                ids.append(k)

        frames = zip(*frames)
        for f in frames:
            writ = dict(zip(ids, f))
            dxl.setPos(writ)
            time.sleep(0.008 / speed)
            #print writ



class MotionSet(object):
    def __init__(self,motions,speed=1,exclude =[]):
        self.motions = motions
        self.speed = speed
        self.exclude = exclude

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
                motion.write(state,speed,self.exclude)
                state = motion
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




dxl = Dxl(lock=20)
tree = ET.ElementTree(file='data2.xml')
state = dxl.getPos()
#state  = parsexml("152 Balance",tree)[0]
balance = MotionSet(parsexml("152 Balance",tree))
bravo = MotionSet(parsexml("21 L attack",tree),exclude=[1,2,3,4,5,6])
kick = MotionSet(parsexml("18 L kick",tree),2,exclude=[1,2,3,4,5,6])



x = raw_input("Proceed (y/n)?")
if x == 'y':
    balance.execute()
    time.sleep(1)
    kick.execute()







