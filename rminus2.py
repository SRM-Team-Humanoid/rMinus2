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
        writ = {key: value for key, value in pose.items() if key in self.ids}
        #print writ
        self.dxl_io.set_goal_position(writ)

    def getPos(self):
        return Motion(1," ".join(map(str,self.dxl_io.get_present_position(self.ids))),0)


class XmlTree(object):

    def __init__(self,str):
        try:
            with open(str) as f:
                pass
            self.tree = ET.ElementTree(file=str)
        except:
            raise RuntimeError("File "+str+" not found.")

    def parsexml(self,text):
        find = "PageRoot/Page[@name='" + text + "']/steps/step"
        motions = []
        prev_frame = 0
        steps = [x for x in self.tree.findall(find)]
        for step in steps:
            motion = Motion(step.attrib['frame'], step.attrib['pose'], prev_frame)
            prev_frame = step.attrib['frame']
            motions.append(motion)

        return motions


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
            dxl.setPos(writ)
            time.sleep(0.008 / speed)
            print writ



class MotionSet(object):
    def __init__(self,motions,speed=1.0,exclude =[],offsets=[]):
        self.motions = motions
        self.speed = speed
        self.exclude = exclude
        self.offsets = offsets
        self.loaded = False

    def setExclude(self,list):
        self.exclude = list

    def setSpeed(self,speed):
        self.speed = speed

    def execute(self,speed=-1,iter=1):
        global state
        if speed<0:
            speed = self.speed
        if not self.loaded:
            for offset in self.offsets:
                for motion in self.motions:
                    motion.updatePose(offset)
            self.loaded = True

        while iter>0:
            for motion in self.motions:
                motion.write(state,speed,self.exclude)
                state = deepcopy(motion)
            iter-=1


class Action():
    def __init__(self,motionsets):
        self.motionsets=motionsets

    def execute(self,iter=1,speed=1):
        while iter>0:
            for motionset in self.motionsets:
                orig = motionset.speed
                motionset.speed = motionset.speed*speed
                motionset.execute()
                motionset.speed = orig
            iter -= 1


class Humanoid(object):
    def __init__(self,dxl,moves):
        self.dxl = dxl
        self.state = dxl.getPos()
        self.moves = moves

    def add_moves(self,moves):
        self.moves.extend(moves)
    def move(self,num):
        moves[num].execute()


tree = XmlTree('data2.xml')
if __name__=='__main__':
    '''
    dxl = Dxl(lock=20)
    tree = ET.ElementTree(file='data2.xml')
    tree2 = ET.ElementTree(file='DRIBLE.xml')
    state = dxl.getPos()
    darwin = {1:90,2:-90,3:67.5,4:-67.5,7:45,8:-45,9:'i',10:'i',13:'i',14:'i',17:'i',18:'i'}
    abmath = {11:15,12:-15,13:-10,14:10,15:-5,16:5}
    hand = {5:45,6:-45}
    offsets = [darwin,abmath,hand]
    offsets2 = [darwin,hand]
    #state  = parsexml("152 Balance",tree)[0]
    balance = MotionSet(parsexml("152 Balance",tree),offsets=offsets)
    l_attack = MotionSet(parsexml("21 L attack",tree),speed=1.2,offsets=offsets)
    kick = MotionSet(parsexml("18 L kick",tree),speed=2,offsets=offsets)
    f_getup = MotionSet(parsexml("27 F getup",tree),speed=2.7,offsets=offsets)
    b_getup = MotionSet(parsexml("28 B getup  ",tree),speed=1.5,offsets=offsets)
    r_inv = MotionSet(parsexml("19 RFT",tree2),speed=1.2,offsets=offsets)
    l_inv = MotionSet(parsexml("20 LFT",tree2),speed=1.2,offsets=offsets)
    r_turn = MotionSet(parsexml("27 RT",tree2),speed=1.2,offsets=offsets)
    l_turn = MotionSet(parsexml("28 LT",tree2),speed=1.2,offsets=offsets)

    #walk goes in superclass

    w1 = MotionSet(parsexml("32 F_S_L",tree),speed=2.1,offsets=offsets)
    w2 = MotionSet(parsexml("33 ",tree),speed=2.1,offsets=offsets)
    w3 = MotionSet(parsexml("38 F_M_R",tree),speed=2.7,offsets=offsets)
    w4 = MotionSet(parsexml("39 ",tree),speed=2.1,offsets=offsets)
    w5 = MotionSet(parsexml("36 F_M_L",tree),speed=2.7,offsets=offsets)
    w6 = MotionSet(parsexml("37 ",tree),speed=2.1,offsets=offsets)

    l_step = MotionSet(parsexml("10 ff_l_r", tree2), speed=1.5, offsets=offsets2)
    r_step = MotionSet(parsexml("9 ff_r_l", tree2), speed=1.5, offsets=offsets2)
    balance2 = MotionSet(parsexml("152 Balance", tree), offsets=offsets2)
    boom_walk = Action([l_step,r_step])

    print state

    walk_init = Action([w1,w2])
    walk_motion = Action([w3,w4,w5,w6])
    balance2.execute()
    x = raw_input("Proceed (y/n)?")
    prev =''
    if x == 'y':
        boom_walk.execute(10,speed=1.4)
        #walk_init.execute(speed=2)
        #walk_motion.execute(10,speed=2)
        balance2.execute()

        while True:
            x = raw_input()
            if x=='l':
                l_turn.execute()
                prev = ''
            if x=='w' and prev =='w':
                walk_motion.execute()
                prev = 'w'
            elif x=='w':
                walk_init.execute()
                walk_motion.execute()
                prev = 'w'
            elif x=='r':
                r_turn.execute()
                prev = ''
            elif x=='a':
                l_inv.execute()
                prev = ''
            elif x=='d':
                r_inv.execute()
                prev = ''
            elif x=='k':
                kick.execute()
                prev = ''
            '''






