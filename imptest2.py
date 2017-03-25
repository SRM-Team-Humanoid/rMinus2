from rMinus2 import *


class Bioloid(object):

    def __init__(self,dxl):
        self.dxl = dxl
        self.state = dxl.getPos()
        self.tree = XmlTree('data.xml',dxl=self.dxl)
        self.tree2 = XmlTree('soccer.xml',dxl=self.dxl)

        #Offsets
        darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
        abmath = {11: 15, 12: -15, 13: -10, 14: 10, 15: -5, 16: 5}
        hand = {5: 60, 6: -60}
        self.walk = Action(self.tree.superparsexml("22 F_S_L",offsets=[]))
        self.balance = MotionSet(self.tree.parsexml("152 Balance"), offsets=[])
        self.moon_walk = Action(self.tree2.superparsexml("11 B_L_S", offsets=[]))
        self.lback = MotionSet(self.tree2.parsexml("18 B_L_E"), offsets=[])
        self.rback = MotionSet(self.tree2.parsexml("17 B_R_E"), offsets=[])
        self.l_step = MotionSet(self.tree2.parsexml("10 ff_l_r"), speed=1.5, offsets=[])
        self.r_step = MotionSet(self.tree2.parsexml("9 ff_r_l"), speed=1.5, offsets=[])
        self.l_attack = MotionSet(self.tree.parsexml("21 L attack"),speed=1.2,offsets=[])
        self.kick = MotionSet(self.tree.parsexml("18 L kick"),speed=2,offsets=[])
        self.f_getup = MotionSet(self.tree.parsexml("27 F getup"),speed=2.7,offsets=[])
        self.b_getup = MotionSet(self.tree.parsexml("28 B getup  "),speed=1.5,offsets=[])
        self.r_inv = MotionSet(self.tree2.parsexml("19 RFT"),speed=1.2,offsets=[])
        self.l_inv = MotionSet(self.tree2.parsexml("20 LFT"),speed=1.2,offsets=[])
        self.r_turn = MotionSet(self.tree2.parsexml("27 RT"),speed=1.2,offsets=[])
        self.l_turn = MotionSet(self.tree2.parsexml("28 LT"),speed=1.2,offsets=[])
        w1 = MotionSet(self.tree.parsexml("32 F_S_L"),speed=2.1,offsets=[])
        w2 = MotionSet(self.tree.parsexml("33 "),speed=2.1,offsets=[])
        w3 = MotionSet(self.tree.parsexml("38 F_M_R"),speed=2.7,offsets=[])
        w4 = MotionSet(self.tree.parsexml("39 "),speed=2.1,offsets=[])
        w5 = MotionSet(self.tree.parsexml("36 F_M_L"),speed=2.7,offsets=[])
        w6 = MotionSet(self.tree.parsexml("37 "),speed=2.1,offsets=[])
        self.boom_walk = Action([self.l_step,self.r_step])
        self.walk_init = Action([w1,w2])
        self.walk_motion = Action([w3,w4,w5,w6])

    def execute(self,move,iter=1):
        move = self.__dict__[move]
        move.execute(iter=iter,state=self.state)
        #print move





bioloid = Bioloid(Dxl(lock=18))
bioloid.execute('balance')
raw_input("?")
bioloid.execute('walk',iter=3)