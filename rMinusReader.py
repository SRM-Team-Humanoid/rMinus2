import pypot.dynamixel
import time
import numpy as np
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
        writ = {key: value for key, value in pose.items() if key in self.ids}
        #print writ
        self.dxl_io.set_goal_position(writ)

    def torque_off(self,ids):
        self.dxl_io.disable_torque(ids)

    def torque_on(self,ids):
        self.dxl_io.enable_torque(ids)

    def getPos(self):
        return Motion(1," ".join(map(str,self.dxl_io.get_present_position(self.ids))),0)
    def getPosString(self):
        return " ".join(map(str,self.dxl_io.get_present_position(self.ids)))



body = {"lhand":[2,4,6],"rhand":[1,3,5],"lleg":[10,12,14,15,16,17,18],"rleg":[9,11,13,15,16,17,18],"torso":range(1,7),"bottom":range(8,19),"body":range(1,21)}

root = ET.Element("root")
pageroot = ET.SubElement(root, "PageRoot")
dxl = Dxl(lock=20)

pagename = raw_input("Page name: ")
page = ET.SubElement(pageroot, "Page", name=pagename)
steps = ET.SubElement(page, "steps")
while True:
    tor = raw_input("Enter(d/e/x) - ")
    if tor == 'd':
        limb = raw_input()
        dxl.torque_off(body[limb])
    elif tor == 'e':
        Frame = raw_input("Frame: ")
        dxl.torque_on(body["body"])
        angles = dxl.getPosString()
        step = ET.SubElement(steps,"step", frame=Frame, pose=angles)
    elif tor == 'x':
        break



filename = raw_input("Enter File name:")
tree = ET.ElementTree(root)
tree.write(filename)
