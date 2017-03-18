import random
import pypot.dynamixel
import time
import numpy as np
import xml.etree.cElementTree as ET

ports = pypot.dynamixel.get_available_ports()
if not ports:
    raise IOError('no port found!')

print('ports found', ports)
print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])
ids = dxl_io.scan(range(25))
print(ids)

while True:
	k = raw_input("")
	if k=="1":
		dxl_io.disable_torque(ids)
	else:
		dxl_io.enable_torque(ids)
		#dxl_io.set_goal_position({ids[0]:0})
