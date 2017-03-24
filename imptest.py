from rMinus2 import *


dxl = Dxl(lock=20)
state = dxl.getPos()
initialize(dxl,state)


darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
tree = XmlTree('data.xml')
walk = Action(tree.superparsexml("22 F_S_L",offsets=[darwin]))


balance = MotionSet(tree.parsexml("152 Balance"), offsets=[darwin])
balance.execute()
