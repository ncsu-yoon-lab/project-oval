import sys

sys.path.insert(0, '/home/sarvesh/Documents/GitHub/wolfwagen/AStar/environment')

import position

node1 = position.Position(0, 0)
node2 = position.Position(0, 1)
node3 = position.Position(0, 0)
node4 = position.Position(1, 0)

node1.set_right(node2)
node1.set_above(node4)


print(node1.__eq__(node3))

print(node1.get_right())
print(node1.get_above())

print(node2.get_left())
print(node4.get_below())
print(node1.__repr__())