import math
import CollisionDetactor
import matplotlib.pyplot as plt
import numpy as np


R = 0.264  # m, from fetch diameter

unit = math.pi / 36


x = list()
y = list()
for i in range(35):
    x.append((-math.pi / 2 + (i + 1) * unit) / math.pi * 180)
    y.append(CollisionDetactor.calculate_collision_distance(-math.pi / 2 + (i + 1) * unit))
x = np.array(x)
y = np.array(y)
plt.plot(x, y)
plt.show()

ax = plt.axes(xlim=(-2, 2), ylim=(-2, 2))


o = np.array(CollisionDetactor.orig_debug)
f = np.array(CollisionDetactor.far_debug)

o_x = o[:,0]
o_y = o[:,1]
ax.plot(o_x, o_y, 'r')

f_x = f[:,0]
f_y = f[:,1]
ax.plot(f_x, f_y, 'b')


c = plt.Circle((0, 1), R, fc='w', ec='b')
ax.add_patch(c)

plt.show()
