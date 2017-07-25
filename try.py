#

from sympy import Point, Circle, Line, var
import sympy
import numpy as np
import math

import matplotlib.pyplot as plt

var('t')
UAV=Point(5,5)
UAV_p=np.array([5,5])
target=Point(10,8)
R0=3
phi0=np.pi/4
phi1=np.arctan2(8,10)
center=[]

np.arctan2()
if phi0>phi1:
    center=Point(UAV.x-R0*np.sin(phi0),UAV.y+R0*np.cos(phi0))#逆时针起始圆圆心
else:
    center=Point(Point.x+R0*np.sin(phi0),Point.y-R0*np.cos(phi0))#顺时针起始圆圆心
c1 = Circle(center, R0)

t1 = c1.tangent_lines(target)
for tangent_line in t1:

    p1=np.array([tangent_line.p1.x.evalf(),tangent_line.p1.y.evalf()])  # 目标点
    p2 = np.array([tangent_line.p2.x.evalf(), tangent_line.p2.y.evalf()]) # 切点


    tan=(p1-p2)[1]/(p1-p2)[0]

    angle=phi0-math.atan(tan)   #

    lens=math.sqrt(sum((p2-UAV_p)**2))
    theta1 = 2 * math.asin(lens/2/R0)

    juge=theta1==angle


    print('')


dd=Point( 3,4)-Point( 4,4)

# p1 = l1.arbitrary_point(t).subs({t: -c1.radius / (c2.radius - c1.radius)})
# p2 = l1.arbitrary_point(t).subs({t:  c1.radius / (c1.radius + c2.radius)})
p1=Point(8,9)
t1 = c1.tangent_lines(p1)




# t2 = c1.tangent_lines(p2)
# ta = t1 + t2

# fig = plt.gcf()
# ax = fig.gca()
# ax.set_xlim((-10, 10))
# ax.set_ylim((-10, 10))
# ax.set_aspect(1)
#
# cp1 = plt.Circle((c1.center.x, c1.center.y), c1.radius, fill = False)
# cp2 = plt.Circle((c2.center.x, c2.center.y), c2.radius, fill = False)
# tp = [0 for i in range(4)]
# for i in range(4):
#     start = ta[i].arbitrary_point(t).subs({t:-10})
#     end = ta[i].arbitrary_point(t).subs({t:10})
#     tp[i] = plt.Line2D([start.x, end.x], [start.y, end.y], lw = 2)
#
# ax.add_artist(cp1)
# ax.add_artist(cp2)
# for i in range(4):
#     ax.add_artist(tp[i])