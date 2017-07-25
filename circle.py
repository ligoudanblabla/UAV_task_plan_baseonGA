
from sympy import Point, Circle, Line, var
import matplotlib.pyplot as plt

var('t')

c1 = Circle(Point(0, 0), 2)
c2 = Circle(Point(4, 4), 3)
l1 = Line(c1.center, c2.center)
p1 = l1.arbitrary_point(t).subs({t: -c1.radius / (c2.radius - c1.radius)})
p2 = l1.arbitrary_point(t).subs({t:  c1.radius / (c1.radius + c2.radius)})
t1 = c1.tangent_lines(p1)
t2 = c1.tangent_lines(p2)
ta = t1 + t2

fig = plt.gcf()
ax = fig.gca()
ax.set_xlim((-10, 10))
ax.set_ylim((-10, 10))
ax.set_aspect(1)

cp1 = plt.Circle((c1.center.x, c1.center.y), c1.radius, fill = False)
cp2 = plt.Circle((c2.center.x, c2.center.y), c2.radius, fill = False)
tp = [0 for i in range(4)]
for i in range(4):
    start = ta[i].arbitrary_point(t).subs({t:-10})
    end = ta[i].arbitrary_point(t).subs({t:10})
    tp[i] = plt.Line2D([start.x, end.x], [start.y, end.y], lw = 2)

ax.add_artist(cp1)
ax.add_artist(cp2)
for i in range(4):
    ax.add_artist(tp[i])
plt.show()