import copy
from math import modf
from operator import itemgetter

import matplotlib.pyplot as plt
import numpy as np
from sympy import Point, Circle, Line
deviation = 0.01  # 误差
# 单架无人机到达目标点所需时间
# 　这里的UAV和target是无人机和目标对象

# v = UAV.v  # 飞机速度
phi0 = phi0 = np.pi / 4   # 转化为弧度，[0,2pi]
R0 = 20  # 最小转弯半径
UAV_p = Point(np.array([0, 0]))
target_p = Point(np.array([4, 0]))

# 以上为所有已知信息

# 1. 求两个圆心，判断出采用哪一个圆
# 2. 求切线
# 3. 确定用那一段弧长

# 1.求两个圆心，判断出采用哪一个圆
c1 = Point(UAV_p.x + R0 * np.sin(phi0), UAV_p.y - R0 * np.cos(phi0))
c2 = Point(UAV_p.x - R0 * np.sin(phi0), UAV_p.y + R0 * np.cos(phi0))
len1 = c1.distance(target_p)
len2 = c2.distance(target_p)
center = c1

if len2 > len1:
    center = c2

# 2. 求切线
circle = Circle(center, R0)
tangent_lines = circle.tangent_lines(target_p)

tangent_line1 = tangent_lines[0]  # 注意这里的切线方向是从target-> 切点
tangent_line1 = Line(tangent_line1.p2, tangent_line1.p1)  # 改为从切点->target
tangent_point1 = tangent_line1.p1  # 切点1
y = float((target_p.y - tangent_point1.y).evalf())
x = float((target_p.x - tangent_point1.x).evalf())
tangent_angle1 = np.arctan2(y, x)  # arctan2(y,x) 向量(x,y)的角度[-pi,pi]

tangent_line2 = tangent_lines[1]
tangent_line2 = Line(tangent_line2.p2, tangent_line2.p1)  # 改为从切点->target
tangent_point2 = tangent_line2.p1  # 切点２
y = float((target_p.y - tangent_point2.y).evalf())
x = float((target_p.x - tangent_point2.x).evalf())
tangent_angle2 = np.arctan2(y, x)  # arctan2(y,x) 向量(x,y)的角度[-pi,pi]

# 3. 确定用哪一段弧长
# a. 确定用顺时针还是逆时针
vec1 = [UAV_p.x - center.x, UAV_p.y - center.y]
vec2 = [np.cos(phi0), np.sin(phi0)]
direction = np.sign(vec1[0] * vec2[1] - vec1[1] * vec2[0])  # 1 表示逆时针 -1 表示顺时针
# b. 判断是哪一个切点，哪一段弧
sin1 = float(tangent_point1.distance(UAV_p).evalf()) / (2 * R0)
angle1 = 2 * np.arcsin(sin1)  # 无人机位置与切点之间的弧度[0,pi] 小弧
sin2 = float(tangent_point2.distance(UAV_p).evalf()) / (2 * R0)
angle2 = 2 * np.arcsin(sin2)

tangent_point = []
hudu = 0

# 判断式的意思  角度要在误差范围内相隔2kpi，使用modf(abs 把值控制在0-1之内，误差范围内靠近1，靠近0 都ok  用于0.5之间的距离来判断
if abs(modf(abs(direction * angle1 + phi0 - tangent_angle1) / (2 * np.pi))[0] - 0.5) > 0.5 - deviation:
    tangent_point = tangent_point1
    hudu = angle1
# modf 返回浮点数的小数部分和整数部分modf(1.23) return [0.23,1]
elif abs(modf(abs(direction * (2 * np.pi - angle1) + phi0 - tangent_angle1) / (2 * np.pi))[
             0] - 0.5) > 0.5 - deviation:
    tangent_point = tangent_point1
    hudu = 2 * np.pi - angle1
elif abs(modf(abs(direction * angle2 + phi0 - tangent_angle2) / (2 * np.pi))[0] - 0.5) > 0.5 - deviation:
    tangent_point = tangent_point2
    hudu = angle2
elif abs(modf(abs(direction * (2 * np.pi - angle2) + phi0 - tangent_angle2) / (2 * np.pi))[
             0] - 0.5) > 0.5 - deviation:
    tangent_point = tangent_point2
    hudu = 2 * np.pi - angle2
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111)

start = np.arctan2(float((UAV_p.y - center.y).evalf()), float((UAV_p.x - center.x).evalf())) * 180 / np.pi
end = start + hudu * 180 / np.pi
if direction == -1:
    end = start - hudu * 180 / np.pi
    start, end = end, start
# add a wedge
c = [float(center.x.evalf()), float(center.y.evalf())]
wedge = mpatches.Wedge(c, R0, start, end)
ax.add_patch(wedge)

ax.scatter(float(tangent_point.x.evalf()), float(tangent_point.y.evalf()), s=60, marker='^', color='blue', alpha=0.8)
ax.scatter(float(UAV_p.x.evalf()), float(UAV_p.y.evalf()), s=60, marker='^', color='blue', alpha=0.8)
ax.scatter(float(target_p.x.evalf()), float(target_p.y.evalf()), s=60, marker='^', color='blue', alpha=0.8)
ax.plot((float(tangent_point.x.evalf()), float(target_p.x.evalf())),
        (float(tangent_point.y.evalf()), float(target_p.y.evalf())))
plt.show()
#
def Arrival_time(UAV, target, R0):
    # 单架无人机到达目标点所需时间
    # 　这里的UAV和target是无人机和目标对象

    v = UAV.v # 飞机速度
    phi0 = UAV.phi  # 转化为弧度，[0,2pi]

    UAV_p = Point(UAV.site)
    target_p = Point(target[0:2])

    # 以上为所有已知信息

    # 1. 求两个圆心，判断出采用哪一个圆
    # 2. 求切线
    # 3. 确定用那一段弧长

    # 1.求两个圆心，判断出采用哪一个圆
    c1 = Point(UAV_p.x + R0 * np.sin(phi0), UAV_p.y - R0 * np.cos(phi0))
    c2 = Point(UAV_p.x - R0 * np.sin(phi0), UAV_p.y + R0 * np.cos(phi0))
    len1 = c1.distance(target_p)
    len2 = c2.distance(target_p)
    center = c1

    if len2 > len1:
        center = c2

    # 2. 求切线
    circle = Circle(center, R0)
    tangent_lines = circle.tangent_lines(target_p)

    tangent_line1 = tangent_lines[0]  # 注意这里的切线方向是从target-> 切点
    tangent_line1 = Line(tangent_line1.p2, tangent_line1.p1)  # 改为从切点->target
    tangent_point1 = tangent_line1.p1  # 切点1
    y = float((target_p.y - tangent_point1.y).evalf())
    x = float((target_p.x - tangent_point1.x).evalf())
    tangent_angle1 = np.arctan2(y, x)  # arctan2(y,x) 向量(x,y)的角度[-pi,pi]

    tangent_line2 = tangent_lines[1]
    tangent_line2 = Line(tangent_line2.p2, tangent_line2.p1)  # 改为从切点->target
    tangent_point2 = tangent_line2.p1  # 切点２
    y = float((target_p.y - tangent_point2.y).evalf())
    x = float((target_p.x - tangent_point2.x).evalf())
    tangent_angle2 = np.arctan2(y, x)  # arctan2(y,x) 向量(x,y)的角度[-pi,pi]

    # 3. 确定用哪一段弧长
    # a. 确定用顺时针还是逆时针
    vec1 = [UAV_p.x - center.x, UAV_p.y - center.y]
    vec2 = [np.cos(phi0), np.sin(phi0)]
    direction = np.sign(vec1[0] * vec2[1] - vec1[1] * vec2[0])  # 1 表示逆时针 -1 表示顺时针
    # b. 判断是哪一个切点，哪一段弧
    sin1 = float(tangent_point1.distance(UAV_p).evalf()) / (2 * R0)
    angle1 = 2 * np.arcsin(sin1)  # 无人机位置与切点之间的弧度[0,pi] 小弧
    sin2 = float(tangent_point2.distance(UAV_p).evalf()) / (2 * R0)
    angle2 = 2 * np.arcsin(sin2)

    tangent_point = []
    hudu = 0

    # 判断式的意思  角度要在误差范围内相隔2kpi，使用modf(abs 把值控制在0-1之内，误差范围内靠近1，靠近0 都ok  用于0.5之间的距离来判断
    if abs(modf(abs(direction * angle1 + phi0 - tangent_angle1) / (2 * np.pi))[0] - 0.5) > 0.5 - deviation:
        tangent_point = tangent_point1
        hudu = angle1
    # modf 返回浮点数的小数部分和整数部分modf(1.23) return [0.23,1]
    elif abs(modf(abs(direction * (2 * np.pi - angle1) + phi0 - tangent_angle1) / (2 * np.pi))[
                 0] - 0.5) > 0.5 - deviation:
        tangent_point = tangent_point1
        hudu = 2 * np.pi - angle1
    elif abs(modf(abs(direction * angle2 + phi0 - tangent_angle2) / (2 * np.pi))[0] - 0.5) > 0.5 - deviation:
        tangent_point = tangent_point2
        hudu = angle2
    elif abs(modf(abs(direction * (2 * np.pi - angle2) + phi0 - tangent_angle2) / (2 * np.pi))[
                 0] - 0.5) > 0.5 - deviation:
        tangent_point = tangent_point2
        hudu = 2 * np.pi - angle2
    path_length = R0 * hudu + float(tangent_point.distance(target_p).evalf())
    return path_length / v

