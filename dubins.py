#




from math import modf

import numpy as np
from sympy import Point, Circle, Line

# def Arrivals_time():


# def Arrival_time(UAV,target):
#     # 单架无人机到达目标点所需时间
#     #　这里的UAV和target是无人机和目标对象
#     UAV_p=UAV[0:2]
#     target_p=target[0:2]
#
#     R0 = UAV[4]  # 最小转弯半径
#     v=UAV[3] # 飞机速度
#     phi0 = UAV[2]*np.pi/180 # 转化为弧度，[0,2pi]
#
#     UAV = Point(UAV_p)  # 命名有问题懒得改了
#     target = Point(target_p) # 命名有问题懒得改了
#
#     # 以上为所有已知信息
#
#     # 1. 求两个圆心，判断出采用哪一个圆
#     # 2. 求切线
#     # 3. 确定用那一段弧长
#
#     # 1.求两个圆心，判断出采用哪一个圆
#     c1 = Point(UAV.x + R0 * np.sin(phi0), UAV.y - R0 * np.cos(phi0))
#     c2 = Point(UAV.x - R0 * np.sin(phi0), UAV.y + R0 * np.cos(phi0))
#     len1 = c1.distance(target)
#     len2 = c2.distance(target)
#     center = c1
#
#     if len2 > len1:
#         center = c2
#
#     # 2. 求切线
#     circle = Circle(center, R0)
#     tangent_lines = circle.tangent_lines(target)
#
#     tangent_line1 = tangent_lines[0]  # 注意这里的切线方向是从target-> 切点
#     tangent_line1 = Line(tangent_line1.p2, tangent_line1.p1)  # 改为从切点->target
#     tangent_point1 = tangent_line1.p1  # 切点1
#     y = float((target.y - tangent_point1.y).evalf())
#     x = float((target.x - tangent_point1.x).evalf())
#     tangent_angle1 = np.arctan2(y, x)  # arctan2(y,x) 向量(x,y)的角度[-pi,pi]
#
#     tangent_line2 = tangent_lines[1]
#     tangent_line2 = Line(tangent_line2.p2, tangent_line2.p1)  # 改为从切点->target
#     tangent_point2 = tangent_line2.p1  # 切点２
#     y = float((target.y - tangent_point2.y).evalf())
#     x = float((target.x - tangent_point2.x).evalf())
#     tangent_angle2 = np.arctan2(y, x)  # arctan2(y,x) 向量(x,y)的角度[-pi,pi]
#
#     # 3. 确定用哪一段弧长
#     # a. 确定用顺时针还是逆时针
#     vec1 = [UAV.x - center.x, UAV.y - center.y]
#     vec2 = [np.cos(phi0), np.sin(phi0)]
#     direction = np.sign(vec1[0] * vec2[1] - vec1[1] * vec2[0])  # 1 表示逆时针 -1 表示顺时针
#     # b. 判断是哪一个切点，哪一段弧
#     sin1 = float(tangent_point1.distance(UAV).evalf()) / (2 * R0)
#     angle1 = 2 * np.arcsin(sin1)  # 无人机位置与切点之间的弧度[0,pi] 小弧
#     sin2 = float(tangent_point2.distance(UAV).evalf()) / (2 * R0)
#     angle2 = 2 * np.arcsin(sin2)
#
#     tangent_point = []
#     hudu = 0
#
#     if abs(direction * angle1 + phi0 - tangent_angle1) % (2 * np.pi) < deviation:
#         tangent_point = tangent_point1
#         hudu = angle1
#
#     elif abs(direction * (2 * np.pi - angle1) + phi0 - tangent_angle1) % (2 * np.pi) < deviation:
#         tangent_point = tangent_point1
#         hudu = 2 * np.pi - angle1
#     elif abs(direction * angle2 + phi0 - tangent_angle2) % (2 * np.pi) < deviation:
#         tangent_point = tangent_point2
#         hudu = angle2
#     elif abs(direction * (2 * np.pi - angle2) + phi0 - tangent_angle2) % (2 * np.pi) < deviation:
#         tangent_point = tangent_point2
#         hudu = 2 * np.pi - angle2
#     path_length= R0*hudu+float(tangent_point.distance(target).evalf())
#     # return path_length/v

def Tangent_lines(circle_C,point_P):
    # 圆外一点到圆的切线，
    # 返回从point到切点的line
    R=float(circle_C.radius.evalf())
    circle=[float(circle_C.center.x.evalf()),float(circle_C.center.y.evalf())]
    point=[float(point_P.x.evalf()),float(point_P.y.evalf())]

    circle_point_angle=np.arctan2(point[1]-circle[1],point[0]-circle[0])
    cos=R/np.sqrt(np.sum((np.array(circle)-np.array(point))**2))
    hudu_half=np.arccos(cos)

    tangent_angle1=circle_point_angle+hudu_half
    tangent_point1=Point(circle[0]+R*np.cos(tangent_angle1),circle[1]+R*np.sin(tangent_angle1))

    tangent_angle2 = circle_point_angle - hudu_half
    tangent_point2 = Point(circle[0] + R * np.cos(tangent_angle2), circle[1] + R * np.sin(tangent_angle2))

    return [Line(Point(point),Point(tangent_point1)),Line(Point(point),Point(tangent_point2))]


deviation = 0.01  # 误差
UAV_p = np.array([300,300])
target_p = np.array([700,500])

UAV = Point(UAV_p)
target = Point(target_p)
R0 = 50  # 最小转弯半径
phi0 = np.pi / 4  # 速度方向 [0,2pi]
# 以上为所有已知信息

# 1. 求两个圆心，判断出采用哪一个圆
# 2. 求切线
# 3. 确定用那一段弧长

# 1.求两个圆心，判断出采用哪一个圆
c1 = Point(UAV.x + R0 * np.sin(phi0), UAV.y - R0 * np.cos(phi0))
c2 = Point(UAV.x - R0 * np.sin(phi0), UAV.y + R0 * np.cos(phi0))
len1 = c1.distance(target)
len2 = c2.distance(target)
center = c1
if len2 > len1:
    center = c2
# 2. 求切线
circle = Circle(center, R0)
#tangent_lines = circle.tangent_lines(target)
tangent_lines=Tangent_lines(circle,target)


tangent_line1 = tangent_lines[0]  # 注意这里的切线方向是从target-> 切点
tangent_line1 = Line(tangent_line1.p2, tangent_line1.p1)  # 改为从切点->target
tangent_point1 = tangent_line1.p1  # 切点1
y = float((target.y - tangent_point1.y).evalf())
x = float((target.x - tangent_point1.x).evalf())
tangent_angle1 = np.arctan2(y, x)  # arctan2(y,x) 向量(x,y)的角度[-pi,pi]

tangent_line2 = tangent_lines[1]
tangent_line2 = Line(tangent_line2.p2, tangent_line2.p1)  # 改为从切点->target
tangent_point2 = tangent_line2.p1  # 切点２
y = float((target.y - tangent_point2.y).evalf())
x = float((target.x - tangent_point2.x).evalf())
tangent_angle2 = np.arctan2(y, x)  # arctan2(y,x) 向量(x,y)的角度[-pi,pi]

# 3. 确定用哪一段弧长
# a. 确定用顺时针还是逆时针
vec1 = [UAV.x - center.x, UAV.y - center.y]
vec2 = [np.cos(phi0), np.sin(phi0)]
direction = np.sign(vec1[0] * vec2[1] - vec1[1] * vec2[0])  # 1 表示逆时针 -1 表示顺时针
# b. 判断是哪一个切点，哪一段弧
sin1 = float(tangent_point1.distance(UAV).evalf()) / (2 * R0)
angle1 = 2 * np.arcsin(sin1)  # 无人机位置与切点之间的弧度[0,pi] 小弧
sin2 = float(tangent_point2.distance(UAV).evalf()) / (2 * R0)
angle2 = 2 * np.arcsin(sin2)

tangent_point = []
hudu = 0

# 判断式的意思  角度要在误差范围内相隔2kpi，使用modf(abs 把值控制在0-1之内，误差范围内靠近1，靠近0 都ok  用于0.5之间的距离来判断
if abs(modf(abs(direction * angle1 + phi0 - tangent_angle1) / (2 * np.pi))[0] - 0.5) > 0.5 - deviation:
    tangent_point = tangent_point1
    hudu = angle1
# modf 返回浮点数的小数部分和整数部分modf(1.23) return [0.23,1]
elif abs(modf(abs(direction * (2 * np.pi - angle1) + phi0 - tangent_angle1) / (2 * np.pi))[0] - 0.5) > 0.5 - deviation:
    tangent_point = tangent_point1
    hudu = 2 * np.pi - angle1
elif abs(modf(abs(direction * angle2 + phi0 - tangent_angle2) / (2 * np.pi))[0] - 0.5) > 0.5 - deviation:
    tangent_point = tangent_point2
    hudu = angle2
elif abs(modf(abs(direction * (2 * np.pi - angle2) + phi0 - tangent_angle2) / (2 * np.pi))[0] - 0.5) > 0.5 - deviation:
    tangent_point = tangent_point2
    hudu = 2 * np.pi - angle2

path_length = R0 * hudu + float(tangent_point.distance(target).evalf())
print(path_length)

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111)

start = np.arctan2(float((UAV.y - center.y).evalf()), float((UAV.x - center.x).evalf())) * 180 / np.pi
end = start + hudu * 180 / np.pi
if direction == -1:
    end = start - hudu * 180 / np.pi
    start, end = end, start
# add a wedge
c = [float(center.x.evalf()), float(center.y.evalf())]
wedge = mpatches.Wedge(c, R0, start, end)
ax.add_patch(wedge)

ax.scatter(float(tangent_point.x.evalf()), float(tangent_point.y.evalf()), s=60, marker='^', color='blue', alpha=0.8)
ax.scatter(float(UAV.x.evalf()), float(UAV.y.evalf()), s=60, marker='^', color='blue', alpha=0.8)
ax.scatter(float(target.x.evalf()), float(target.y.evalf()), s=60, marker='^', color='blue', alpha=0.8)
ax.plot((float(tangent_point.x.evalf()), float(target.x.evalf())),
        (float(tangent_point.y.evalf()), float(target.y.evalf())))
plt.show()
