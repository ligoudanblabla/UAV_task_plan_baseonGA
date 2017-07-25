import copy
from math import modf
from operator import itemgetter

import matplotlib.pyplot as plt
import numpy as np
from sympy import Point, Circle, Line
import time


def sign1(x):
    if x >= 0:
        return 1
    else:
        return -1


def sign2(x):
    if x > 0:
        return 1
    else:
        return -1


def sat(x):
    if x > 1:
        x = 1
    elif x < -1:
        x = -1
    return x




def cal_border_hudu(c, site, phi, theta, R, l1):
    vec1 = site - c
    vec2 = [np.cos(phi), np.sin(phi)]
    direction = np.sign(vec1[0] * vec2[1] - vec1[1] * vec2[0])

    if direction == 1:
        # 1是逆时针 -1 是顺时针
        theta = np.pi - theta
        l1 = border[type % 2][1] - border[type % 2][0] - l1  # type 0 2 用的是x轴长度  type 1 3 用的是y轴的长度
    temp1 = 2 * theta * (sign1(l1 - 2 * R * np.sin(theta)) + 1) / 2
    temp2 = theta + np.pi - np.arcsin(sat(((l1 - R * np.sin(theta)) / R)))
    temp3 = (sign2(2 * R * np.sin(theta) - l1) + 1) / 2
    hudu = temp1 + temp2 * temp3
    return hudu, direction


# 边界处理
border = [[-1000, 1000], [-1000, 1000]]
site = np.array([1000, 950])
phi =  2* np.pi / 8
R = 200
phi = phi % (2 * np.pi)  # 把角度变成[0,2pi]

theta_reduced = [0, -np.pi / 2, np.pi, np.pi / 2]
type = -1
theta = phi
l1 = 0
if site[1] >= border[1][1] and 0 < phi < np.pi:
    # 大于y的上界
    type = 0
    l1 = border[0][1] - site[0]
elif site[0] >= border[0][1] and (0 < phi < np.pi / 2 or 3 * np.pi / 2 < phi <= np.pi * 2):
    # 大于x的上界
    type = 1
    l1 = site[1] - border[1][0]
elif site[1] <= border[1][0] and np.pi <= phi <= np.pi * 2:
    # y   小于y的下界
    type = 2
    l1 = site[0] - border[0][0]
elif site[0] <= border[0][0] and np.pi / 2 <= phi <= np.pi * 3 / 2:
    # 小于x的下界
    type = 3
    l1 = border[1][1] - site[1]

if type != -1:
    theta = (theta - theta_reduced[type]) % (2 * np.pi)
    # theta为速度方向顺时针旋转到边界线的弧度

    # 　第二步骤确定圆心和对应的顺时针逆时针，已经弧度
    c1 = np.array([site[0] + R * np.sin(phi), site[1] - R * np.cos(phi)])
    hudu1, direction1 = cal_border_hudu(c1, site, phi, theta, R, l1)

    c2 = [site[0] - R * np.sin(phi), site[1] + R * np.cos(phi)]
    hudu2, direction2 = cal_border_hudu(c2, site, phi, theta, R, l1)

    msg = [c1, hudu1, direction1]
    if hudu1 > hudu2:
        msg = [c2, hudu2, direction2]  # 选择小的那一边

## 画图观察
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

start = np.arctan2(site[1] - msg[0][1], site[0] - msg[0][0])
end = (start + msg[2] * msg[1])
if msg[2] == -1:
    start, end = end, start

fig = plt.figure()
ax = fig.add_subplot(111)
wedge = mpatches.Wedge(msg[0], R, start * 180 / np.pi, end * 180 / np.pi)
ax.add_patch(wedge)
ax.plot([site[0] - 100 * np.cos(phi), site[0] + 100 * np.cos(phi)],
        [site[1] - 100 * np.sin(phi), site[1] + 100 * np.sin(phi)])
ax.plot([border[0][0], border[0][1]], [border[1][1], border[1][1]])
ax.plot([border[0][1], border[0][1]], [border[1][0], border[1][1]])
ax.plot([border[0][0], border[0][1]], [border[1][0], border[1][0]])
ax.plot([border[0][0], border[0][0]], [border[1][0], border[1][1]])
plt.show()
