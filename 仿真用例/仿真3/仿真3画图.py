import matplotlib.pyplot as plt


import numpy as np

plt.rcParams['font.sans-serif']=['SimHei']
plt.rcParams['axes.unicode_minus']=False

y1=[32.48, 66.26, 89.36, 1]
y2=[23.31, 52.25, 79.25, 1]
x=[5,10,15,20]
plt.plot(x,y1,linewidth=1.3,label=u'并行NSGA')
plt.plot(x,y2,linewidth=1.3,label=u'5架无人机')