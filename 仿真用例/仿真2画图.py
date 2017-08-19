from scipy import io
import matplotlib.pyplot as plt

import numpy as np

plt.rcParams['font.sans-serif']=['SimHei']
plt.rcParams['axes.unicode_minus']=False

data=io.loadmat('popSize_20_UAVunm_6.mat')
data=data['data']

time=[ sum(data_i[0][0])/len(data_i[0][0]) for data_i in data]
x=np.array([data_i[1][0][0] for data_i in data])
x=x*20

plt.plot(x,time,linewidth=1.3,label=u'5架无人机')
##########
data=io.loadmat('popSize_20_UAVunm_11.mat')
data=data['data']

time=[ sum(data_i[0][0])/len(data_i[0][0]) for data_i in data]
x=np.array([data_i[1][0][0] for data_i in data])
x=x*20
plt.plot(x,time,linewidth=1.3,label=u'10架无人机')

#####
data=io.loadmat('popSize_20_UAVunm_16.mat')
data=data['data']

time=[ sum(data_i[0][0])/len(data_i[0][0]) for data_i in data]
x=np.array([data_i[1][0][0] for data_i in data])
x=x*20
plt.plot(x,time,linewidth=1.3,label=u'15架无人机')
##############
plt.legend()
plt.xlabel(u'种群个数', fontsize=13)
plt.ylabel('时间/s', fontsize=13,horizontalalignment='right',rotation='horizontal')
plt.plot(np.linspace(20,160,50),np.ones(50),linestyle='dashed')
plt.show()