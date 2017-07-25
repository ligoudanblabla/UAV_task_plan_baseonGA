
# import copy
# from math import modf
# from operator import itemgetter
# from scipy import  io
# import matplotlib.pyplot as plt
# import numpy as np
# from sympy import Point, Circle, Line
# import time

#
# data1=np.array([[5,2.1],[10,2.5],[15,2.8],[20,3.1]])
# data2=np.array([[5, 4.2],[10,5.5],[15,6.2],[20,7.1]])
# plt.plot(data1[:,0],data1[:,1],label='PopSize=80',)
# plt.plot(data2[:,0],data2[:,1],label='PopSize=120',)
# plt.scatter(data1[:, 0], data1[:, 1], s=50, marker='o', color='red', alpha=0.8)
# plt.scatter(data2[:, 0], data2[:, 1], s=50, marker='o', color='blue', alpha=0.8)
# plt.ylabel('time(s)',fontsize=14)
# plt.xlabel('UAV_num',fontsize=14)
# plt.legend(fontsize=14)
# plt.show()

# var='0123456789abcdef'
# for str in var:
#     data=int(str,16)
#     b=bin(data)
#
#     print(','.join(b[2:]))
str='00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FF 00 00 00 3F 00 00 00 FF 00 00 00 FF 00 00 0F C0 00 00 FF 00 00 00 FF C0 00 00 7F 00 00 00 FF 00 00 00 FE 00 00 0F C0 00 00 FF 00 00 00 FF E0 00 00 FF 00 00 00 FF 00 00 00 80 00 00 00 FC 00 00 00 F0 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00'
l=str.split(' ')
sss=''.join(l)
str=''
for i in sss:
    str+='\''+i+'\''+','
print(str)
f=open('file.txt','w')
f.write(str)
