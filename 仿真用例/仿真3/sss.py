# code # utf-8
import os
from scipy import io
# filename = 'Lineradd_data_5_10_300.mat'
filename = 'NSGA_data10_10_400.mat'

data = io.loadmat(filename)

percent=0
for row in data['data']:
    percent+=row[2]
percent/=len(data['data'])
print(percent)
print(len(data['data']))
# [32.48, 66.26, 89.36, 1]
# [23.31, 52.25, 79.25, 1]
