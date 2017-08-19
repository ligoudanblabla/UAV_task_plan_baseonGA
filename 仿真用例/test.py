from scipy import io
data=io.loadmat('popSize_20_UAVunm_11.mat')
data=data['data']
print(data['data'])