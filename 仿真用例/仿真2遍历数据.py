# 有边界处理用的长Dubins路径,多线程
import copy
from math import modf
from operator import itemgetter
from scipy import io
import matplotlib.pyplot as plt
import numpy as np
from sympy import Point, Circle, Line
import time
from threading import Thread
import multiprocessing
from scipy import io
'''
每架无人机由一个状态位置0，表示正在搜索；1表示正在执行，攻击任务（此时路径已经规划好）；3 表示正在处理边界，
冲突消解，多架无人机同时发现多个目标，给无人机和目标令牌，令牌多的无人机先选，选择令牌多的目标。关键问题，发现两个目标只能处理一个？


程序流程
对时间进行循环，每隔0.1s 采样一下
	对于状态为0的无人机，此时应该是沿着速度方向前进0.1s，到新的位置后判断是否发现目标。是否即将出界
1．	发现目标，首先判断自己能不能够打，不能打的话开始组建联盟。
组建联盟：这里不存在信息发送，队长直接计算，其他无人机到达的时间，然后自行优化建立联盟，状态为0的无人机，和状态为3的无人机，被选中后，转为状态2，并获得规划路径
2．	发现快要出界了，那么提前规划好一段边缘路径。
对于状态为0,3的无人机，沿着之前规划好的路径走就行了，若规划的路径走完了，那么状态变为0,开始搜索。
'''
#          x,y,phi(degree),v,r,s 资源 令牌数   分布表示 坐标（x,y），初始速度方向phi，速度大小v，最小转弯半径r，探测距离s
# UAVs_msg = [[10, 10, 160, 20, 150, 320, [1, 2, 3], 6],
#             [150, 150, 0, 25, 120, 330, [2, 0, 1], 5],
#             [900, 700, 225, 18, 100, 300, [1, 3, 1], 4],
#             [-800, 800, 270, 19, 100, 300, [1, 2, 1], 3],
#             [-900, -600, 60, 15, 100, 300, [1, 0, 0], 2],
#             [600, -900, 100, 15, 130, 300, [1, 2, 3], 1],
#             [-200, 900, 270, 15, 130, 300, [1, 2, 3], 1]
#             ]
UAVs_msg = [[10, 10, 160, 15, 100, 300, [1, 2, 3], 10],
            [150, 150, 0, 15, 100, 300, [2, 0, 1], 9],
            [900, 700, 225, 15, 100, 300, [1, 3, 1], 8],
            [-800, 800, 270, 15, 100, 300, [1, 2, 1], 7],
            [-900, -600, 60, 15, 100, 300, [1, 0, 0], 6],
            [600, -900, 100, 15, 100, 300, [1, 2, 3], 5],
            [150, 250, 0, 15, 100, 300, [2, 0, 1], 4],
            [900, 300, 225, 15, 100, 300, [1, 3, 1], 3],
            [-800, -200, 170, 15, 100, 300, [1, 3, 1], 2],
            [-900, -200, 160, 15, 100, 300, [1, 1, 1], 1],
            [600, -500, 140, 15, 100, 300, [2, 2, 3], 0],
            [150, 550, 0, 15, 100, 300, [2, 0, 1], -1],
            [900, 350, 225, 15, 100, 300, [1, 3, 1], -2],
            [-800, -250, 170, 15, 100, 300, [1, 3, 1], -3],
            [-900, -250, 160, 15, 100, 300, [1, 1, 1], -4],
            [600, -550, 140, 15, 100, 300, [2, 2, 3], -5]
            ]

Insted_INF=99999

border = np.array([[-1000, 1000], [-1000, 1000]])
UAV_num = len(UAVs_msg)
# x,y,resource,令牌
Targets_msg = [[300, 0, [3, 2, 2], 3],
               [-600, 500, [2, 1, 1], 2],
               [0, 300, [0, 0, 1], 1]
               ]

target_num = len(Targets_msg)
Targets_condition = np.ones(target_num)  # 目标状态 1表示未被摧毁，0表示被摧毁了

run_time = 1000  # 总共仿真时间
time_interval = 0.1  # 采样时间间隔
deviation = 0.01  # 误差
### 多进程GA
GA_thread_num =1# 进程个数

GA_iter_num = 40  # 遗传算法迭代次数
GA_exchang = 20  # 数据迁移代数
popSize=20

GA_runtimes=[]


class self:
    def __init__(self, msg):
        # 无人机属性
        self.site = np.array(msg[0:2])  # 当前所在位置
        self.phi = np.radians(msg[2])  # 当前航向角，采用弧度制
        self.v = msg[3]  # 无人机速度
        self.r_min = msg[4]  # 最小转弯半径
        self.detect_scope = msg[5]  # 搜索半径
        self.resource = msg[6]  # 携带资源
        self.priority = msg[7]  # 令牌数，优先级别

        # 无人机航迹
        self.path = []  # 已经飞过的航迹，里面存储位置np.zeros(2),类型
        self.planning_route = []  # 路径规划，用于状态
        self.condition = 1  # 1 表示搜索，2表示执行攻击任务，3表示边界最小转移


class GAPSO:
    # 修改 加入线程处理
    # 有很多固定的变量就写成一个类通过调用类的指定函数来搞定
    # 用0,1向量表示解
    def __init__(self):
        self.popSize = popSize
        self.crossoverRate = 0.8
        self.mutationRate = 0.2
        self.population = []
        # 存储种群，里面存储结构体结构体中包含基因
        self.value = []
        # 　种群对应的适应度
        self.fvalue = []  # 存储不同目标函数的值[[1,1,2],[1,1,2],...]
        self.rank = []  # 非支配排序 0 表示最高层
        self.corwed = []  # 拥挤度算子
        # 这里偷懒了，其实最好是吧population，fvalue，rank，corwed，写到一个类people类里面

    def InitPop(self):
        # 初始化种群
        self.population = []
        self.fvalue = []
        self.corwed = []
        self.rank = []

        for i in range(self.popSize):
            gene = np.random.randint(0, 2, self.gene_len)
            fvalue = self.CalFit(gene)
            self.population.append(gene)
            self.fvalue.append(fvalue)
        self.NSGAII()
        self.Corwed()

    def NSGAII(self):
        # 计算种群的非支配排序和拥挤度算子
        S = [[] for i in range(self.popSize)]
        n = np.zeros(self.popSize)
        rank = np.zeros(self.popSize)
        F = []
        H = []
        for i in range(self.popSize):
            for j in range(self.popSize):
                result = self.control(self.fvalue[i], self.fvalue[j])
                if result == 1:
                    S[i].append(j)
                elif result == -1:
                    n[i] = n[i] + 1
            if n[i] == 0:
                rank[i] = 1  # rank从1开始
                F.append(i)
        p = 1
        while len(F) != 0:
            H = []
            for i in F:
                for j in S[i]:
                    n[j] = n[j] - 1
                    if n[j] == 0:
                        rank[j] = p + 1
                        H.append(j)
            p = p + 1
            F = H
        self.rank = rank

    def Corwed(self):
        # 计算拥挤度算子
        # 遍历不同的目标函数
        self.corwed = np.zeros(self.popSize)
        value_index = [[value.copy(), i] for value, i in zip(self.fvalue, range(self.popSize))]
        # 前面是value 后面是index
        for m in range(len(self.fvalue[0])):
            sbi_fvalue = sorted(value_index, key=lambda x: x[0][m])
            # 按照第i个value从小到大排序，sorted不改变value_index的值
            self.corwed[sbi_fvalue[0][1]] = Insted_INF
            self.corwed[sbi_fvalue[-1][1]] = Insted_INF
            for i in range(1, self.popSize - 1):
                self.corwed[sbi_fvalue[i][1]] += sbi_fvalue[i + 1][0][m] - sbi_fvalue[i + 1][0][m]

    def control(self, fvalue1, fvalue2):
        # fvalue1 支配 fvalue2 返回1
        # 互不支配 返回 0
        # 2 支配1 返回-1
        # 注意这里是值越小越优
        result = np.alltrue(np.array(fvalue1) > np.array(fvalue2))
        if np.alltrue(np.array(fvalue1) == np.array(fvalue2)):
            return 0
        elif np.alltrue(np.array(fvalue1) <= np.array(fvalue2)):
            return 1
        elif np.alltrue(np.array(fvalue1) >= np.array(fvalue2)):
            return -1

    def CalFit(self, gene):

        resourse = [gene[i] * np.array(UAV_groups[self.candidate[i]].resource) for i in range(self.gene_len)]
        # resourse = [gene[i] * np.array(UAVs_msg[self.candidate[i]][6]) for i in range(self.gene_len)]
        resourse = np.sum(resourse, axis=0)  # 求烈和
        if np.alltrue(resourse >= np.array(Targets_msg[self.target][2])):
            v1 = np.max(np.array(gene) * self.arrivals_time)
            v2 = sum(gene)
            v3 = np.sum(np.array(gene) * self.arrivals_time)
            # return 0.1 * v1 + 2 * v2 + 0.1 * v3 / v2
            return [v1, v2, v3]
        else:
            return [Insted_INF, Insted_INF, Insted_INF]

    # def process(self):
    #     # args是关键字参数，需要加上名字，写成args=(self,)
    #     th1 = Thread(target=GAPSO.run, args=(self,ll))
    #     th1.start()
    #     th1.join()

    def run(self, ll, iter_num=20):
        # print('第 %d 个开始时间%s' % (ll, time.ctime()))
        # print('iter_num:%s'% iter_num)
        # time.sleep(2)
        for i in range(iter_num):
            self.Breed()
        # print('第 %d 个结束时间%s' % (ll, time.ctime()))

    def set_parameter(self, target, target_candidate, arrivals_time):
        # 设置参数,初始化种群
        self.target = target
        self.candidate = target_candidate
        self.gene_len = len(target_candidate)
        self.arrivals_time = np.array(arrivals_time)  # 每个地方选1的值

        self.InitPop()

    def getPareto_list(self):
        # 取出非支配解
        pareto_list = []  # 非支配解集合,这里用的是强支配,对强支配再进行一下若支配处理,用最后的弱支配做解集合
        for i in range(self.popSize):
            if self.rank[i] == 1:
                flag = 1
                for j in range(len(pareto_list)):
                    if np.alltrue(self.population[i] == pareto_list[j]):
                        flag = 0
                        break
                if flag == 1:
                    pareto_list.append(self.population[i])
        return pareto_list

    def cal_GAPSO(self, target, target_candidate, arrivals_time):
        # 改成多线程之后这个函数功能要分拆成3个,1. 赋值 ,2. 迭代  3. 取值
        self.target = target
        self.candidate = target_candidate
        self.gene_len = len(target_candidate)
        self.arrivals_time = np.array(arrivals_time)  # 每个地方选1的值

        iter_num = 20
        self.InitPop()
        for i in range(iter_num):
            self.Breed()

        # 取出非支配解
        pareto_list = []  # 非支配解集合,这里用的是强支配,对强支配再进行一下若支配处理,用最后的弱支配做解集合
        for i in range(self.popSize):
            if self.rank[i] == 1:
                flag = 1
                for j in range(len(pareto_list)):
                    if np.alltrue(self.population[i] == pareto_list[j]):
                        flag = 0
                        break
                if flag == 1:
                    pareto_list.append(self.population[i])
        return self.choose_one(pareto_list)

    def best_pareto_gene(self):
        Pareto_list = self.getPareto_list()
        return self.pareto_best(Pareto_list)

    def pareto_best(self, pareto_list):
        priority_value = 1
        value_list = []
        for gene in pareto_list:
            value_list.append([gene, self.CalFit(gene)])
        value_list.sort(key=lambda x: x[1][priority_value])
        best_gene = value_list[0][0]
        return best_gene

    def gene_coal_maxtime(self, best_gene):
        # 给定基因返回基因对应的,联盟和联盟到达时间
        coalition = []
        max_time = 0
        for i in range(self.gene_len):
            if best_gene[i] == 1:
                coalition.append(target_candidate[i])
                if max_time < arrivals_time[i]:
                    max_time = arrivals_time[i]
        return coalition, max_time

    def choose_one(self, pareto_list):
        # 从 pareto解集中取出一个解
        priority_value = 1
        value_list = []
        for gene in pareto_list:
            value_list.append([gene, self.CalFit(gene)])
        value_list.sort(key=lambda x: x[1][priority_value])

        best_gene = value_list[0][0]
        coalition = []
        max_time = 0
        for i in range(self.gene_len):
            if best_gene[i] == 1:
                coalition.append(target_candidate[i])
                if max_time < arrivals_time[i]:
                    max_time = arrivals_time[i]
        return coalition, max_time


        # # 取出有效值
        # self.population.sort(key=itemgetter(1))

    def Filter(self):
        # 选择出父母，简单的竞标形式 选出2个两队
        # rank 是越小越优化 corwed是越大越优
        candidateindex = np.random.randint(0, self.popSize, 2)
        if self.rank[candidateindex[0]] < self.rank[candidateindex[1]]:
            return copy.deepcopy(self.population[candidateindex[0]])
        elif self.rank[candidateindex[0]] == self.rank[candidateindex[1]] and self.corwed[candidateindex[0]] >= \
                self.corwed[candidateindex[1]]:
            rand_num=np.random.rand()
            if rand_num>0.3:
                return copy.deepcopy(self.population[candidateindex[0]])
            else:
                return copy.deepcopy(self.population[candidateindex[1]])
        else:
            return copy.deepcopy(self.population[candidateindex[1]])

    def Breed(self):

        new_population = []
        for i in range(0, self.popSize, 2):  # interval is two
            father = self.Filter()
            mather = self.Filter()
            babby1, babby2 = self.Crossover(father, mather)  # 交叉变异 有关概率的事情都放在对应函数里面处理
            babby1 = self.Mutation(babby1)
            babby2 = self.Mutation(babby2)
            if i < self.popSize:
                new_population.append(babby1)
                self.fvalue[i] = self.CalFit(babby1)
            if i + 1 < self.popSize:
                new_population.append(babby2)
                self.fvalue[i + 1] = self.CalFit(babby2)
        self.population = new_population

        self.NSGAII()
        self.Corwed()

    def Crossover(self, father, mather):
        # 选出两个点进行交叉
        index = int(np.floor(self.gene_len / 2))
        babby1 = copy.deepcopy(father)
        babby2 = copy.deepcopy(mather)
        if np.random.rand() < self.crossoverRate:
            babby1[index:] = mather[index:]
            babby2[index:] = father[index:]
        return babby1, babby2

    def Mutation(self, people):
        # 变异函数
        if np.random.rand() < self.mutationRate:  # 0 to 1 random number
            index = np.random.randint(0, self.gene_len)
            people[index] = 1 - people[index]  # 1 变0  0 变成1
        return people

def cal_coalition_cost_time(target, target_candidate, arrivals_time):
    for i in range(GA_thread_num):
        GAPSO_list[i].set_parameter(target, target_candidate, arrivals_time)
    GA_iter_temp = 0

    start = time.time()
    while GA_iter_temp < GA_iter_num:
        threads = []

        for i in range(GA_thread_num):
            # t=Thread(target=GAPSO_list[i].run,args=(i,GA_exchang))
            t = multiprocessing.Process(target=GAPSO_list[i].run, args=(i,GA_exchang))
            # t = multiprocessing.Process(target=GAPSO_list[i].run)
            t.start()
            threads.append(t)

        for i in range(GA_thread_num):
            threads[i].join()
        # 每跑完一定代数进行一次数据迁移

        for i in range(GA_thread_num):
            # i把最好的给i-1
            best_gene = GAPSO_list[i].best_pareto_gene()

            # i-1的rank最烂的index   argsort() 不改变list   rank越大越烂
            worstgene_index_i_1 = np.argsort(GAPSO_list[i - 1].rank)[-1]
            GAPSO_list[i - 1].population[worstgene_index_i_1] = copy.deepcopy(best_gene)
            # 重新排序
            GAPSO_list[i - 1].NSGAII()
            GAPSO_list[i - 1].Corwed()

        GA_iter_temp += GA_exchang
    end = time.time()
    GA_runtimes.append(end - start)
    # print('GA cost time %s ' % (end - start))

    # 最后取出,所有子种群里面最好的
    All_Pareto_list = []
    for i in range(GA_thread_num):
        i_Pareto_list = GAPSO_list[i].getPareto_list()
        All_Pareto_list.extend(i_Pareto_list)
    best_gene = GAPSO_list[0].pareto_best(All_Pareto_list)
    coalition, cost_time = GAPSO_list[0].gene_coal_maxtime(best_gene)
    return coalition, cost_time


for thread_num in range(1,9):
    GA_thread_num=thread_num
    print('GA_thread_num:%s' % thread_num)
    for rerun in range(10):

        print('rerun:%s'%rerun)
        # coa,time=ggogo.cal_GAPSO(1,[1,2,3,5],[50,80,100,170])
        # print('')
        #####  初始化
        UAV_groups = []
        for msg_i in UAVs_msg:
            UAV_groups.append(self(msg_i))
        target=1

        target_candidate = [0, 2, 3, 4, 5, 6, 7, 8, 9, 10,11,12,13,14,15]
        arrivals_time = [20, 30, 40, 50, 89, 84, 24, 56, 66, 92,100,25,43,76,24]
        # target_candidate=[0,2,3,4,5,6,7,8,9,10]
        # arrivals_time = [20, 30, 40, 50, 89,84,24,56,66,92]
        # target_candidate = [0, 2, 3, 4, 5]
        # arrivals_time=[20,30,40,50,89]
        ggogo = GAPSO()
        ggogo.set_parameter(target, target_candidate, arrivals_time)


