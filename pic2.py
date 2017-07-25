import copy
from math import modf
from operator import itemgetter
from scipy import io
import matplotlib.pyplot as plt
import numpy as np
from sympy import Point, Circle, Line
import time

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
UAVs_msg = [[10, 10, 160, 15, 100, 300, [1, 2, 3], 6],
            [150, 150, 0, 15, 100, 300, [2, 0, 1], 5],
            [900, 700, 225, 15, 100, 300, [1, 3, 1], 4],
            [-800, 800, 270, 15, 100, 300, [1, 2, 1], 3],
            [-900, -600, 60, 15, 100, 300, [1, 0, 0], 2],
            [600, -900, 100, 15, 100, 300, [1, 2, 3], 1],
            [-800, -900, 45, 15, 130, 300, [2, 1, 1], 1]
            ]
# UAVs_msg = [[10, 10, 160, 15, 100, 300, [1, 2, 3], 6],
#             [150, 150, 0, 15, 100, 300, [2, 0, 1], 5],
#             [900, 700, 225, 15, 100, 300, [1, 3, 1], 4],
#             [-800, 800, 270, 15, 100, 300, [1, 2, 1], 3],
#             [-900, -600, 60, 15, 100, 300, [1, 0, 0], 2],
#             [600, -900, 100, 15, 100, 300, [1, 2, 3], 1],
#             [-800, -900, 45, 15, 130, 300, [2, 1, 1], 1],
#             [-700, -900, 145, 15, 130, 300, [2, 1, 1], 1],
#             [800, -900, 145, 15, 130, 300, [2, 1, 1], 1],
#             ]
border = [[-1000, 1000], [-1000, 1000]]
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


class UAV:
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

    def move(self):
        self.path.append(self.site)
        if self.condition == 1:

            self.site = self.site + self.v * time_interval * np.array([np.cos(self.phi), np.sin(self.phi)])
            # 做一下边界处理
        else:

            self.site = self.planning_route.pop(0)
            if len(self.planning_route) == 0:
                # 都加完了，转为搜索
                self.condition = 1

    def search_target(self):
        # 判断当前范围内有没有目标
        detect_target = []  # 本次移动所发现目标
        for i in range(target_num):
            if Targets_condition[i] == 0:  # 已经处理过的目标
                continue
            target_site = np.array(Targets_msg[i][0:2])
            dis = np.sqrt(sum(np.square(target_site - self.site)))
            if dis <= self.detect_scope:
                detect_target.append(i)
        return detect_target


class GAPSO:
    # 有很多固定的变量就写成一个类通过调用类的指定函数来搞定
    # 用0,1向量表示解
    def __init__(self):
        self.popSize = 80
        self.crossoverRate = 0.8
        self.mutationRate = 0.2
        self.population = []
        # 存储种群，里面存储结构体结构体中包含基因
        self.value = []
        # 　种群对应的适应度
        self.fvalue = []  # 存储不同目标函数的值[[1,1,2],[1,1,2],...]
        self.rank = []  # 非支配排序 0 表示最高层
        self.corwed = []  # 拥挤度算子

    def InitPop(self):
        # 初始化种群
        self.population = []
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
            self.corwed[sbi_fvalue[0][1]] = np.inf
            self.corwed[sbi_fvalue[-1][1]] = np.inf
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
            return [np.inf, np.inf, np.inf]


    def cal_GAPSO(self, target, target_candidate, arrivals_time):

        self.target = target
        self.candidate = target_candidate
        self.gene_len = len(target_candidate)
        self.arrivals_time = np.array(arrivals_time)  # 每个地方选1的值

        iter_num = 20
        self.InitPop()
        for i in range(iter_num):
            self.Breed()


        # 取出非支配解
        pareto_list=[] # 非支配解集合,这里用的是强支配,对强支配再进行一下若支配处理,用最后的弱支配做解集合
        for i in range(self.popSize):
            if self.rank[i]==1:
                flag=1
                for j in range(len(pareto_list)):
                    if np.alltrue(self.population[i]==pareto_list[j]):
                        flag = 0
                        break
                if flag==1:
                    pareto_list.append(self.population[i])
        return self.choose_one(pareto_list)


    def choose_one(self,pareto_list):
        # 从 pareto解集中取出一个解
        priority_value=1
        value_list=[]
        for gene in pareto_list:
            value_list.append([gene,self.CalFit(gene)])
        value_list.sort(key=lambda x:x[1][priority_value])

        best_gene=value_list[0][0]
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
            return copy.deepcopy(self.population[candidateindex[0]])
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
                self.fvalue[i]=self.CalFit(babby1)
            if i + 1 < self.popSize:
                new_population.append(babby2)
                self.fvalue[i+1] = self.CalFit(babby2)
        self.population = new_population

        self.NSGAII()
        self.Corwed()

    def Crossover(self, father, mather):
        # 选出两个点进行交叉
        index = np.floor(self.gene_len / 2)
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


def clash_avoid(group_find_targets):
    # 分配好哪架无人机处理哪个目标，group_find_targets [[1,[2,3,4]],...] 第一架无人机发现了目标2，3,4,
    # 返回[[1,3],[2,2]] 类型，表示第一架无人机围绕第三个目标组建联盟，第二架无人机围绕第2个目标组建联盟
    # 依次组建联盟
    UAV_task = []  # 返回数据
    # 根据无人机令牌数，从大到小排序
    group_find_targets = sorted(group_find_targets, key=lambda x: UAV_groups[x[0]].priority, reverse=True)

    for find_msg in group_find_targets:
        UAVi = find_msg[0]  # UAV index
        find_target = copy.copy(find_msg[1])  # 发现目标集合

        # 删除已经被挑走的目标
        for cp in UAV_task:
            if cp[1] in find_target:
                find_target.remove(cp[1])

        if find_target != []:
            # 按照目标令牌数从大到小排序
            find_target = sorted(find_target, key=lambda tar_index: Targets_msg[tar_index][3], reverse=True)
            # 选择第一个目标，建立联盟
            UAV_task.append([UAVi, find_target[0]])
    return UAV_task


def Arrivals_time(target_index, target_candidate):
    # 计算候选集合到目标的最短时间
    arrivals_time = []
    target = Targets_msg[target_index]
    for UAV_index in target_candidate:
        UAV = UAV_groups[UAV_index]
        arrivals_time.append(Arrival_time(UAV, target, UAV.r_min))
    return arrivals_time


def Arrival_time(UAV, target, R0):
    direction, hudu, tangent_site, center = Dubins_msg(UAV, target, R0)
    path_length = R0 * hudu + np.sqrt(np.sum((np.array(target[0:2]) - tangent_site) ** 2))
    return path_length / UAV.v


def Tangent_lines(circle_C, point_P):
    # 圆外一点到圆的切线，
    # 返回从point到切点的line
    R = float(circle_C.radius.evalf())
    circle = [float(circle_C.center.x.evalf()), float(circle_C.center.y.evalf())]
    point = [float(point_P.x.evalf()), float(point_P.y.evalf())]

    circle_point_angle = np.arctan2(point[1] - circle[1], point[0] - circle[0])
    cos = R / np.sqrt(np.sum((np.array(circle) - np.array(point)) ** 2))
    hudu_half = np.arccos(cos)

    tangent_angle1 = circle_point_angle + hudu_half
    tangent_point1 = Point(circle[0] + R * np.cos(tangent_angle1), circle[1] + R * np.sin(tangent_angle1))

    tangent_angle2 = circle_point_angle - hudu_half
    tangent_point2 = Point(circle[0] + R * np.cos(tangent_angle2), circle[1] + R * np.sin(tangent_angle2))

    return [Line(Point(point), Point(tangent_point1)), Line(Point(point), Point(tangent_point2))]


def Dubins_msg(UAV, target, R0):
    # 单架无人机到达目标点所需时间
    # 　这里的UAV和target是无人机和目标对象
    v = UAV.v  # 飞机速度
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
    center = Point(round(center.x.evalf(), 4), round(center.y.evalf(), 4))
    circle = Circle(center, R0)
    # start=time.time()
    # tangent_lines = circle.tangent_lines(target_p)
    tangent_lines = Tangent_lines(circle, target_p)
    # end=time.time()
    # print(end-start)
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

    # 返回 旋转方向 弧度 切点坐标 圆心坐标
    return direction, hudu, (float(tangent_point.x.evalf()), float(tangent_point.y.evalf())), (
        float(center.x.evalf()), float(center.y.evalf()))


# def Arrivals_time(target, target_candidate):
#     # 计算候选集合到目标的最短时间
#     candidate_num = len(target_candidate)
#     arrivals_time = np.zeros(candidate_num)
#     target_site = np.array(Targets_msg[target][0:2])
#
#     for i in range(candidate_num):
#         uav_index = target_candidate[i]
#         uav_site = UAV_groups[uav_index].site
#         uav_v = UAV_groups[uav_index].v
#         dis = np.sqrt(sum(np.square(uav_site - target_site)))
#         arrivals_time[i] = dis / uav_v
#     return arrivals_time

def Path_plan(target_index, coalition, cost_time):
    # 对联盟成员，路径进行规划，调整转弯半径，指定时间到达目标点，cost_time为花费时间，
    # 注意这里用的都是编号，而不是对象
    target = Targets_msg[target_index]
    for UAV_index in coalition:
        UAV = UAV_groups[UAV_index]
        fixtime_R = FixTime_R(UAV, target, cost_time)
        Dubins_path_plan(UAV, target, fixtime_R)


def Dubins_path_plan(UAV, target, R0):
    direction, hudu, tangent_site, center = Dubins_msg(UAV, target, R0)
    # 弧度间隔
    len_interval = UAV.v * time_interval
    hudu_interval = len_interval / R0
    theta0 = np.arctan2(UAV.site[1] - center[1], UAV.site[0] - center[0])
    theta_add = 0
    # 添加弧线段
    while abs(abs(theta_add) - hudu) > deviation and abs(theta_add) < hudu:  # 计算会有一定的误差
        theta_add += direction * hudu_interval  # 顺时针为正，逆时针为负
        theta = theta0 + theta_add
        point = [center[0] + R0 * np.cos(theta), center[1] + R0 * np.sin(theta)]
        UAV.planning_route.append(point)
    # 添加直线段
    line_angle = np.arctan2(target[1] - tangent_site[1], target[0] - tangent_site[0])

    UAV.phi = line_angle  # 速度方向
    UAV.condition = 2  # 进入攻击状态

    start_site = UAV.planning_route[-1]
    tagent_now_dis = 0
    tagent_target_dis = np.sqrt(np.sum((np.array(target[0:2]) - tangent_site) ** 2))
    while abs(tagent_now_dis - tagent_target_dis) > deviation and tagent_now_dis < tagent_target_dis:
        new_point = np.array(
            [start_site[0] + len_interval * np.cos(line_angle), start_site[1] + len_interval * np.sin(line_angle)])
        UAV.planning_route.append(new_point)
        start_site = new_point
        tagent_now_dis = np.sqrt(np.sum((start_site - tangent_site) ** 2))
    io.savemat(r'./path.mat', {'data': np.array(UAV.planning_route)})


def FixTime_R(UAV, target, cost_time):
    # 固定时间内无人机从当前点到目标点所需转弯半径
    # 调整UAV的转弯半径，使其长度匹配，用二分法处理
    # 　这里的UAV和target是无人机和目标对象
    dis = np.sqrt((target[0] - UAV.site[0]) ** 2 + (target[1] - UAV.site[1]) ** 2)

    t_min = Arrival_time(UAV, target, UAV.r_min)  # 二分法的下界
    R_min = UAV.r_min

    R_max = abs(border[0][0] - border[0][1]) / 2
    t_max = Arrival_time(UAV, target, R_max)  # 二分法的

    t = t_min
    R = R_min

    while abs(t - cost_time) > deviation:
        if t < cost_time:
            t_min = t
            R_min = R
        if t > cost_time:
            t_max = t
            R_max = R
        R = (R_min + R_max) / 2
        t = Arrival_time(UAV, target, R)
    return R


def Form_coalition(target, target_candidate):
    # 联盟组建，给出目标和候选无人机，返回联盟集合，和完成任务时间时间，
    # 1. 求解各个无人机的最短到达时间
    # 2. 采用并行多目标GAPSO算法进行求解
    arrivals_time = Arrivals_time(target, target_candidate)

    return ggogo.cal_GAPSO(target, target_candidate, arrivals_time)


def plot_UAV_target():
    uav_site = np.array([i[0:2] for i in UAVs_msg])
    target_site = np.array([i[0:2] for i in Targets_msg])
    plt.scatter(uav_site[:, 0], uav_site[:, 1], s=100, marker='^', color='blue', alpha=0.8, label='UAV')
    plt.scatter(target_site[:, 0], target_site[:, 1], s=100, marker='o', color='red', alpha=0.8, label='Target')
    for i in range(UAV_num):
        plt.annotate(
            'UAV%s' % (i + 1),
            xy=(uav_site[i, 0], uav_site[i, 1]),
            xytext=(0, -10),
            textcoords='offset points',
            ha='center',
            va='top')
    for i in range(target_num):
        plt.annotate(
            'Target%s' % (i + 1),
            xy=(target_site[i, 0], target_site[i, 1]),
            xytext=(0, -10),
            textcoords='offset points',
            ha='center',
            va='top')
    plt.legend(loc=9)
    # 初始化无人机群


def liner_add(target, target_candidate, arrivals_time):
    data = [ i for i in  zip(target_candidate,arrivals_time)]
    data.sort(key=lambda x:x[1]) # 根据到达时间进行排序
    coalition=[]
    max_time=0
    resource=np.zeros(len(Targets_msg[0][2]))
    for i in data:
        resource+=UAV_groups[i[0]].resource
        coalition.append(i[0])
        max_time=i[1]
        if np.alltrue(resource>=np.array(Targets_msg[target][2])):
            break
    return coalition, max_time
UAV_groups = []
for msg_i in UAVs_msg:
    UAV_groups.append(UAV(msg_i))

ggogo = GAPSO()
if __name__ == '__main__':
    #
    # coa,time=ggogo.cal_GAPSO(1,[1,2,3,5],[50,80,100,170])
    # print('')

    for t in np.arange(0, 144, time_interval):
        if t == 63.1:
            print('tets')
        group_find_targets = []  # 存放当下无人机发现目标集  ([i,[1 3 6]],[...])

        # 执行搜索
        for UAVi, i in zip(UAV_groups, range(UAV_num)):  # 这里其实可以写成多线程技术，反正各架无人机此时一样

            if UAVi.condition == 1:
                # 执行搜索任务
                find_targets = UAVi.search_target()

                if find_targets != []:  # 若发现目标则添加进去
                    group_find_targets.append([i, find_targets])

                    # # 先判断当前的情况再move
                    # UAVi.move()

        # 进行冲突处理和组建联盟
        UAV_task = clash_avoid(group_find_targets)  # 返回哪家无人机处理哪个目标

        # 构造一个不含队长的候选集
        candidate = []
        for i in range(UAV_num):
            if UAV_groups[i].condition == 1 or UAV_groups[i].condition == 3:
                candidate.append(i)
        for cp in UAV_task:
            if cp[0] in candidate:
                candidate.remove(cp[0])  # remove 队长

        # 采用优化算法来建立联合打击联盟
        for cp in UAV_task:
            captain = cp[0]  # captain 不一定被选中
            target = cp[1]  #
            target_candidate = candidate.copy()  # 此目标的候选集合
            target_candidate.insert(0, captain)  # 把captain插到最前面

            arrivals_time = Arrivals_time(target, target_candidate)
            start = time.time()
            # coalition, cost_time = ggogo.cal_GAPSO(target, target_candidate, arrivals_time)
            coalition, cost_time = liner_add(target, target_candidate, arrivals_time)
            end = time.time()
            print('GA cost time %s ', end - start)
            # 组建联盟，并返回花费时间
            # 获得coalition，后需进行航迹规划，并且把里面的无人机状态改为2，最后在candidate中remove掉这些元素
            # 并且去掉相应的资源
            Path_plan(target, coalition, cost_time)
            for i in coalition:
                # 把进行攻击无人机状态改为2，最后在candidate中remove掉这些元素
                UAV_groups[i].condition = 2
                if i in candidate:
                    candidate.remove(i)
                # 消耗资源
                for j in range(len(Targets_msg[target][2])):
                    if UAV_groups[i].resource[j] >= Targets_msg[target][2][j]:
                        UAV_groups[i].resource[j] -= Targets_msg[target][2][j]
                        Targets_msg[target][2][j] = 0
                    else:
                        Targets_msg[target][2][j] -= UAV_groups[i].resource[j]
                        UAV_groups[i].resource[j] = 0
            # 如果captain不在coalition 里面，则把captain 添加到candidate 中
            if captain not in coalition:
                candidate.append(captain)

            # 打完了，把目标的状态位设置一下
            Targets_condition[target] = 0
            print('find target %s', target)
        # 无人机走一步
        for UAVi in UAV_groups:
            UAVi.move()
            # if UAV_groups[0].planning_route == [] or UAV_groups[2].planning_route == []:
            #     print('test')
    for UAVi in UAV_groups:
        plt.plot(np.array(UAVi.path)[:, 0], np.array(UAVi.path)[:, 1], linewidth=1)
    plot_UAV_target()
    plt.show()
