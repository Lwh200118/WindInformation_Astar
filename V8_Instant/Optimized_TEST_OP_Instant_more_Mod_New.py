import os
import math
import pandas as pd
from scipy.interpolate import LinearNDInterpolator
import numpy as np
import matplotlib.pyplot as plt
import logging
import heapq

show_animation = True

def LineOfSight(s, e, obmap):
    x0, y0 = s
    x1, y1 = e
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    max_iterations = 10000  # 设置一个足够大的迭代次数上限
    iteration = 0

    while True:
        if int(y0) < 0 or int(y0) >= len(obmap) or int(x0) < 0 or int(x0) >= len(obmap[0]):
            return False  # 索引无效时返回False
        if x0 == x1 and y0 == y1:
            break
        if obmap[int(y0)][int(x0)]:
            return False
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
        iteration += 1
        if iteration > max_iterations:
            print("Warning: LineOfSight reached max iterations")
            break
    return True

def Pruning(path, obmap):
    if not path:
        raise ValueError("输入路径为空")

    pruned_path = [path[0]]  # 初始化修剪后的路径，首先包含起点
    k = 0  # 最后一个加入pruned_path的点的索引初始化为0
    i = 0  # 当前考虑的点的索引初始化为0

    while i < len(path) - 1:
        # 检查当前点与下一个点之间是否有直线视野
        if not LineOfSight(pruned_path[k], path[i + 1], obmap):
            # 如果没有直线视野，则将当前点加入到修剪后的路径
            k += 1
            pruned_path.append(path[i])
        i += 1  # 移动到路径中的下一个点

    # 确保终点被包含在修剪后的路径中
    if pruned_path[-1] != path[-1]:
        pruned_path.append(path[-1])

    return pruned_path
##
def read_data(filename):
    data = pd.read_csv(filename, delim_whitespace=True, 
                       names=['x', 'z', 'y', 'u', 'w', 'v'])
    return data

def interpolate_u(data, x, y):
    points = data[['x', 'y']].to_numpy()
    u_values = data['u'].to_numpy()
    interpolator = LinearNDInterpolator(points, u_values)
    u_value_at_coord = interpolator([x, y])
    if np.isnan(u_value_at_coord[0]):
        return 0  # 或其他合理的默认值
    else:
        return u_value_at_coord[0]

def interpolate_v(data, x, y):
    # 提取坐标和u值仅使用x和z进行插值
    points = data[['x', 'y']].to_numpy()  # 修改至二维坐标
    v_values = data['v'].to_numpy()

    # 创建插值器
    interpolator = LinearNDInterpolator(points, v_values)

    # 在指定的坐标上执行插值
    v_value_at_coord = interpolator([x, y])  # 仅需要x和z坐标
    if np.isnan(v_value_at_coord[0]):
        return 0  # 或其他合理的默认值
    else:
        return v_value_at_coord[0]

def interpolate_w(data, x, y):
    # 提取坐标和u值仅使用x和z进行插值
    points = data[['x', 'y']].to_numpy()  # 修改至二维坐标
    w_values = data['w'].to_numpy()

    # 创建插值器
    interpolator = LinearNDInterpolator(points, w_values)

    # 在指定的坐标上执行插值
    w_value_at_coord = interpolator([x, y])  # 仅需要x和z坐标
    if np.isnan(w_value_at_coord[0]):
        return 0  # 或其他合理的默认值
    else:
        return w_value_at_coord[0]


def create_square_obstacle(center, size):
    ox, oy = [], []
    half_size = size // 2
    left_bottom_x = center[0] - half_size
    left_bottom_y = center[1] - half_size

    # 添加四个角落的点
    ox.append(left_bottom_x)
    oy.append(left_bottom_y)
    ox.append(left_bottom_x + size)
    oy.append(left_bottom_y)
    ox.append(left_bottom_x)
    oy.append(left_bottom_y + size)
    ox.append(left_bottom_x + size)
    oy.append(left_bottom_y + size)

    # 添加四条边上剩余的点
    for i in range(1, size):
        ox.append(left_bottom_x + i)
        oy.append(left_bottom_y)
        ox.append(left_bottom_x)
        oy.append(left_bottom_y + i)
        ox.append(left_bottom_x + i)
        oy.append(left_bottom_y + size)
        ox.append(left_bottom_x + size)
        oy.append(left_bottom_y + i)

    return ox, oy

class AStarPlanner:
    def __init__(self, ox, oy, resolution, rr, data_dir):
        self.resolution = resolution
        self.rr = rr
        self.data_dir = data_dir  # 修改为数据目录
        self.current_data = None  # 当前风场数据
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()
        self.interpolation_cache_u = {}
        self.interpolation_cache_v = {}
        self.interpolation_cache_w = {}
        self.start_time = 5100  # 起始时间
        self.time_interval = 0.2   # 时间间隔
        self.U_t = 8  # 固定速度值 (m/s)!!!!!!!!!!!!!!!!!!!
        
        # 设置日志记录器
        self.logger = logging.getLogger('PathPlannerLogger')
        self.logger.setLevel(logging.DEBUG)
        
        # 创建文件处理器并指定编码为 utf-8
        fh = logging.FileHandler('path_planner.log', encoding='utf-8')
        fh.setLevel(logging.DEBUG)
        
        # 创建日志格式化器
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        
        # 将文件处理器添加到日志记录器
        self.logger.addHandler(fh)
    
    def interpolate_u(self, x, y):
        if (x, y) not in self.interpolation_cache_u:
            self.interpolation_cache_u[(x, y)] = interpolate_u(self.current_data, x, y)
        return self.interpolation_cache_u[(x, y)]

    def interpolate_v(self, x, y):
        if (x, y) not in self.interpolation_cache_v:
            self.interpolation_cache_v[(x, y)] = interpolate_v(self.current_data, x, y)
        return self.interpolation_cache_v[(x, y)]

    def interpolate_w(self, x, y):
        if (x, y) not in self.interpolation_cache_w:
            self.interpolation_cache_w[(x, y)] = interpolate_w(self.current_data, x, y)
        return self.interpolation_cache_w[(x, y)]
    
    def load_data(self, time):
        rounded_time = round(time * 5) / 5  # 四舍五入到最近的0.2秒的倍数
        folder_name = f"{rounded_time:.1f}"  # 修改为保留一位小数的字符串
        filename = os.path.join(self.data_dir, folder_name, "U_0_mySurfaces_60.dat")
        if os.path.exists(filename):
            self.current_data = read_data(filename)
        else:
            raise FileNotFoundError(f"文件 {filename} 不存在")
    
    class Node:
        def __init__(self, x, y, cost, parent_index, h=0.0, g=0.0, m=0, time=0.0):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
            self.h = h  # 启发式值
            self.g = g
            self.m = m  # 搜索深度属性
            self.time = time  # 飞行时间

        def __str__(self):    
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)


    def planning(self, sx, sy, gx, gy):
        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                        self.calc_xyindex(sy, self.miny),
                        0.0, -1, m=0, time=self.start_time)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                        self.calc_xyindex(gy, self.miny),
                        0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        self.load_data(self.start_time)  # 初始化时加载起始时间的风场数据

        while True:
            if not open_set:
                print("Open_set is empty...")
                break
            
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o], R=100, K=0.5))
            current = open_set[c_id]

            if show_animation:
                plt.plot(self.calc_grid_position(current.x, self.minx),
                        self.calc_grid_position(current.y, self.miny),
                        "xc")
                plt.gcf().canvas.mpl_connect('key_release_event',
                                            lambda event: [exit(0) if event.key == 'escape' else None]
                                            )
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal!")
                ngoal.parent_index = current.parent_index
                ngoal.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            self.expand_grid(current, ngoal, open_set, closed_set, c_id, nstart)

        pathx, pathy = self.calc_final_path(ngoal, closed_set)
        return pathx, pathy
    

    def expand_grid(self, current, ngoal, open_set, closed_set, c_id, nstart):
        for i, _ in enumerate(self.motion):
            new_x_index = current.x + self.motion[i][0]
            new_y_index = current.y + self.motion[i][1]
            node = self.Node(new_x_index, new_y_index, current.cost, c_id, m=current.m + 1)
            new_x = self.calc_grid_position(new_x_index, self.minx)
            new_y = self.calc_grid_position(new_y_index, self.miny)

            # 计算当前节点与终点的距离和原始起点与终点的距离比
            distance_to_goal = math.hypot(new_x - self.calc_grid_position(ngoal.x, self.minx), new_y - self.calc_grid_position(ngoal.y, self.miny))
            initial_distance_to_goal = math.hypot(self.calc_grid_position(ngoal.x, self.minx) - self.calc_grid_position(nstart.x, self.minx), self.calc_grid_position(ngoal.y, self.miny) - self.calc_grid_position(nstart.y, self.miny))
            if distance_to_goal / initial_distance_to_goal > 1.05:
                continue

            # 计算拓展方向与当前节点到终点的方向差距
            goal_vector = [self.calc_grid_position(ngoal.x, self.minx) - new_x, self.calc_grid_position(ngoal.y, self.miny) - new_y]
            current_vector = [new_x - self.calc_grid_position(current.x, self.minx), new_y - self.calc_grid_position(current.y, self.miny)]
            dot_product = goal_vector[0] * current_vector[0] + goal_vector[1] * current_vector[1]
            magnitude_goal = math.hypot(goal_vector[0], goal_vector[1])
            magnitude_current = math.hypot(current_vector[0], current_vector[1])

            # 检查向量模是否为零
            if magnitude_goal != 0 and magnitude_current != 0:
                if dot_product / (magnitude_goal * magnitude_current) < 0:
                    continue

            # 计算拓展节点到原始起点和终点的距离和
            distance_to_start = math.hypot(new_x - self.calc_grid_position(nstart.x, self.minx), new_y - self.calc_grid_position(nstart.y, self.miny))
            distance_sum = distance_to_start + distance_to_goal
            if distance_sum >= 1.4 * initial_distance_to_goal:
                continue

            # 插值和计算成本
            u = self.interpolate_u(new_x, new_y)
            v = self.interpolate_v(new_x, new_y)
            w = self.interpolate_w(new_x, new_y)

            dx = (self.motion[i][0]) * self.resolution
            dy = (self.motion[i][1]) * self.resolution
            WindEffect = self.calculate_cost(dx, dy, u, v, w)
            
            # 累积飞行时间
            distance = math.hypot(dx, dy)
            time_elapsed = distance / self.U_t
            node.time = current.time + time_elapsed

            # 更新风场数据，使用四舍五入的时间点
            self.load_data(node.time)

            # 更新节点成本
            node.cost = current.cost + WindEffect  # 更新总成本 
            
            # 输出调试信息，包括累计飞行时间
            print(f'当前节点: x={current.x * self.resolution}, y={current.y * self.resolution}；计算成本: dx={dx}, dy={dy}, u={u}, v={v}, w={w}, cost={node.cost}, g={node.g}, h={node.h}, time={current.time}')
            print(f'新节点: x={node.x * self.resolution}, y={node.y * self.resolution}, time={node.time}')       
            n_id = self.calc_grid_index(node)

            if not self.verify_node(node):
                continue

            if n_id in closed_set:
                continue

            if n_id not in open_set:
                open_set[n_id] = node
            else:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id] = node


    def calc_final_path(self, ngoal, closedset):
        # 根据ngoal和closedset计算路径
        pathx, pathy = [self.calc_grid_position(ngoal.x, self.minx)], [self.calc_grid_position(ngoal.y, self.miny)]
        path_details = []  # 存储uvw速度值及h和g值
        parent_index = ngoal.parent_index

        # 添加调试信息，检查ngoal和closedset
        print(f"ngoal: {ngoal}")
        print(f"closedset: {closedset}")

        while parent_index != -1:
            n = closedset[parent_index]
            x = self.calc_grid_position(n.x, self.minx)
            y = self.calc_grid_position(n.y, self.miny)
            u = self.interpolate_u(x, y)
            v = self.interpolate_v(x, y)
            w = self.interpolate_w(x, y)
            # 收集uvw速度值及h和g值
            path_details.append((x, y, u, v, w, n.h, n.g))
            parent_index = n.parent_index

        # 反转路径详情，以使其从起点开始
        path_details = list(reversed(path_details))

        # 添加调试信息，检查path_details
        print(f"path_details: {path_details}")

        # 提取修剪前的路径x和y坐标
        simple_path = [(p[0], p[1]) for p in path_details]

        # 调试信息
        print(f"simple_path: {simple_path}")

        # 调用Pruning函数，使用只包含x和y坐标的路径列表
        pruned_simple_path = Pruning(simple_path, self.obmap)

        # 添加调试信息，检查Pruning结果
        print(f"pruned_simple_path: {pruned_simple_path}")

        if not pruned_simple_path:
            raise ValueError("修剪后的路径为空")

        # 创建一个字典，将简化路径中的坐标映射到uvw值及h和g值
        coord_to_details = {(p[0], p[1]): p[2:] for p in path_details}

        pruned_path_with_details = [(*p, *coord_to_details[p]) for p in pruned_simple_path if p in coord_to_details]

        # 提取修剪后的路径坐标
        pruned_pathx, pruned_pathy = zip(*[(pp[0], pp[1]) for pp in pruned_simple_path])

        # 返回修剪后的路径坐标
        return pruned_pathx, pruned_pathy

    @staticmethod  # 静态方法，calc_heuristic函数不用传入self，因为要经常修改启发函数，目的是为了提高阅读性
    def calc_heuristic(n1, n2, R=100, K=0.5):  # n1: ngoal，n2: open_set[o]
        Dx = abs(n1.x - n2.x)
        Dy = abs(n1.y - n2.y)
        D = 1  # 曼哈顿距离系数
        D2 = math.sqrt(2)  # 切比雪夫距离系数

        h = D * (Dx + Dy) - (2 - 2 * D2) * min(Dx, Dy) + D2 * min(Dx, Dy)
        
        # 动态权重系数 k(n) 的计算
        mR_ratio = n2.m / R
        if mR_ratio > K:
            k = 1 - mR_ratio
        else:
            k = 1 - K

        return (1 + k) * h
    # @staticmethod  # 
    # def calc_heuristic(n1, n2):  # n1: ngoal，n2: open_set[o]     
    #     h =  math.hypot(n1.x - n2.x, n1.y - n2.y)
    #     return h

    # 得到全局地图中的具体坐标: 传入地图中最小处障碍物的pos和index
    def calc_grid_position(self, index, minpos):   
        pos = index * self.resolution + minpos
        return pos

    # 位置转化为以栅格大小为单位的索引: 传入position,min_pos
    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.resolution)  # (当前节点-最小处的坐标)/分辨率=pos_index  round四舍五入向下取整

    # 计算栅格地图节点的index： 传入某个节点
    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)   

    # 验证是否为可通行节点
    def verify_node(self, node):
        posx = self.calc_grid_position(node.x, self.minx)
        posy = self.calc_grid_position(node.y, self.miny)

        if posx < self.minx:
            return False
        elif posy < self.miny:
            return False
        elif posx >= self.maxx:
            return False
        elif posy >= self.maxy:
            return False

        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.minx = round(min(ox))    # 地图中的临界值 -10
        self.miny = round(min(oy))    # -10
        self.maxx = round(max(ox))    # 60 
        self.maxy = round(max(oy))    # 60
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.resolution)   # 35
        self.ywidth = round((self.maxy - self.miny) / self.resolution)   # 35
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        self.obmap = [[False for i in range(int(self.ywidth))]
                       for i in range(int(self.xwidth))]
        for ix in range(int(self.xwidth)):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(int(self.ywidth)):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):  #将ox,oy打包成元组，返回列表，并遍历
                    d = math.hypot(iox - x, ioy - y)  
                    if d <= self.rr:          #代价小于车辆半径，可正常通过，不会穿越障碍物
                        self.obmap[ix][iy] = True
                        break

    def calculate_cost(self, dx, dy, u, v, w, Cd=1, S=1):
        # 常量定义
        rho = 1.225  # 空气密度 (kg/m^3)
        g = 9.81  # 重力加速度 (m/s^2)
        eta = 0.4845  # 效率
        m = 6.2  # 质量 (kg)
        U_t = 8  # 固定速度值 (m/s)
        D = math.sqrt(dx**2 + dy**2)  # 移动距离

        # 计算Pi
        P_i = (1 / eta) * (m * g**1.5) / math.sqrt(rho * S)

        # 根据移动方向和速度成分计算U和V
        U = U_t if dy == 0 else U_t * dx / D
        V = U_t if dx == 0 else U_t * dy / D
        U_tt = math.sqrt((U + u)**2 + (V + v)**2)

        # 计算风场对无人机的影响
        if U > 0 and u > 0:
            wind_effect_u = u**2 * (U - u)
        elif U > 0 and u < 0:
            wind_effect_u = u**2 * (U - u)
        elif U < 0 and u > 0:
            wind_effect_u = u**2 * (abs(U) + u)
        elif U == 0:
            wind_effect_u = abs( u**3 )
        else:  # U < 0 and u < 0
            wind_effect_u = u**2 * -(U - u)

        if V > 0 and v > 0:
            wind_effect_v = v**2 * (V - v) 
        elif V > 0 and v < 0:
            wind_effect_v = v**2 * (V - v)
        elif V < 0 and v > 0:
            wind_effect_v = v**2 * (abs(V) + v)
        elif V == 0:
            wind_effect_v = abs( v**3 )
        else:  # V < 0 and v < 0
            wind_effect_v = v**2 * -(V - v)

       # 使用给定的公式计算成本
        cost = ((0.5 * rho * Cd * S * (wind_effect_u + wind_effect_v + abs(w**3)) + P_i) + 1270 ) * D / U_tt
        # + m * g * D
        return cost
    

    def get_motion_model(self):
        # dx, dy, cost
       motion = [
            [1, 0, 0],
            [0, 1, 0],
            [0, -1, 0],[-1, 0, 0],
            [1, -1, 0],[-1, -1, 0],
            [-1, 1, 0],[1, 1, 0],
            [2, 1, 0],[2, -1, 0],
            [-2, 1, 0],[-2, -1, 0],
            [-1, 2, 0],[-1, -2, 0],
            [1, 2, 0],[1, -2, 0],
        ]
       return motion

def main():
    data_dir = r"E:\UAV\研究计划\WindFieldInfo\Instant_filed_02"
    print(__file__ + '  start!')
    plt.title("Astar_PM_V8_Instant_New_More_Nod_0.2s")
    sx, sy, gx, gy = 230,180,50,180
    grid_size, robot_radius = 1.25, 5.0
    width, height = 320, 240
    border_spacing = 10
    building_spacing = 20
    columns = 8

    # 计算起始x坐标
    ##centers = [(-20, -20)]
    centers = [(100, 20), (180, 20), (260, 20),
               (100, 100), (180, 100), (260, 100),
               (100, 180), (180, 180), (260, 180)]
    obstacle_size = 20  # 障碍物尺寸

    ox, oy = [], []
    for center in centers:
        ox_temp, oy_temp = create_square_obstacle(center, obstacle_size)
        ox.extend(ox_temp)
        oy.extend(oy_temp)

    
    # 添加外围边界
    ox.extend([0] * (height + 1))
    oy.extend(list(range(0, height + 1)))
    ox.extend([width] * (height + 1))
    oy.extend(list(range(0, height + 1)))
    ox.extend(list(range(0, width + 1)))
    oy.extend([0] * (width + 1))
    ox.extend(list(range(0, width + 1)))
    oy.extend([height] * (width + 1))

    if show_animation:
        plt.plot(ox, oy, ".k")  # 黑色. 障碍物
        plt.plot(sx, sy, "og")  # 绿色圆圈 开始坐标
        plt.plot(gx, gy, "xb")  # 蓝色x 目标点
        plt.grid(True)
        plt.axis('equal')  # 保持栅格的横纵坐标刻度一致

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, data_dir)  # 注意这里传入了data
    pathx, pathy = a_star.planning(sx, sy, gx, gy)

    print(f"Final path: {list(pathx)}, {list(pathy)}")  # 添加调试信息

    # output_filename = f'New_optimized_Vi_path_{sx}_{sy}_to_{gx}_{gy}.dat'
    output_filename = f'New_optimized_UW_PM_path_Instant_{sx}_{sy}_to_{gx}_{gy}_V8UTT_More_Mod_New_0.2.dat'
    with open(output_filename, 'w') as file:
        file.write('X,Y\n')  # 文件头
        for x, y in zip(pathx, pathy):
            file.write(f'{x},{y}\n')

    if show_animation:
        plt.xlim(0, width)
        plt.ylim(0, height)
        plt.plot(pathx, pathy, "-r")  # 红色直线 最终路径
        plt.show()
        plt.pause(0.001)   # 动态显示

if __name__ == '__main__':
    main()