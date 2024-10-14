import math
import numpy as np
import matplotlib.pyplot as plt

show_animation = True

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

    def __init__(self, ox, oy, resolution, rr):

        self.resolution = resolution   # grid resolution [m]
        self.rr = rr       # robot radius [m]
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()


    # 创建一个节点类，节点的信息包括：xy坐标，cost代价,parent_index
    class Node:
        def __init__(self, x, y , cost, parent_index):
            self.x = x    # index of grid
            self.y = y
            self.cost = cost  # g(n)
            self.parent_index = parent_index   # index of previous Node

        def __str__(self):    
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)
 

    # 经过planning函数处理，传入sx,sy,gx,gy, 返回pathx,pathy(最终的路径)
    def planning(self, sx, sy, gx, gy):

        # 1、sx = nstart  sy = ngoal  初始化nstart、ngoal坐标作为一个节点，传入节点全部信息
        nstart = self.Node(self.calc_xyindex(sx, self.minx),  # position min_pos   2 (2.5)
                           self.calc_xyindex(sy, self.miny),  # 2 (2.5)
                           0.0,
                           -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                           self.calc_xyindex(gy, self.miny),
                           0.0,
                           -1)
        # 2、open表、closed表设定为字典
        # 3、起点加入open表 
        open_set, closed_set = dict(), dict()   # key - value: hash表
        open_set[self.calc_grid_index(nstart)] = nstart   

        while 1:
            if len(open_set) == 0:
                print("Open_set is empty...")
                break  
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]  

            # 将当前节点显示出来
            if show_animation:
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny),
                         "xc")   # 青色x 搜索点
                # 按esc退出
                plt.gcf().canvas.mpl_connect('key_release_event',
                                              lambda event: [exit(0) if event.key == 'escape' else None]
                                            )
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal!")
                ngoal.parent_index = current.parent_index
                print("ngoal_parent_index:",ngoal.parent_index)
                ngoal.cost = current.cost
                print("ngoal_cost:",ngoal.cost)
                break

            # 删除open表中的c_id的子节点,并把current添加到closed_set
            del open_set[c_id]
            closed_set[c_id] = current

            # 基于motion model做栅格扩展，也就是搜索方式，可进行改进，如使用双向搜索、JPS等
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,    # 当前x+motion列表中第0个元素dx
                                 current.y + move_y,
                                 current.cost + move_cost,c_id)
                n_id = self.calc_grid_index(node)   # 返回该节点位置index

                # 如果节点不可通过，跳过
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node   # 直接加入a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node    # This path is the best until now. record it

        pathx, pathy = self.calc_final_path(ngoal, closed_set) 
        
        return pathx, pathy

    
    def calc_final_path(self, ngoal, closedset):    # 传入目标点和closed表，经过函数处理得到最终所有的xy列表
        pathx, pathy = [self.calc_grid_position(ngoal.x, self.minx)], [
                        self.calc_grid_position(ngoal.y, self.miny)]
        parent_index = ngoal.parent_index
        while parent_index != -1:
            n = closedset[parent_index]
            pathx.append(self.calc_grid_position(n.x, self.minx))
            pathy.append(self.calc_grid_position(n.y, self.miny))
            parent_index = n.parent_index

        return pathx, pathy


                

    @staticmethod  # 
    def calc_heuristic(n1, n2):  # n1: ngoal，n2: open_set[o]     
        h =  math.hypot(n1.x - n2.x, n1.y - n2.y)
        return h


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



    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        ##motion = [
                   ## [1, 1, 1.25*math.sqrt(2)],
               ## ]
        motion = [
                   [1, 0, 1.25],
                   [0, 1, 1.25],
                   [-1, 0, 1.25],
                   [0, -1, 1.25],
                   [1, 1, 1.25*math.sqrt(2)],
                   [1, -1, 1.25*math.sqrt(2)],
                   [-1, 1, 1.25*math.sqrt(2)],
                   [-1, -1, 1.25*math.sqrt(2)],
                   [2, 1, 1.25*math.sqrt(5)],[2, -1, 1.25*math.sqrt(5)],
                   [-2, 1, 1.25*math.sqrt(5)],[-2, -1, 1.25*math.sqrt(5)],
                   [-1, 2, 1.25*math.sqrt(5)],[-1, -2, 1.25*math.sqrt(5)],
                   [1, 2, 1.25*math.sqrt(5)],[1, -2, 1.25*math.sqrt(5)]    
                 ]
        
        return motion




def main():
    print(__file__ + '  start!')
    plt.title("Astar")
    # start and goal position  [m]
    sx, sy, gx, gy = 230,180,50,180
    grid_size, robot_radius = 1.25, 5.0
    width, height = 320, 240
    obstacle_size = 10
    border_spacing = 10
    building_spacing = 20
    columns = 8

    # 计算起始x坐标
    # centers = [(-20, -20)]
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
        plt.plot(ox,oy,".k") # 黑色.       障碍物
        plt.plot(sx,sy,"og") # 绿色圆圈    开始坐标
        plt.plot(gx,gy,"xb") # 蓝色x       目标点
        plt.grid(True)
        plt.axis('equal')  # 保持栅格的横纵坐标刻度一致


    a_star = AStarPlanner(ox, oy, grid_size ,robot_radius)  # grid_size=resolution 初始化中传入的参数
    pathx, pathy = a_star.planning(sx, sy, gx, gy)  # 开始与结束的坐标传入函数进行处理后，得到pathx,pathy：最终规划出的路径坐标

    # 保存路径坐标到文件
    output_filename = f'Astar_path_D_direct_{sx}_{sy}_to_{gx}_{gy}_R5.dat'
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