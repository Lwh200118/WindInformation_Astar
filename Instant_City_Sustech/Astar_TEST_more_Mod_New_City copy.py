import os
import math
import numpy as np
import matplotlib.pyplot as plt

show_animation = True

def read_dat_file(filename):
    data = np.loadtxt(filename, skiprows=1)  # 跳过文件头
    return data

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        self.resolution = resolution   # grid resolution [m]
        self.rr = int(rr)  # robot radius [m] 转换为整数
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x    # index of grid
            self.y = y
            self.cost = cost  # g(n)
            self.parent_index = parent_index   # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny),
                           0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny),
                          0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while True:
            if not open_set:
                print("Open_set is empty...")
                break
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            if show_animation:
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny),
                         "xc")  # 青色x 搜索点
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

            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,    # 当前x+motion列表中第0个元素dx
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                n_id = self.calc_grid_index(node)   # 返回该节点位置index

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

    def calc_final_path(self, ngoal, closedset):
        pathx, pathy = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        parent_index = ngoal.parent_index
        while parent_index != -1:
            n = closedset[parent_index]
            pathx.append(self.calc_grid_position(n.x, self.minx))
            pathy.append(self.calc_grid_position(n.y, self.miny))
            parent_index = n.parent_index

        return pathx, pathy

    @staticmethod
    def calc_heuristic(n1, n2):
        h = math.hypot(n1.x - n2.x, n1.y - n2.y)
        return h

    def calc_grid_position(self, index, minpos):
        pos = index * self.resolution + minpos
        return pos

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        posx = self.calc_grid_position(node.x, self.minx)
        posy = self.calc_grid_position(node.y, self.miny)

        if posx < self.minx or posy < self.miny or posx >= self.maxx or posy >= self.maxy:
            return False

        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.resolution)
        self.ywidth = round((self.maxy - self.miny) / self.resolution)
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        self.obmap = [[False for _ in range(int(self.ywidth))]
                      for _ in range(int(self.xwidth))]
        for iox, ioy in zip(ox, oy):
            ix = self.calc_xyindex(iox, self.minx)
            iy = self.calc_xyindex(ioy, self.miny)
            for dx in range(-self.rr, self.rr):
                for dy in range(-self.rr, self.rr):
                    d = math.hypot(dx, dy)
                    if d <= self.rr:
                        x = ix + dx
                        y = iy + dy
                        if 0 <= x < self.xwidth and 0 <= y < self.ywidth:
                            self.obmap[x][y] = True

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [
            [1, 0, 1.25],
            [0, 1, 1.25],
            [-1, 0, 1.25],
            [0, -1, 1.25],
            [1, 1, 1.25 * math.sqrt(2)],
            [1, -1, 1.25 * math.sqrt(2)],
            [-1, 1, 1.25 * math.sqrt(2)],
            [-1, -1, 1.25 * math.sqrt(2)],
            [2, 1, 1.25 * math.sqrt(5)], [2, -1, 1.25 * math.sqrt(5)],
            [-2, 1, 1.25 * math.sqrt(5)], [-2, -1, 1.25 * math.sqrt(5)],
            [-1, 2, 1.25 * math.sqrt(5)], [-1, -2, 1.25 * math.sqrt(5)],
            [1, 2, 1.25 * math.sqrt(5)], [1, -2, 1.25 * math.sqrt(5)]
        ]
        return motion

def main():
    filename = r"E:\gtian\BlockMesh-LES-1600w-west\Sustech_H_60.dat"
    points = read_dat_file(filename)
    ox, oy = points[:, 0], points[:, 1]

    print(__file__ + '  start!')
    plt.title("Astar")
    sx, sy, gx, gy = 0, 200, 320, 200  # 修改为你的起点和终点
    grid_size, robot_radius = 1.25, 5.0

    if show_animation:
        plt.plot(ox, oy, ".k")  # 黑色. 障碍物
        plt.plot(sx, sy, "og")  # 绿色圆圈 开始坐标
        plt.plot(gx, gy, "xb")  # 蓝色x 目标点
        plt.grid(True)
        plt.axis('equal')  # 保持栅格的横纵坐标刻度一致
        plt.xlim([-100, 500])  # 设置x轴范围
        plt.ylim([0, 400])  # 设置y轴范围

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    pathx, pathy = a_star.planning(sx, sy, gx, gy)

    print(f"Final path: {list(pathx)}, {list(pathy)}")

    output_filename = f'Astar_path_City_{sx}_{sy}_to_{gx}_{gy}.dat'
    with open(output_filename, 'w') as file:
        file.write('X,Y\n')
        for x, y in zip(pathx, pathy):
            file.write(f'{x},{y}\n')

    if show_animation:
        plt.xlim([-100, 500])  # 设置x轴范围
        plt.ylim([0, 400])  # 设置y轴范围
        plt.plot(pathx, pathy, "-r")  # 红色直线 最终路径
        plt.show()
        plt.pause(0.001)   # 动态显示

if __name__ == '__main__':
    main()