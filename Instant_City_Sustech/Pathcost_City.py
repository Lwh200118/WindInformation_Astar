import math
import pandas as pd
import numpy as np
from scipy.interpolate import LinearNDInterpolator
import os

# 读取风场数据
def read_data(filename, minx=-100, maxx=500, miny=0, maxy=400):
    data = pd.read_csv(filename, delim_whitespace=True, names=['x', 'y', 'z', 'u', 'v', 'w'])
    # 裁剪数据，使其范围限定在展示范围内
    data = data[(data['x'] >= minx) & (data['x'] <= maxx) & (data['y'] >= miny) & (data['y'] <= maxy)]
    return data

# 插值函数
def interpolate_u(data, x, y):
    points = data[['x', 'y']].to_numpy()
    u_values = data['u'].to_numpy()
    interpolator = LinearNDInterpolator(points, u_values)
    u_value_at_coord = interpolator([x, y])
    return 0 if np.isnan(u_value_at_coord[0]) else u_value_at_coord[0]

def interpolate_v(data, x, y):
    points = data[['x', 'y']].to_numpy()
    v_values = data['v'].to_numpy()
    interpolator = LinearNDInterpolator(points, v_values)
    v_value_at_coord = interpolator([x, y])
    return 0 if np.isnan(v_value_at_coord[0]) else v_value_at_coord[0]

def interpolate_w(data, x, y):
    points = data[['x', 'y']].to_numpy()
    w_values = data['w'].to_numpy()
    interpolator = LinearNDInterpolator(points, w_values)
    w_value_at_coord = interpolator([x, y])
    return 0 if np.isnan(w_value_at_coord[0]) else w_value_at_coord[0]

# 成本计算函数
def calculate_cost(dx, dy, u, v, w, Cd=1, S=1):
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
    else:  # U < 0 and u < 0
        wind_effect_u = u**2 * -(U - u)

    if V > 0 and v > 0:
        wind_effect_v = v**2 * (V - v) 
    elif V > 0 and v < 0:
        wind_effect_v = v**2 * (V - v)
    elif V < 0 and v > 0:
        wind_effect_v = v**2 * (abs(V) + v)
    else:  # V < 0 and v < 0
        wind_effect_v = v**2 * -(V - v)

    # 使用给定的公式计算成本
    cost = ((0.5 * rho * Cd * S * (wind_effect_u + wind_effect_v + abs(w**3)) + P_i) + 1270 ) * D / U_tt
    return cost

# 读取路径
def read_path_from_file(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()[1:]
        path = [(float(line.split(',')[0]), float(line.split(',')[1])) for line in lines]
    return path

# 加载风场数据
def load_wind_data(data_dir, time):
    # 将时间戳四舍五入到最近的0.2秒间隔
    rounded_time = round(time)
    filename = os.path.join(data_dir, f"{rounded_time}", "U_0_mySurfaces_60.dat")
    if os.path.exists(filename):
        return read_data(filename)
    else:
        raise FileNotFoundError(f"文件 {filename} 不存在")

# 计算总成本并输出路径信息
def calculate_total_cost_and_output(path, data_dir, output_filename, energy_output_filename, start_time=3600, time_interval=1.0, U_t=8):
    total_cost = 0
    total_distance = 0
    current_time = start_time
    next_output_time = current_time + time_interval
    energy_data = []

    with open(output_filename, 'w') as file:
        file.write("x,y,u,v,w,current_cost,total_cost\n")
        for i in range(len(path)-1):
            x1, y1 = path[i]
            x2, y2 = path[i+1]
            dx = x2 - x1
            dy = y2 - y1
            distance = math.sqrt(dx**2 + dy**2)
            total_distance += distance
            time_elapsed = distance / U_t
            current_time += time_elapsed

            # 加载当前时间点的风场数据
            wind_data = load_wind_data(data_dir, current_time)

            u = interpolate_u(wind_data, x2, y2)
            v = interpolate_v(wind_data, x2, y2)
            w = interpolate_w(wind_data, x2, y2)
            current_cost = calculate_cost(dx, dy, u, v, w)
            total_cost += current_cost

            file.write(f"{x2},{y2},{u},{v},{w},{current_cost},{total_cost}\n")

            # 每过一个时间间隔记录一次能耗总值和飞行距离
            if current_time >= next_output_time:
                energy_data.append((next_output_time, total_cost, total_distance))
                next_output_time += time_interval

    # 写入能耗总值和飞行距离到.dat文件
    with open(energy_output_filename, 'w') as energy_file:
        for time, energy, distance in energy_data:
            energy_file.write(f"{time},{energy},{distance}\n")

    return total_cost

# 主程序
def main():
    data_dir = r"E:\gtian\BlockMesh-LES-1600w-west\postProcessing\surfaces"
    path_filename = r"E:\UAV\研究计划\TEST\Instant_City_Sustech\Astar_path_City_0_200_to_320_200.dat"
    output_filename = 'path_cost_output_PM_60_80_to_160_80.csv'
    energy_output_filename = 'energy_consumption_Astar_City_F0200F320200_New_1.0.dat'
    path = read_path_from_file(path_filename)
    
    total_cost = calculate_total_cost_and_output(path, data_dir, output_filename, energy_output_filename)
    print(f"Total path cost: {total_cost}")

if __name__ == '__main__':
    main()