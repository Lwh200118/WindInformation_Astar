import math
import pandas as pd
import numpy as np
from scipy.interpolate import LinearNDInterpolator

# 读取风场数据
def read_data(filename):
    data = pd.read_csv(filename, delim_whitespace=True, 
                       names=['x', 'z', 'y', 'u', 'w', 'v'])
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
    U_t = 16  # 固定速度值 (m/s)
    D = math.sqrt(dx**2 + dy**2)  # 移动距离

    # 计算Pi
    P_i = (1 / eta) * (m * g**1.5) / math.sqrt(rho * S)

    # 根据移动方向和速度成分计算U和V
    U = U_t if dy == 0 else U_t * dx / D
    V = U_t if dx == 0 else U_t * dy / D
    # U = U_t if dx == 0 else U_t * dy / D
    # V = U_t if dy == 0 else U_t * dx / D
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

# 计算总成本并输出路径信息
def calculate_total_cost_and_output(path, data, output_filename):
    total_cost = 0
    with open(output_filename, 'w') as file:
        file.write("x,y,u,v,w,current_cost,total_cost\n")
        for i in range(len(path)-1):
            x1, y1 = path[i]
            x2, y2 = path[i+1]
            dx = x2 - x1
            dy = y2 - y1
            u = interpolate_u(data, x2, y2)
            v = interpolate_v(data, x2, y2)
            w = interpolate_w(data, x2, y2)
            current_cost = calculate_cost(dx, dy, u, v, w)
            total_cost += current_cost
            file.write(f"{x2},{y2},{u},{v},{w},{current_cost},{total_cost}\n")
    return total_cost

# 主程序
def main():
    wind_filename = r"E:\UAV\研究计划\WindFieldInfo\UMean_H75_h60_V8.dat"
    path_filename = r"E:\UAV\研究计划\TEST\New_optimized_UW_PMU35_path_50_20_to_230_180_V16UTT.dat"
    output_filename = 'path_cost_output_PM_60_80_to_160_80.csv'
    wind_data = read_data(wind_filename)
    path = read_path_from_file(path_filename)
    
    total_cost = calculate_total_cost_and_output(path, wind_data, output_filename)
    print(f"Total path cost: {total_cost}")

if __name__ == '__main__':
    main()