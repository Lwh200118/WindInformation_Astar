import matplotlib.pyplot as plt
import numpy as np
import os

def create_square_obstacle(center, size):
    ox, oy = [], []
    half_size = size // 2
    left_bottom_x = center[0] - half_size
    left_bottom_y = center[1] - half_size

    ox.append(left_bottom_x)
    oy.append(left_bottom_y)
    ox.append(left_bottom_x + size)
    oy.append(left_bottom_y)
    ox.append(left_bottom_x)
    oy.append(left_bottom_y + size)
    ox.append(left_bottom_x + size)
    oy.append(left_bottom_y + size)

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

def read_path_from_dat(file_path):
    path_x, path_y = [], []
    with open(file_path, 'r') as file:
        next(file)  # Skip header line
        for line in file:
            x, y = map(float, line.strip().split(','))
            path_x.append(x)
            path_y.append(y)
    return path_x, path_y

def read_wind_field(file_path):
    wind_data = np.loadtxt(file_path)
    X = wind_data[:, 0]
    Y = wind_data[:, 2]
    U = wind_data[:, 3]
    V = wind_data[:, 5]
    return X, Y, U, V

def load_wind_data(data_dir, time):
    filename = os.path.join(data_dir, f"{time}", "U_0_mySurfaces_60.dat")
    if os.path.exists(filename):
        return read_wind_field(filename)
    else:
        raise FileNotFoundError(f"文件 {filename} 不存在")

def generate_map_and_mark_centers():
    width, height = 320, 240
    obstacle_size = 20
    U_t = 8  # 固定速度值 (m/s)
    start_time = 5100  # 起始时间
    time_interval = 0.2  # 时间间隔

    centers = [(100, 20), (180, 20), (260, 20),
               (100, 100), (180, 100), (260, 100),
               (100, 180), (180, 180), (260, 180)]

    ox, oy = [], []
    for center in centers:
        ox_temp, oy_temp = create_square_obstacle(center, obstacle_size)
        ox.extend(ox_temp)
        oy.extend(oy_temp)

    ox.extend([0] * (height + 1))
    oy.extend(list(range(0, height + 1)))
    ox.extend([width] * (height + 1))
    oy.extend(list(range(0, height + 1)))
    ox.extend(list(range(0, width + 1)))
    oy.extend([0] * (width + 1))
    ox.extend(list(range(0, width + 1)))
    oy.extend([height] * (width + 1))

    plt.figure(figsize=(10, 8))
    plt.plot(ox, oy, ".k")  # Plot obstacles

    # Mark and label left and right centers 10m from each center
    for center in centers:
        left_center = (center[0] - 50, center[1])
        right_center = (center[0] + 50, center[1])
        plt.plot(*left_center, 'bo')  # Mark with blue dot
        plt.plot(*right_center, 'ro')  # Mark with red dot
        plt.text(*left_center, f"{left_center}", fontsize=8)
        plt.text(*right_center, f"{right_center}", fontsize=8)

    # Read and plot traditional A* paths from DAT files
    path_x, path_y = read_path_from_dat(r"E:\UAV\研究计划\TEST\Astar_path_D_direct_50_100_to_230_100_R5.dat")
    plt.plot(path_x, path_y, '-g', label='Traditional A* Path')

    # Read and plot improved A* paths from DAT files
    path_x, path_y = read_path_from_dat(r"E:\UAV\研究计划\TEST\V8_Instant\New_optimized_UW_PM_path_Instant_50_100_to_230_100_V8UTT_More_Mod_0_2.dat")
    plt.plot(path_x, path_y, '-r', label='Improved A* U=8m/s')

    data_dir = r"E:\UAV\研究计划\WindFieldInfo\Instant_filed_02"

    # 动态加载并绘制风场数据
    for i in range(len(path_x)):
        if i == 0:
            continue
        x1, y1 = path_x[i-1], path_y[i-1]
        x2, y2 = path_x[i], path_y[i]
        dx = x2 - x1
        dy = y2 - y1
        distance = np.sqrt(dx**2 + dy**2)
        time_elapsed = distance / U_t
        current_time = start_time + int(time_elapsed / time_interval) * time_interval

        try:
            X, Y, U, V = load_wind_data(data_dir, current_time)
            step = 50
            plt.quiver(X[::step], Y[::step], U[::step], V[::step], scale=200, color='black', alpha=0.6)
        except FileNotFoundError as e:
            print(e)

    plt.grid(True)
    plt.axis('equal')
    plt.xlim(0, width)
    plt.ylim(0, height)
    plt.title("Map with Traditional and Improved A* Paths and Wind Field")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    generate_map_and_mark_centers()