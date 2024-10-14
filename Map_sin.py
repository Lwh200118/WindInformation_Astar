import matplotlib.pyplot as plt
import numpy as np

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

def generate_map_and_mark_centers():
    width, height = 300, 160
    obstacle_size = 10

    centers = [(100, 80)]

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
        left_center = (center[0] - 40, center[1])
        right_center = (center[0] + 60, center[1])
        plt.plot(*left_center, 'bo')  # Mark with blue dot
        plt.plot(*right_center, 'ro')  # Mark with red dot
        plt.text(*left_center, f"{left_center}", fontsize=8)
        plt.text(*right_center, f"{right_center}", fontsize=8)

    # Read and plot traditional A* path from DAT file
    path_x, path_y = read_path_from_dat('Astar_path_D_direct_60_80_to_160_80_R5.dat')
    plt.plot(path_x, path_y, '-r', label='Traditional A* Path')

    # Read and plot improved A* path from DAT file
    path_x, path_y = read_path_from_dat('New_optimized_UW_PM_path_60_80_to_160_80_V8UTT.dat')
    plt.plot(path_x, path_y, '-g', label='Improved A* Path')

    # Read and plot wind field from file
    X, Y, U, V = read_wind_field(r"E:\UAV\研究计划\WindFieldInfo\U10_H30.dat")

    # Reduce arrow density by selecting every nth point
    step = 50
    plt.quiver(X[::step], Y[::step], U[::step], V[::step], scale=350, color='black', alpha=0.6)

    plt.grid(True)
    plt.axis('equal')
    plt.xlim(0, width)
    plt.ylim(0, height)
    plt.title("Map with Traditional and Improved A* Paths and Wind Field")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    generate_map_and_mark_centers()