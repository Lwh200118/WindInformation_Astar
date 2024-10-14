import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

def create_square_obstacle(center, size):
    ox, oy = [], []
    half_size = size // 2
    left_bottom_x = center[0] - half_size
    left_bottom_y = center[1] - half_size

    for i in range(size + 1):
        ox.append(left_bottom_x + i)
        oy.append(left_bottom_y)
        ox.append(left_bottom_x + i)
        oy.append(left_bottom_y + size)
        ox.append(left_bottom_x)
        oy.append(left_bottom_y + i)
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

def generate_map_and_save_images(output_folder, wind_field_folder, speed, interval, drone_image_path):
    width, height = 320, 240
    obstacle_size = 20

    centers = [(100, 20), (180, 20), (260, 20),
               (100, 100), (180, 100), (260, 100),
               (100, 180), (180, 180), (260, 180)]

    ox, oy = [], []
    for center in centers:
        ox_temp, oy_temp = create_square_obstacle(center, obstacle_size)
        ox.extend(ox_temp)
        oy.extend(oy_temp)

    # Add boundary obstacles
    ox.extend([0] * (height + 1))
    oy.extend(list(range(0, height + 1)))
    ox.extend([width] * (height + 1))
    oy.extend(list(range(0, height + 1)))
    ox.extend(list(range(0, width + 1)))
    oy.extend([0] * (width + 1))
    ox.extend(list(range(0, width + 1)))
    oy.extend([height] * (width + 1))

    traditional_path_x, traditional_path_y = read_path_from_dat(r"E:\UAV\研究计划\TEST\Astar_path_D_direct_50_100_to_230_100_R5_Mod.dat")
    improved_path_x, improved_path_y = read_path_from_dat(r"E:\UAV\研究计划\TEST\V8_Instant\New_optimized_UW_PM_path_Instant_50_100_to_230_100_V8UTT_More_Mod_0_2.dat")

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Calculate distances and times for improved path
    distances = np.sqrt(np.diff(improved_path_x)**2 + np.diff(improved_path_y)**2)
    times = np.cumsum(distances / speed)
    times = np.insert(times, 0, 0)  # Insert the starting point time

    # Load the drone image
    drone_image = plt.imread(drone_image_path)

    # Generate images at each interval
    total_time = times[-1]
    current_time = 5100.0
    while current_time <= 5100.0 + total_time + interval:  # 增加一个时间间隔
        # Find the closest index in the times array
        idx = np.searchsorted(times, current_time - 5100.0)

        wind_file_path = os.path.join(wind_field_folder, f"{current_time:.1f}", "U_0_mySurfaces_60.dat")
        if not os.path.exists(wind_file_path):
            print(f"Wind field file for time {current_time:.1f} does not exist.")
            current_time += interval
            continue

        X, Y, U, V = read_wind_field(wind_file_path)
        step = 50

        plt.figure(figsize=(10, 8))

        # Calculate wind speed magnitude
        speed = np.sqrt(U**2 + V**2)
        plt.contourf(X.reshape((height, width)), Y.reshape((height, width)), speed.reshape((height, width)), cmap='jet', alpha=0.5)

        plt.plot(ox, oy, ".k")  # Plot obstacles

        for center in centers:
            left_center = (center[0] - 50, center[1])
            right_center = (center[0] + 50, center[1])
            plt.plot(*left_center, 'bo')  # Mark with blue dot
            plt.plot(*right_center, 'ro')  # Mark with red dot
            plt.text(*left_center, f"{left_center}", fontsize=8)
            plt.text(*right_center, f"{right_center}", fontsize=8)

        plt.plot(traditional_path_x, traditional_path_y, '-g', label='Traditional A* Path')
        plt.plot(improved_path_x[:idx+1], improved_path_y[:idx+1], '-r', label='Already Flown Path (Improved)')
        plt.plot(improved_path_x[idx:], improved_path_y[idx:], '--b', label='Future Path (Improved)')

        plt.quiver(X[::step], Y[::step], U[::step], V[::step], scale=200, color='black', alpha=0.6)

        # Add the drone image at the current position
        imagebox = OffsetImage(drone_image, zoom=0.03)  # Adjust zoom to fit the size
        ab = AnnotationBbox(imagebox, (improved_path_x[idx], improved_path_y[idx]), frameon=False)
        plt.gca().add_artist(ab)

        plt.grid(True)
        plt.axis('equal')
        plt.xlim(0, width)
        plt.ylim(0, height)
        plt.title(f"Map with Traditional and Improved A* Paths and Wind Field_1.0s (Time: {current_time:.1f}s)")
        plt.legend()
        plt.savefig(os.path.join(output_folder, f"More_F_10_{current_time - 5100.0:.1f}.png"))  # 使用浮点数格式化
        plt.close()

        current_time += interval

if __name__ == '__main__':
    output_folder = 'output_images'
    wind_field_folder = r"E:\UAV\研究计划\WindFieldInfo\Instant_filed_02"
    speed = 8  # Example speed in m/s
    interval = 1.0  # Example interval in seconds
    drone_image_path = r"E:\UAV\研究计划\TEST\output_images\UAV.png"  # Path to the drone image
    generate_map_and_save_images(output_folder, wind_field_folder, speed, interval, drone_image_path)