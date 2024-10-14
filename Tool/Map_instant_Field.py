import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from scipy.interpolate import griddata

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
    Z = wind_data[:, 2]
    U = wind_data[:, 3]
    V = wind_data[:, 4]
    return X, Z, U, V

def get_speed_range(wind_field_folder, start_time, end_time, interval):
    min_speed, max_speed = float('inf'), float('-inf')
    current_time = start_time
    while current_time <= end_time:
        wind_file_path = os.path.join(wind_field_folder, f"{current_time:.1f}", "U_0_mySurfaces_60.dat")
        if not os.path.exists(wind_file_path):
            current_time += interval
            continue
        X, Z, U, V = read_wind_field(wind_file_path)
        speed = np.sqrt(U**2 + V**2)
        min_speed = min(min_speed, np.min(speed))
        max_speed = max(max_speed, np.max(speed))
        current_time += interval
    return min_speed, max_speed

def generate_map_and_save_images(output_folder, wind_field_folder, speed, interval, drone_image_path, min_speed, max_speed):
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

    traditional_path_x, traditional_path_y = read_path_from_dat(r"E:\UAV\研究计划\TEST\Astar_path_D_direct_130_20_to_230_100_R5.dat")
    improved_path_x, improved_path_y = read_path_from_dat(r"E:\UAV\研究计划\TEST\V8_Instant\New_optimized_UW_PM_path_Instant_130_20_to_230_100_V8UTT_More_Mod_1_0.dat")

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

        X, Z, U, V = read_wind_field(wind_file_path)
        step = 50

        # Debugging: print the shapes of the arrays
        print(f"Shapes - X: {X.shape}, Z: {Z.shape}, U: {U.shape}, V: {V.shape}")

        plt.figure(figsize=(10, 8))

        # Calculate wind speed magnitude
        speed = np.sqrt(U**2 + V**2)
        
        # Create a grid for contour plot
        xi = np.linspace(min(X), max(X), width)
        zi = np.linspace(min(Z), max(Z), height)
        xi, zi = np.meshgrid(xi, zi)
        speed_grid = griddata((X, Z), speed, (xi, zi), method='cubic')

        # Plot the wind speed contour
        contour = plt.contourf(xi, zi, speed_grid, cmap='jet', alpha=0.5, levels=np.linspace(min_speed, max_speed, 100))
        plt.colorbar(contour, label='Wind Speed (m/s)')  # Add colorbar

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

        plt.quiver(X[::step], Z[::step], U[::step], V[::step], scale=200, color='black', alpha=0.6)

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
        plt.savefig(os.path.join(output_folder, f"More_F13020F230100_10_{current_time - 5100.0:.1f}.png"))  # 使用浮点数格式化
        plt.close()

        current_time += interval

if __name__ == '__main__':
    output_folder = 'output_images'
    wind_field_folder = r"E:\UAV\研究计划\WindFieldInfo\Instant_filed_02"
    speed = 8  # Example speed in m/s
    interval = 1.0  # Example interval in seconds
    drone_image_path = r"E:\UAV\研究计划\TEST\output_images\UAV.png"  # Path to the drone image

    # Calculate the speed range
    start_time = 5100.0
    end_time = 5100.0 + 100  # Example end time
    min_speed, max_speed = get_speed_range(wind_field_folder, start_time, end_time, interval)

    generate_map_and_save_images(output_folder, wind_field_folder, speed, interval, drone_image_path, min_speed, max_speed)