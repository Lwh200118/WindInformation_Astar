import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from scipy.interpolate import griddata

def read_path_from_dat(file_path):
    path_x, path_y = [], []
    with open(file_path, 'r') as file:
        next(file)  # Skip header line
        for line in file:
            x, y = map(float, line.strip().split(','))
            path_x.append(x)
            path_y.append(y)
    return path_x, path_y

def read_dat_file(filename):
    """
    读取 .dat 文件并返回点的坐标
    """
    data = np.loadtxt(filename, skiprows=1)  # 跳过文件头
    return data

def read_wind_field(file_path):
    wind_data = np.loadtxt(file_path)
    X = wind_data[:, 0]
    Y = wind_data[:, 1]
    U = wind_data[:, 3]
    V = wind_data[:, 4]
    return X, Y, U, V

def get_speed_range(wind_field_folder, start_time, end_time, interval):
    min_speed, max_speed = float('inf'), float('-inf')
    current_time = start_time
    while current_time <= end_time:
        wind_file_path = os.path.join(wind_field_folder, f"{int(current_time)}", "U_0_mySurfaces_60.dat")
        if not os.path.exists(wind_file_path):
            current_time += interval
            continue
        X, Y, U, V = read_wind_field(wind_file_path)
        speed = np.sqrt(U**2 + V**2)
        min_speed = min(min_speed, np.min(speed))
        max_speed = max(max_speed, np.max(speed))
        current_time += interval
    return min_speed, max_speed

def read_energy_data(file_path):
    times, energies, distances = [], [], []
    with open(file_path, 'r') as file:
        for line in file:
            time, energy, distance = map(float, line.strip().split(','))
            times.append(time)
            energies.append(energy)
            distances.append(distance)
    return times, energies, distances

def generate_map_and_save_images(output_folder, wind_field_folder, speed, interval, drone_image_path, min_speed, max_speed, display_min_speed, display_max_speed):
    width, height = 500, 400  # 修改地图尺寸

    # 使用之前代码中的障碍物位置
    filename = r"E:\gtian\BlockMesh-LES-1600w-west\Sustech_H_60.dat"
    points = read_dat_file(filename)
    ox, oy = points[:, 0], points[:, 1]

    traditional_path_x, traditional_path_y = read_path_from_dat(r"E:\UAV\研究计划\TEST\Instant_City_Sustech\Astar_path_City_0_200_to_320_200.dat")
    improved_path_x, improved_path_y = read_path_from_dat(r"E:\UAV\研究计划\TEST\Instant_City_Sustech\New_optimized_UW_PM_path_Instant_City_0_200_to_320_200_V8UTT_More_Mod_1_0.dat")

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
    current_time = 3600
    while current_time <= 3600 + total_time + interval:  # 增加一个时间间隔
        # Find the closest index in the times array
        idx = np.searchsorted(times, current_time - 3600)

        wind_file_path = os.path.join(wind_field_folder, f"{int(current_time)}", "U_0_mySurfaces_60.dat")
        if not os.path.exists(wind_file_path):
            print(f"Wind field file for time {current_time} does not exist.")
            current_time += interval
            continue

        X, Y, U, V = read_wind_field(wind_file_path)
        step = 15

        fig, ax = plt.subplots(figsize=(10, 8))

        # Calculate wind speed magnitude
        speed = np.sqrt(U**2 + V**2)
        
        # Create a grid for contour plot
        xi = np.linspace(min(X), max(X), width)
        yi = np.linspace(min(Y), max(Y), height)
        xi, yi = np.meshgrid(xi, yi)
        speed_grid = griddata((X, Y), speed, (xi, yi), method='cubic')

        # Plot the wind speed contour
        contour = ax.contourf(xi, yi, speed_grid, cmap='jet', alpha=0.5, levels=np.linspace(display_min_speed, display_max_speed, 100))
        fig.colorbar(contour, ax=ax, label='Wind Speed (m/s)')  # Add colorbar

        ax.plot(ox, oy, ".k")  # Plot obstacles

        ax.plot(traditional_path_x, traditional_path_y, '-g', label='Traditional A* Path')
        ax.plot(improved_path_x[:idx+1], improved_path_y[:idx+1], '-r', label='Already Flown Path (Improved)')
        ax.plot(improved_path_x[idx:], improved_path_y[idx:], '--b', label='Future Path (Improved)')

        ax.quiver(X[::step], Y[::step], U[::step], V[::step], scale=200, color='black', alpha=0.6)

        # Add the drone image at the current position
        imagebox = OffsetImage(drone_image, zoom=0.03)  # Adjust zoom to fit the size
        ab = AnnotationBbox(imagebox, (improved_path_x[idx], improved_path_y[idx]), frameon=False)
        ax.add_artist(ab)

        ax.grid(True)
        ax.axis('equal')
        ax.set_xlim(-100, 500)  # 修改x轴范围
        ax.set_ylim(0, 400)  # 修改y轴范围
        ax.set_title(f"Map with Traditional and Improved A* Paths and Wind Field_1.0s (Time: {current_time:.1f}s)")
        ax.legend()

        plt.savefig(os.path.join(output_folder, f"More_F0200F320200_New_City_10_{current_time - 3600:.1f}.png"))  # 使用浮点数格式化
        plt.close()

        current_time += interval

def generate_energy_plot(output_folder, traditional_energy_data_file, improved_energy_data_file):
    # Read energy data
    traditional_energy_times, traditional_energies, traditional_distances = read_energy_data(traditional_energy_data_file)
    improved_energy_times, improved_energies, improved_distances = read_energy_data(improved_energy_data_file)

    # Generate energy consumption plot
    plt.figure(figsize=(10, 8))
    plt.plot(traditional_distances, traditional_energies, '-o', label='Traditional A* Path Energy Consumption')
    plt.plot(improved_distances, improved_energies, '-x', label='Improved A* Path Energy Consumption')
    plt.xlabel('Distance (m)')
    plt.ylabel('Total Energy Consumption (J)')
    plt.title('Energy Consumption vs Distance')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_folder, "Energy_Consumption_Comparison.png"))
    plt.close()

if __name__ == '__main__':
    output_folder = 'output_images'
    wind_field_folder = r"E:\gtian\BlockMesh-LES-1600w-west\postProcessing\surfaces"
    speed = 8  # Example speed in m/s
    interval = 1.0  # Example interval in seconds
    drone_image_path = r"E:\UAV\研究计划\TEST\output_images\UAV.png"  # Path to the drone image

    # Calculate the speed range
    start_time = 3600
    end_time = 3650  # Example end time
    min_speed, max_speed = get_speed_range(wind_field_folder, start_time, end_time, interval)

    # Set display speed range
    display_min_speed = 0  # Example minimum display speed
    display_max_speed = 7  # Example maximum display speed

    # Paths to the energy data files
    traditional_energy_data_file = r"E:\UAV\研究计划\TEST\Instant_City_Sustech\energy_consumption_Astar_City_F0200F320200_New_1.0.dat"
    improved_energy_data_file = r"E:\UAV\研究计划\TEST\Instant_City_Sustech\energy_consumption_Op_City_F0200F320200_New_1.0.dat"

    generate_map_and_save_images(output_folder, wind_field_folder, speed, interval, drone_image_path, min_speed, max_speed, display_min_speed, display_max_speed)
    generate_energy_plot(output_folder, traditional_energy_data_file, improved_energy_data_file)