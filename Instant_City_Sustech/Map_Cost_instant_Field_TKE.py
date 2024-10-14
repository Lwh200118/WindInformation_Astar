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
    W = wind_data[:, 5]  
    return X, Y, U, V, W

def compute_mean_velocity(wind_field_folder, start_time, end_time, interval):
    all_U, all_V, all_W = [], [], []
    current_time = start_time
    while current_time <= end_time:
        wind_file_path = os.path.join(wind_field_folder, f"{int(current_time)}", "U_0_mySurfaces_60.dat")
        if not os.path.exists(wind_file_path):
            current_time += interval
            continue
        X, Y, U, V, W = read_wind_field(wind_file_path)
        all_U.append(U)
        all_V.append(V)
        all_W.append(W)
        current_time += interval
    mean_U = np.mean(all_U, axis=0)
    mean_V = np.mean(all_V, axis=0)
    mean_W = np.mean(all_W, axis=0)
    return mean_U, mean_V, mean_W

def get_tke_for_display(wind_field_folder, start_time, end_time, interval, mean_U, mean_V, mean_W, xlim, ylim):
    min_tke, max_tke = float('inf'), float('-inf')
    current_time = start_time
    while current_time <= end_time:
        wind_file_path = os.path.join(wind_field_folder, f"{int(current_time)}", "U_0_mySurfaces_60.dat")
        if not os.path.exists(wind_file_path):
            current_time += interval
            continue
        X, Y, U, V, W = read_wind_field(wind_file_path)
        u_prime = U - mean_U
        v_prime = V - mean_V
        w_prime = W - mean_W
        tke = 0.5 * (u_prime**2 + v_prime**2 + w_prime**2)
        
        mask = (X >= xlim[0]) & (X <= xlim[1]) & (Y >= ylim[0]) & (Y <= ylim[1])
        local_tke = tke[mask]
        
        if local_tke.size > 0:
            min_tke = min(min_tke, np.min(local_tke))
            max_tke = max(max_tke, np.max(local_tke))
        
        current_time += interval
    return min_tke, max_tke

def read_energy_data(file_path):
    times, energies, distances = [], [], []
    with open(file_path, 'r') as file:
        for line in file:
            time, energy, distance = map(float, line.strip().split(','))
            times.append(time)
            energies.append(energy)
            distances.append(distance)
    return times, energies, distances

def generate_map_and_save_images(output_folder, wind_field_folder, speed, interval, drone_image_path, min_tke, max_tke, mean_U, mean_V, mean_W, xlim, ylim, display_min_tke, display_max_tke):
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

        X, Y, U, V, W = read_wind_field(wind_file_path)
        step = 15

        fig, ax = plt.subplots(figsize=(10, 8))

        # Calculate turbulent kinetic energy (TKE)
        u_prime = U - mean_U
        v_prime = V - mean_V
        w_prime = W - mean_W
        tke = 0.5 * (u_prime**2 + v_prime**2 + w_prime**2)
        
        # Create a grid for contour plot
        xi = np.linspace(xlim[0], xlim[1], width)
        yi = np.linspace(ylim[0], ylim[1], height)
        xi, yi = np.meshgrid(xi, yi)
        tke_grid = griddata((X, Y), tke, (xi, yi), method='cubic')

        # Plot the TKE contour as an image
        im = ax.imshow(tke_grid, extent=(xlim[0], xlim[1], ylim[0], ylim[1]), origin='lower', cmap='jet', alpha=0.5, vmin=display_min_tke, vmax=display_max_tke)
        fig.colorbar(im, ax=ax, label='Turbulent Kinetic Energy (TKE)')  # Add colorbar

        ax.plot(ox, oy, ".k")  # Plot obstacles

        ax.plot(traditional_path_x, traditional_path_y, '-g', label='Traditional A* Path')
        ax.plot(improved_path_x[:idx+1], improved_path_y[:idx+1], '-r', label='Already Flown Path (Improved)')
        ax.plot(improved_path_x[idx:], improved_path_y[idx:], '--b', label='Future Path (Improved)')

        # Add the drone image at the current position
        imagebox = OffsetImage(drone_image, zoom=0.03)  # Adjust zoom to fit the size
        ab = AnnotationBbox(imagebox, (improved_path_x[idx], improved_path_y[idx]), frameon=False)
        ax.add_artist(ab)

        ax.grid(True)
        ax.axis('equal')
        ax.set_xlim(xlim)  # 修改x轴范围
        ax.set_ylim(ylim)  # 修改y轴范围
        ax.set_title(f"Map with Traditional and Improved A* Paths and TKE (Time: {current_time:.1f}s)")
        ax.legend()

        plt.savefig(os.path.join(output_folder, f"More_F0200F320200_New_City_TKE_10_{current_time - 3600:.1f}.png"))  # 使用浮点数格式化
        plt.close()

        current_time += interval

def generate_energy_plot(output_folder, traditional_energy_data_file, improved_energy_data_file):
    # Read energy data
    traditional_energy_times, traditional_energies, traditional_distances = read_energy_data(traditional_energy_data_file)
    improved_energy_times, improved_nergies, improved_distances = read_energy_data(improved_energy_data_file)

    # Generate energy consumption plot
    plt.figure(figsize=(10, 8))
    plt.plot(traditional_distances, traditional_energies, '-o', label='Traditional A* Path Energy Consumption')
    plt.plot(improved_distances, improved_nergies, '-x', label='Improved A* Path Energy Consumption')
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

    # Calculate the mean velocity
    start_time = 3600
    end_time = 3650  # Example end time
    mean_U, mean_V, mean_W = compute_mean_velocity(wind_field_folder, start_time, end_time, interval)

    # Define the display range
    xlim = (-100, 500)
    ylim = (-100, 500)

    # Calculate the TKE range for display
    min_tke, max_tke = get_tke_for_display(wind_field_folder, start_time, end_time, interval, mean_U, mean_V, mean_W, xlim, ylim)

    # Set display TKE range
    display_min_tke = 0  # Example minimum display TKE
    display_max_tke = 1  # Example maximum display TKE

    # Paths to the energy data files
    traditional_energy_data_file = r"E:\UAV\研究计划\TEST\Instant_City_Sustech\energy_consumption_Astar_City_F0200F320200_New_1.0.dat"
    improved_energy_data_file = r"E:\UAV\研究计划\TEST\Instant_City_Sustech\energy_consumption_Op_City_F0200F320200_New_1.0.dat"

    generate_map_and_save_images(output_folder, wind_field_folder, speed, interval, drone_image_path, min_tke, max_tke, mean_U, mean_V, mean_W, xlim, ylim, display_min_tke, display_max_tke)
    generate_energy_plot(output_folder, traditional_energy_data_file, improved_energy_data_file)