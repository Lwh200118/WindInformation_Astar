import numpy as np

def generate_path(start, end, grid_size):
    """
    生成从起点到终点的直线路径。
    参数:
    - start: 起点坐标 (x, y)
    - end: 终点坐标 (x, y)
    - grid_size: 网格的大小
    返回:
    - path: 包含路径上所有点的列表
    """
    # 计算起点和终点在网格中的坐标
    start_grid = np.array(start) / grid_size
    end_grid = np.array(end) / grid_size
    
    # 计算直线路径需要的步数
    steps = int(max(abs(end_grid - start_grid)))
    
    # 在网格坐标系中生成直线路径
    x_coords = np.linspace(start_grid[0], end_grid[0], steps + 1)
    y_coords = np.linspace(start_grid[1], end_grid[1], steps + 1)
    
    # 转换回实际坐标系
    path = np.stack([x_coords, y_coords], axis=-1) * grid_size
    return path

def save_path_to_file(path, filename):
    """
    将路径保存到.dat文件中。
    参数:
    - path: 路径列表
    - filename: 文件名
    """
    with open(filename, 'w') as file:
        file.write('X,Y\n')
        for point in path:
            file.write(f'{point[0]},{point[1]}\n')

def main():
    start = (50, 20)  # 起点坐标
    end = (230, 100)  # 终点坐标
    grid_size = 1.25  # 网格大小
    
    # 生成路径
    path = generate_path(start, end, grid_size)
    
    # 保存路径到.dat文件
    save_path_to_file(path, 'Direct_path_50_20_to_230_100.dat')
    
    print("路径已保存到 path.dat 文件中。")

if __name__ == '__main__':
    main()