import numpy as np
import pandas as pd

def read_path_from_dat(file_path):
    """
    从 .dat 文件中读取路径。
    
    参数：
    - file_path: .dat 文件路径
    
    返回：
    - path: 路径，格式为[(x1, y1), (x2, y2), ...]
    """
    data = pd.read_csv(file_path, delim_whitespace=True, skiprows=1)  # 跳过第一行
    path = list(zip(data.iloc[:, 0], data.iloc[:, 1]))
    return path

def calculate_overlap(path1, path2, tolerance=1e-5):
    """
    计算两条路径的重合度。
    
    参数：
    - path1: 第一条路径，格式为[(x1, y1), (x2, y2), ...]
    - path2: 第二条路径，格式为[(x1, y1), (x2, y2), ...]
    - tolerance: 判断两点是否重合的容差，默认值为1e-5
    
    返回：
    - overlap_ratio: 重合度，取值范围为0到1
    """
    path1 = np.array(path1)
    path2 = np.array(path2)
    
    overlap_count = 0
    
    for point1 in path1:
        for point2 in path2:
            if np.linalg.norm(point1 - point2) < tolerance:
                overlap_count += 1
                break
    
    overlap_ratio = overlap_count / len(path1)
    
    return overlap_ratio

def main():
    # 文件路径
    file_path1 = r"E:\UAV\研究计划\TEST\V8_Instant\New_optimized_UW_PM_path_Instant_50_100_to_230_100_V8UTT_More_Mod_0_2.dat"
    file_path2 = r"E:\UAV\研究计划\TEST\V8_Instant\New_optimized_UW_PM_path_Instant_50_100_to_230_100_V8UTT_More_Mod_1_0.dat"
    
    # 读取路径
    path1 = read_path_from_dat(file_path1)
    path2 = read_path_from_dat(file_path2)
    
    # 计算重合度
    overlap_ratio = calculate_overlap(path1, path2)
    print(f"两条路径的重合度为: {overlap_ratio:.2f}")

if __name__ == "__main__":
    main()