import numpy as np

def read_wind_field(file_path):
    """
    读取风场数据文件，并返回X, Y, U, V四列数据
    """
    wind_data = np.loadtxt(file_path)
    X = wind_data[:, 0]
    Y = wind_data[:, 1]
    U = wind_data[:, 3]
    V = wind_data[:, 4]
    return X, Y, U, V

def extract_wind_field_in_region(X, Y, U, V, x_min, x_max, y_min, y_max):
    """
    提取在指定显示区域内的风场数据
    """
    mask = (X >= x_min) & (X <= x_max) & (Y >= y_min) & (Y <= y_max)
    return X[mask], Y[mask], U[mask], V[mask]

def save_extracted_data(file_path, X, Y, U, V):
    """
    保存提取的数据到文件
    """
    extracted_data = np.vstack((X, Y, U, V)).T
    np.savetxt(file_path, extracted_data, header='X Y U V', comments='')

def main(input_file, output_file, x_min, x_max, y_min, y_max):
    # 读取风场数据
    X, Y, U, V = read_wind_field(input_file)
    
    # 提取在显示区域内的数据
    X_extracted, Y_extracted, U_extracted, V_extracted = extract_wind_field_in_region(X, Y, U, V, x_min, x_max, y_min, y_max)
    
    # 保存提取的数据
    save_extracted_data(output_file, X_extracted, Y_extracted, U_extracted, V_extracted)
    print(f"提取的数据已保存到 {output_file}")

if __name__ == '__main__':
    input_file = r"E:\gtian\BlockMesh-LES-1600w-west\postProcessing\surfaces\3600\U_0_mySurfaces_60.dat"
    output_file = r"E:\UAV\研究计划\TEST\extracted_wind_field.dat"
    x_min, x_max = -100, 500  # 显示区域的X轴范围
    y_min, y_max = 0, 400     # 显示区域的Y轴范围
    
    main(input_file, output_file, x_min, x_max, y_min, y_max)