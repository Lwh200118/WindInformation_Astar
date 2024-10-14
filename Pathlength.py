import math

# 定义一个函数来计算两点之间的欧几里得距离
def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# 读取dat文件并解析数据
def read_dat_file(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
    
    # 跳过第一行（标题行），解析后续行的数据
    data = []
    for line in lines[1:]:
        x, y = map(float, line.strip().split(','))
        data.append((x, y))
    
    return data

# 计算总路线的长度
def calculate_total_route_length(data):
    total_length = 0.0
    for i in range(1, len(data)):
        total_length += euclidean_distance(data[i-1][0], data[i-1][1], data[i][0], data[i][1])
    
    return total_length

# 主函数
def main():
    filename = r"E:\UAV\研究计划\TEST\Instant_City_Sustech\New_optimized_UW_PM_path_Instant_City_0_200_to_320_200_V8UTT_More_Mod_1_0.dat"  # 请将此处替换为你的dat文件的实际路径
    data = read_dat_file(filename)
    total_length = calculate_total_route_length(data)
    print(f"总路线的长度为: {total_length:.2f}")

if __name__ == "__main__":
    main()