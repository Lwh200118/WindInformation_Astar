import matplotlib.pyplot as plt
import numpy as np

# Data preparation
data = {
    'Path Length': [100, 180, 230],  # Example path lengths
    'Traditional A* Wind Speed 8m/s': [17053.4951, 28660.76738, 44432.72812],  # Traditional A* energy at wind speed 8 m/s
    'Improved A* Wind Speed 8m/s': [16716.28994, 28150.02068, 42190.55988],  # Improved A* energy at wind speed 8 m/s
    'Traditional A* Wind Speed 3.5m/s': [20847.5946040148, 35361.010270918, 49681.302329142],  # Traditional A* energy at wind speed 3.5 m/s
    'Improved A* Wind Speed 3.5m/s': [19992.3744904968, 33715.5289358806, 47017.0736013773]  # Improved A* energy at wind speed 3.5 m/s
}

# Calculate energy reduction rates
data['Energy Reduction Rate 8 m/s'] = [(data['Traditional A* Wind Speed 8m/s'][i] - data['Improved A* Wind Speed 8m/s'][i]) / data['Traditional A* Wind Speed 8m/s'][i] * 100 for i in range(len(data['Path Length']))]
data['Energy Reduction Rate 3.5 m/s'] = [(data['Traditional A* Wind Speed 3.5m/s'][i] - data['Improved A* Wind Speed 3.5m/s'][i]) / data['Traditional A* Wind Speed 3.5m/s'][i] * 100 for i in range(len(data['Path Length']))]

# Polynomial fitting for 8 m/s
coeffs_8ms = np.polyfit(data['Path Length'], data['Energy Reduction Rate 8 m/s'], 2)  # 2nd degree polynomial
poly_8ms = np.poly1d(coeffs_8ms)
x_fit_8ms = np.linspace(min(data['Path Length']), max(data['Path Length']), 100)
y_fit_8ms = poly_8ms(x_fit_8ms)

# Polynomial fitting for 3.5 m/s
coeffs_3_5ms = np.polyfit(data['Path Length'], data['Energy Reduction Rate 3.5 m/s'], 2)  # 2nd degree polynomial
poly_3_5ms = np.poly1d(coeffs_3_5ms)
x_fit_3_5ms = np.linspace(min(data['Path Length']), max(data['Path Length']), 100)
y_fit_3_5ms = poly_3_5ms(x_fit_3_5ms)

# Plotting the data
plt.figure(figsize=(10, 6))

# Plot for 8 m/s
plt.plot(data['Path Length'], data['Energy Reduction Rate 8 m/s'], marker='o', linestyle='-', color='b', label='Energy Reduction Rate 8 m/s')
plt.plot(x_fit_8ms, y_fit_8ms, linestyle='--', color='b', label='Fitted Line 8 m/s')

# Plot for 3.5 m/s
plt.plot(data['Path Length'], data['Energy Reduction Rate 3.5 m/s'], marker='o', linestyle='-', color='r', label='Energy Reduction Rate 3.5 m/s')
plt.plot(x_fit_3_5ms, y_fit_3_5ms, linestyle='--', color='r', label='Fitted Line 3.5 m/s')

# Annotate data points
for i in range(len(data['Path Length'])):
    plt.annotate(f"{data['Energy Reduction Rate 8 m/s'][i]:.2f}%", 
                 (data['Path Length'][i], data['Energy Reduction Rate 8 m/s'][i]), 
                 textcoords="offset points", xytext=(0,10), ha='center', color='b')
    plt.annotate(f"{data['Energy Reduction Rate 3.5 m/s'][i]:.2f}%", 
                 (data['Path Length'][i], data['Energy Reduction Rate 3.5 m/s'][i]), 
                 textcoords="offset points", xytext=(0,-15), ha='center', color='r')

# Chart title and labels
plt.title('Energy Reduction Rate Comparison at Different Wind Speeds', fontsize=14)
plt.xlabel('Path Length (m)', fontsize=12)
plt.ylabel('Energy Reduction Rate (%)', fontsize=12)
plt.legend(loc='upper left', fontsize=12)
plt.grid(True)

# Display the chart
plt.show()