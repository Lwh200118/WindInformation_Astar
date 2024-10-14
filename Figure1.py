import matplotlib.pyplot as plt
import numpy as np

# Data preparation
data = {
    'Path Length': [100, 180, 230],  # Example path lengths
    'Traditional A* Energy 8 m/s': [17053.4951, 28660.76738, 44432.72812],  # Traditional A* energy at 8 m/s
    'Improved A* Energy 8 m/s': [16716.28994, 28150.02068, 42190.55988],  # Improved A* energy at 8 m/s
    'Traditional A* Energy 16 m/s': [10162.60735, 17226.51905, 25129.88338],  # Traditional A* energy at 16 m/s (example)
    'Improved A* Energy 16 m/s': [10099.97952, 17132.45867, 24804.95625]  # Improved A* energy at 16 m/s (example)
}

# Calculate energy reduction rates
data['Energy Reduction Rate 8 m/s'] = [(data['Traditional A* Energy 8 m/s'][i] - data['Improved A* Energy 8 m/s'][i]) / data['Traditional A* Energy 8 m/s'][i] * 100 for i in range(len(data['Path Length']))]
data['Energy Reduction Rate 16 m/s'] = [(data['Traditional A* Energy 16 m/s'][i] - data['Improved A* Energy 16 m/s'][i]) / data['Traditional A* Energy 16 m/s'][i] * 100 for i in range(len(data['Path Length']))]

# Polynomial fitting for 8 m/s
coeffs_8ms = np.polyfit(data['Path Length'], data['Energy Reduction Rate 8 m/s'], 2)  # 2nd degree polynomial
poly_8ms = np.poly1d(coeffs_8ms)
x_fit_8ms = np.linspace(min(data['Path Length']), max(data['Path Length']), 100)
y_fit_8ms = poly_8ms(x_fit_8ms)

# Polynomial fitting for 16 m/s
coeffs_16ms = np.polyfit(data['Path Length'], data['Energy Reduction Rate 16 m/s'], 2)  # 2nd degree polynomial
poly_16ms = np.poly1d(coeffs_16ms)
x_fit_16ms = np.linspace(min(data['Path Length']), max(data['Path Length']), 100)
y_fit_16ms = poly_16ms(x_fit_16ms)

# Plotting the data
plt.figure(figsize=(10, 6))

# Plot for 8 m/s
plt.plot(data['Path Length'], data['Energy Reduction Rate 8 m/s'], marker='o', linestyle='-', color='b', label='Energy Reduction Rate 8 m/s')
plt.plot(x_fit_8ms, y_fit_8ms, linestyle='--', color='b', label='Fitted Line 8 m/s')

# Plot for 16 m/s
plt.plot(data['Path Length'], data['Energy Reduction Rate 16 m/s'], marker='o', linestyle='-', color='r', label='Energy Reduction Rate 16 m/s')
plt.plot(x_fit_16ms, y_fit_16ms, linestyle='--', color='r', label='Fitted Line 16 m/s')

# Annotate data points
for i in range(len(data['Path Length'])):
    plt.annotate(f"{data['Energy Reduction Rate 8 m/s'][i]:.2f}%", 
                 (data['Path Length'][i], data['Energy Reduction Rate 8 m/s'][i]), 
                 textcoords="offset points", xytext=(0,10), ha='center', color='b')
    plt.annotate(f"{data['Energy Reduction Rate 16 m/s'][i]:.2f}%", 
                 (data['Path Length'][i], data['Energy Reduction Rate 16 m/s'][i]), 
                 textcoords="offset points", xytext=(0,-15), ha='center', color='r')

# Display polynomial equations
# plt.text(150, max(y_fit_8ms), f'8 m/s: y = {coeffs_8ms[0]:.2e}x^2 + {coeffs_8ms[1]:.2e}x + {coeffs_8ms[2]:.2e}', color='b')
# plt.text(150, max(y_fit_16ms) , f'16 m/s: y = {coeffs_16ms[0]:.2e}x^2 + {coeffs_16ms[1]:.2e}x + {coeffs_16ms[2]:.2e}', color='r')

# Chart title and labels
plt.title('Energy Reduction Rate Comparison Between Traditional and Improved A* Algorithm', fontsize=14)
plt.xlabel('Path Length (m)', fontsize=12)
plt.ylabel('Energy Reduction Rate (%)', fontsize=12)
plt.legend()
plt.grid(True)

# Display the chart
plt.show()