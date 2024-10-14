import matplotlib.pyplot as plt
import numpy as np

# Data preparation
data = {
    'Load Weight': [6.2, 7.2, 7.7],  # Example load weights in kg
    'Traditional A* Load': [17053.4951045781, 22661.6039299642, 24510.7797162985],  # Traditional A* energy at different loads
    'Improved A* Load': [16716.2899420256, 21660.354240967, 23422.7298337956],  # Improved A* energy at different loads
}

# Calculate energy reduction rates
data['Energy Reduction Rate'] = [(data['Traditional A* Load'][i] - data['Improved A* Load'][i]) / data['Traditional A* Load'][i] * 100 for i in range(len(data['Load Weight']))]

# Polynomial fitting for energy reduction rate
coeffs = np.polyfit(data['Load Weight'], data['Energy Reduction Rate'], 2)  # 2nd degree polynomial
poly = np.poly1d(coeffs)
x_fit = np.linspace(min(data['Load Weight']), max(data['Load Weight']), 100)
y_fit = poly(x_fit)

# Plotting the data
plt.figure(figsize=(10, 6))

# Plot for energy reduction rate
plt.plot(data['Load Weight'], data['Energy Reduction Rate'], marker='o', linestyle='-', color='b', label='Energy Reduction Rate')
plt.plot(x_fit, y_fit, linestyle='--', color='b', label='Fitted Line')

# Annotate data points
for i in range(len(data['Load Weight'])):
    plt.annotate(f"{data['Energy Reduction Rate'][i]:.2f}%", 
                 (data['Load Weight'][i], data['Energy Reduction Rate'][i]), 
                 textcoords="offset points", xytext=(0,10), ha='center', color='b')

# Chart title and labels
plt.title('Energy Reduction Rate Comparison at Different Load Weights', fontsize=14)
plt.xlabel('Load Weight (kg)', fontsize=12)
plt.ylabel('Energy Reduction Rate (%)', fontsize=12)
plt.legend(loc='upper left', fontsize=12)
plt.grid(True)

# Display the chart
plt.show()