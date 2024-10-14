import matplotlib.pyplot as plt
import numpy as np

# Define data
uav_speeds = ['8 m/s', '16 m/s']
energy_reduction = [5.05, 1.29]
path_lengths = [277.6, 273.46]

# Define the width of the bars
bar_width = 0.35

# Define the positions of the bars
index = np.arange(len(uav_speeds))

# Create the figure and the first axis
fig, ax1 = plt.subplots()

# Plot the energy reduction rate bar chart on the first axis
bars1 = ax1.bar(index - bar_width/2, energy_reduction, bar_width, label='Energy Reduction Rate (%)', color='skyblue')

# Create the second axis sharing the same x-axis
ax2 = ax1.twinx()

# Plot the flight path length bar chart on the second axis
bars2 = ax2.bar(index + bar_width/2, path_lengths, bar_width, label='Flight Path Length (m)', color='orange')

# Add data labels for energy reduction rate
for bar in bars1:
    yval = bar.get_height()
    ax1.text(bar.get_x() + bar.get_width()/2, yval + 0.1, round(yval, 2), ha='center', va='bottom', fontsize=10)

# Add data labels for flight path length
for bar in bars2:
    yval = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2, yval + 5, round(yval, 2), ha='center', va='bottom', fontsize=10)

# Add labels and title
ax1.set_xlabel('UAV Flight Speed', fontsize=12)
ax1.set_ylabel('Energy Reduction Rate (%)', fontsize=12, color='skyblue')
ax2.set_ylabel('Flight Path Length (m)', fontsize=12, color='orange')
plt.title('Comparison of Energy Reduction Rate and Flight Path Length at Different UAV Speeds', fontsize=14)
ax1.set_xticks(index)
ax1.set_xticklabels(uav_speeds, fontsize=12)

# Add legends
bars = [bars1, bars2]
labels = [bar.get_label() for bar in bars]
ax1.legend(bars, labels, loc='lower left', fontsize=12)

# Show grid for both axes
ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
ax2.grid(False)  # Disable grid for the second axis

# Save the figure
plt.savefig('UAV_energy_path_comparison.png', dpi=300, bbox_inches='tight')

# Show the figure
plt.show()