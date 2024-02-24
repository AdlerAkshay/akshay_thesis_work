import matplotlib.pyplot as plt

# Sample data
x = [1, 2, 3, 4, 5]
y = [2, 4, 6, 8, 10]

# Plotting the line
plt.plot(x, y, label='Line')

# Adding text label to a specific point on the line
index_to_label = 3
label_text = 'Label Point'

# Get the coordinates of the point on the line
label_x = x[index_to_label]
label_y = y[index_to_label]

# Add the text label to the plot
plt.text(label_x, label_y, label_text, color='red', fontsize=12, ha='right')

# Adding labels and legend
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Line Plot with Text Label')
plt.legend()

# Show the plot
plt.show()
