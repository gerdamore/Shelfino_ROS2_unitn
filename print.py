import matplotlib.pyplot as plt
import csv

# Read points from the file
points = []
with open('points.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    for row in csvreader:
        x, y, theta = map(float, row)
        points.append((x, y))

# Plot points
x_values = [point[0] for point in points]
y_values = [point[1] for point in points]

plt.plot(x_values, y_values, 'ro')  # Plot points as red dots
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectory')
plt.grid(True)
plt.show()


