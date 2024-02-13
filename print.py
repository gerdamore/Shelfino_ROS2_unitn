import matplotlib.pyplot as plt
import csv
from matplotlib.patches import Rectangle

# Read trajectory points from the file
trajectory_points = []
with open('points.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    for row in csvreader:
        x, y, theta = map(float, row)
        trajectory_points.append((x, y))

# Read obstacle points from the file
obstacle_points = []
with open('obstacles.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    for row in csvreader:
        x_bl, y_bl, x_br, y_br, x_tr, y_tr, x_tl, y_tl, radius = map(float, row)
        # Extracting points for the box
        bottom_left = (x_bl, y_bl)
        top_left = (x_tl, y_tl)
        top_right = (x_tr, y_tr)
        bottom_right = (x_br, y_br)
        # Storing all points
        obstacle_points.append((bottom_left, top_left, top_right, bottom_right))

# Read victim points from the file
victim_points = []
with open('victims.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    for row in csvreader:
        x, y = map(float, row)
        victim_points.append((x, y))

# Plot trajectory points
x_values_traj = [point[0] for point in trajectory_points]
y_values_traj = [point[1] for point in trajectory_points]

plt.plot(x_values_traj, y_values_traj, 'ro')  # Plot trajectory points as red dots

# Plot obstacle points
for obstacle in obstacle_points:
    bottom_left, top_left, top_right, bottom_right = obstacle
    x_values_obst = [bottom_left[0], top_left[0], top_right[0], bottom_right[0], bottom_left[0]]
    y_values_obst = [bottom_left[1], top_left[1], top_right[1], bottom_right[1], bottom_left[1]]
    plt.plot(x_values_obst, y_values_obst, 'b-')  # Plot obstacle as a blue rectangle

# Plot victim points
x_values_victim = [point[0] for point in victim_points]
y_values_victim = [point[1] for point in victim_points]
plt.plot(x_values_victim, y_values_victim, 'gx')  # Plot victim points as green crosses

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectory with Obstacles and Victims')
plt.grid(True)
plt.show()
