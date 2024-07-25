import math
import matplotlib.pyplot as plt

# Initialize parameters
initial_x = 0
initial_y = 0
x = initial_x
y = initial_y
z = 0  # assuming z is constant for this simulation
yaw = 0
velocity = 0.05  # Example constant velocity
dt = 1 # Time step
steps_per_control = 1  # Example number of steps per control
max_change = 1  # Example maximum change in position

# Example angular velocity sequence
angular_velocity_sequence = [0.0,0.0,0.0,0.0,+math.pi/4,0.0,0.0,0.0,0.0,0.0,0.0,+math.pi/4,0.0,0.0,0.0,0.0]

# List to store coordinates
coordinates = []

for angular_velocity in angular_velocity_sequence:
    print('Setting angular velocity {}'.format(angular_velocity))
    for _ in range(steps_per_control):
        # Update position based on Dubins car dynamics
        x += velocity * math.cos(yaw) * dt
        y += velocity * math.sin(yaw) * dt
        yaw += angular_velocity * dt
        print(x, y, z, yaw)

        # Append coordinates to list
        coordinates.append((x, y))

        # Check if the total change exceeds the limit
        if abs(x - initial_x) > max_change or abs(y - initial_y) > max_change:
            print('Exceeded safety limit. Landing...')
            break

# Plotting the coordinates
x_coords, y_coords = zip(*coordinates)
plt.plot(x_coords, y_coords, marker='o')
plt.title('Trajectory of Dubins Car')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.xlim(-1.1*max_change, 1.1*max_change)
plt.ylim(-1.1*max_change, 1.1*max_change)
plt.grid(True)
plt.axhline(0, color='black',linewidth=0.5)
plt.axvline(0, color='black',linewidth=0.5)
plt.show()
