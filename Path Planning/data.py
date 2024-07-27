import numpy as np

# Load the binary occupancy grid from a .npy file
grid = np.load('Map/data.npy')

# Define start point, end point, and waypoints as lists of (x, y) tuples
start_point = (400, 700)
end_point = (300,700)
waypoints = [(700,711), (765,656), (730,400) ,(810,185) , (713,95) , (550,80) , (418,65) , (66,103), (35,218), (137,366),  (98,524), (58,673)  ]

import matplotlib.pyplot as plt

def visualize_grid_with_points(grid, start_point, end_point, waypoints):
    plt.figure(figsize=(10, 10))
    plt.imshow(grid, cmap='gray', interpolation='none')

    # Plot the start point
    plt.plot(start_point[0], start_point[1], 'go', markersize=10)  # 'go' means green circle

    # Plot the end point
    plt.plot(end_point[0], end_point[1], 'bo', markersize=10)  # 'bo' means blue circle

    # Extract x and y coordinates from waypoints
    x_coords = [x for (x, y) in waypoints]
    y_coords = [y for (x, y) in waypoints]

    # Plot the waypoints
    plt.plot(x_coords, y_coords, 'ro-', markersize=5)  # 'ro-' means red circles connected by lines

    # Annotate the start and end points
    plt.text(start_point[0], start_point[1], 'Start', color='green', fontsize=12, ha='right')
    plt.text(end_point[0], end_point[1], 'End', color='blue', fontsize=12, ha='right')

    # Annotate each waypoint
    for (i, (x, y)) in enumerate(waypoints):
        plt.text(x, y, f'({x},{y})', color='red', fontsize=12, ha='right')

    plt.title('Binary Occupancy Grid with Start, End, and Waypoints')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(False)
    plt.show()

# Call the function with the grid, start point, end point, and waypoints
visualize_grid_with_points(grid, start_point, end_point, waypoints)


