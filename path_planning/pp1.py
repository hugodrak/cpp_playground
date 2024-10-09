import heapq

import matplotlib.pyplot as plt
import numpy as np


class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to node
        self.h = 0  # Heuristic, estimated cost from node to end
        self.f = 0  # Total cost

    def __lt__(self, other):
        return self.f < other.f

def a_star(grid, start, end):
    # Create start and end node
    start_node = Node(start)
    end_node = Node(end)

    # Initialize open and closed list
    open_list = []
    closed_list = set()

    # Add the start node
    heapq.heappush(open_list, start_node)

    # Loop until the end
    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)

        # Check if we reached the end
        if current_node.position == end_node.position:
            path = []
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            
            # Make sure within range and not in water
            if node_position[0] > (len(grid) - 1) or node_position[0] < 0 or node_position[1] > (len(grid[len(grid)-1]) - 1) or node_position[1] < 0:
                continue
            if grid[node_position[0]][node_position[1]] != 0:
                continue
            
            # Create new node and append
            new_node = Node(node_position, current_node)
            children.append(new_node)

        # Loop through children
        for child in children:
            if child.position in closed_list:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            heapq.heappush(open_list, child)

    return None  # Path not found

# Example grid (0 = water, 1 = obstacle)
grid = [
    [0, 0, 0, 0, 1],
    [0, 1, 1, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0]
]

start = (0, 0)  # Starting position
end = (4, 4)   # Ending position


# Assuming the rest of the A* code is defined here...

path = a_star(grid, start, end)
print(path)

# Convert path to a grid for visualization
path_grid = np.zeros_like(grid)
for point in path:
    path_grid[point[0]][point[1]] = 2  # 2 will represent the path

# Update the path_grid with obstacles and start/end points
for i in range(len(grid)):
    for j in range(len(grid[i])):
        if grid[i][j] == 1:
            path_grid[i][j] = 1  # 1 represents obstacles
path_grid[start[0]][start[1]] = 3  # 3 represents the start point
path_grid[end[0]][end[1]] = 4      # 4 represents the end point

# Set up the color map (water: blue, obstacle: black, path: yellow, start: green, end: red)
cmap = plt.cm.jet
cmap.set_under('blue')  # Water
cmap.set_over('red')    # End
norm = plt.Normalize(0.5, 4.5)

# Create the plot
fig, ax = plt.subplots()
ax.imshow(path_grid, cmap=cmap, norm=norm)

# Add gridlines and set ticks
ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
ax.set_xticks(np.arange(-.5, len(grid[0]), 1))
ax.set_yticks(np.arange(-.5, len(grid), 1))

# Remove tick labels
ax.set_xticklabels([])
ax.set_yticklabels([])

# Show the plot
plt.show()

