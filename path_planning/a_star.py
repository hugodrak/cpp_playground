import heapq
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