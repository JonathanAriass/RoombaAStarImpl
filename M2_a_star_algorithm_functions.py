from typing import List, Tuple, Dict, Set
import numpy as np
import heapq
from math import sqrt

def create_node(position: Tuple[int, int], g: float = float('inf'),
                h: float = 0.0, parent: Dict = None) -> Dict:
    """
        Create a node for the A* algorithm.

        Args:
            position: (x, y) coordinates of the node
            g: Cost from start to this node (default: infinity)
            h: Estimated cost from this node to goal (default: 0)
            parent: Parent node (default: None)

        Returns:
            Dictionary containing node information
    """
    return {
        'position': position,
        'g': g,
        'h': h,
        'f': g + h,
        'parent': parent
    }


def calculate_euclidian_distance(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
    """
        Calculate the Euclidian distance between two points.

        Args:
            pos1: (x, y) coordinates of the first point
            pos2: (x, y) coordinates of the second point

        Returns:
            Euclidian distance between the two points
    """
    x1, y1 = pos1
    x2, y2 = pos2

    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def get_valid_neighbours(grid: np.ndarray, position: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
        Get valid neighbours of a given node and surrounding nodes.

        Args:
            grid: Grid with obstacles
            position: (x, y) coordinates of the node

        Returns:
            List of valid neighbours
    """
    x, y = position
    rows, cols = grid.shape
    robot_width, robot_height = 50, 50

    # TODO: Take into account that position of robot is 50x50 on grid

    possible_moves = [
        (x + robot_width, y),     # Right
        (x - 1, y),               # Left
        (x, y + robot_height),    # Down
        (x, y - 1),               # Up
        (x + robot_width, y + robot_height),  # Diagonal Right Down
        (x + robot_width, y - 1),             # Diagonal Right Up
        (x - 1, y + robot_height),            # Diagonal Left Down
        (x - 1, y - 1)                        # Diagonal Left Up
    ]

    def is_valid(nx, ny):
        """
            Check if the 50x50 area is within bounds and free of obstacles.

            Args:
                nx: x coordinate of the node
                ny: y coordinate of the node
        """
        if 0 <= nx < rows - robot_width + 1 and 0 <= ny < cols - robot_height + 1:
            # Check that the 50x50 area is all zeros
            for i in range(nx, nx + robot_width):
                for j in range(ny, ny + robot_height):
                    if grid[i, j] != 0:
                        return False
            return True
        return False

    return [
        (nx, ny) for nx, ny in possible_moves
        if is_valid(nx, ny)
    ]


def reconstruct_path(goal_node: Dict) -> List[Tuple[int, int]]:
    """
        Reconstruct the path from the goal node to the start node.

        Args:
            goal_node: Goal node

        Returns:
            List of nodes from start to goal    
    """
    path = []
    current = goal_node

    while current is not None:
        path.append(current['position'])
        current = current['parent']
    
    return path[::-1] # Reverse path (start to goal)


def find_path(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
        Find the best path from start to goal using the A* algorithm.

        Args:
            grid: Grid with obstacles
            start: (x, y) coordinates of the start node
            goal: (x, y) coordinates of the goal node

        Returns:
            List of nodes from start to goal
    """
    # Initialize start node
    start_node = create_node(position=start, g=0, h=calculate_euclidian_distance(start, goal))

    # Initialize open and closed lists
    open_list = [(start_node['f'], start)] # Priority queue
    open_dict = {start: start_node} # For quick node lookup
    closed_list = set() # Explored nodes

    while open_list:
        print("###########################")
        print(open_list)
        print("###########################")
        print()
        print()

        # Get node with lowest f value
        _, current_pos = heapq.heappop(open_list)
        current_node = open_dict[current_pos]
        
        # Check if goal is reached
        if current_pos == goal:
            return reconstruct_path(current_node)
        
        # Add current node to closed list
        closed_list.add(current_pos)

        # Explore neighbours
        for neighbour_pos in get_valid_neighbours(grid, current_pos):
            # Skip if already explored
            if neighbour_pos in closed_list:
                continue

            # Calculate g and h values
            tentative_g = current_node['g'] + calculate_euclidian_distance(current_pos, neighbour_pos)

            # Create or update neighbour node
            if neighbour_pos not in open_list:
                neighbour = create_node(position=neighbour_pos, g=tentative_g, h=calculate_euclidian_distance(neighbour_pos, goal), parent=current_node)

                # Add neighbour to open list
                heapq.heappush(open_list, (neighbour['f'], neighbour_pos))
                
                # Add neighbour to open dict
                open_dict[neighbour_pos] = neighbour

            elif tentative_g < open_dict[neighbour_pos]['g']:
                # Found a better path to the neighbour
                neighbour = open_dict[neighbour_pos]
                neighbour['g'] = tentative_g
                neighbour['f'] = tentative_g + neighbour['h']
                neighbour['parent'] = current_node
    
    return [] # No path found

