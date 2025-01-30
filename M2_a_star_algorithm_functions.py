from typing import List, Tuple, Dict, Set
import numpy as np
import heapq
from math import sqrt
import matplotlib.pyplot as plt
import pdb


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
    # scaled_h = h * 1.5
    weigth = 1.1
    # Use Chen and Sturvetant’s AAAI 2021 paper Necessary and Sufficient
    # Conditions for Avoiding Reopenings in Best First Suboptimal Search
    # with General Bounding Functions to scale the heuristic
    scaled_f = g + h if g < h else (g + (2 * weigth - 1) * h) / weigth

    return {
        'position': position,
        'g': g,
        'h': h,
        'f': scaled_f,
        'parent': parent
    }


def calculate_manhattan_distance(current_position: Tuple[int, int], goal_position: Tuple[int, int]) -> float:
    """
        Calculate the Euclidian distance between two points.

        Args:
            current_position: (x, y) coordinates of the current point
            goal_position: (x, y) coordinates of the goal point

        Returns:
            Euclidian distance between the two points
    """
    current_x, current_y = current_position
    goal_x, goal_y = goal_position

    return abs(current_x - goal_x) + abs(current_y - goal_y)


def calculate_diagonal_distance(current_position: Tuple[int, int], goal_position: Tuple[int, int]) -> float:
    """
        Calculate the diagonal distance between two points. As D and D2 are
        equal to 1, it gets the name of Chebyshev distance. It is also known
        as chessboard distance.

        Args:
            current_position: (x, y) coordinates of the current point
            goal_position: (x, y) coordinates of the goal point

        Returns:
            Diagonal distance between the two points
    """
    current_x, current_y = current_position
    goal_x, goal_y = goal_position

    return max(abs(current_x - goal_x), abs(current_y - goal_y))


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

    # All possible moves (up, down, left, right and diagonals)
    possible_moves = [
        (x + 1, y),  # Right
        (x - 1, y),  # Left
        (x, y + 1),  # Down
        (x, y - 1),  # Up
        (x + 1, y + 1),  # Diagonal right-down
        (x + 1, y - 1),  # Diagonal right-up
        (x - 1, y + 1),  # Diagonal left-down
        (x - 1, y - 1)  # Diagonal left-up
    ]

    return [
        (nx, ny) for nx, ny in possible_moves
        if 0 <= nx < rows and 0 <= ny < cols  # Within grid bounds
        and (grid[nx, ny] == 0 or grid[nx, ny] == 3)  # Not an obstacle
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

    return path[::-1]  # Reverse path (start to goal)


def find_path(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]):
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
    start_node = create_node(position=start, g=0, h=calculate_diagonal_distance(start, goal))

    # Initialize heat map
    heat_map = np.zeros_like(grid, dtype=float)

    # Initialize open and closed lists
    open_list = [(start_node['f'], start)]  # Priority queue
    open_dict = {start: start_node}  # For quick node lookup
    closed_list = set()  # Explored nodes

    # Initialize visit count array
    visit_count = np.zeros_like(grid, dtype=int)

    while open_list:
        # Get node with lowest f value
        _, current_pos = heapq.heappop(open_list)
        current_node = open_dict[current_pos]

        # Update visit count and heat map
        visit_count[current_pos[0]][current_pos[1]] += 1
        heat_map[current_pos[0]][current_pos[1]] = np.log1p(visit_count[current_pos[0]][current_pos[1]])

        # Check if goal is reached
        if current_pos == goal:
            path = reconstruct_path(current_node)
            return path, heat_map

        # Add current node to closed list
        closed_list.add(current_pos)

        # Explore neighbours
        for neighbour_pos in get_valid_neighbours(grid, current_pos):

            # Skip if already explored
            if neighbour_pos in closed_list:
                continue

            # Calculate g and h values
            tentative_g = current_node['g'] + calculate_diagonal_distance(current_pos, neighbour_pos)

            # Create or update neighbour node
            if neighbour_pos not in open_list:
                neighbour = create_node(position=neighbour_pos, g=tentative_g, h=calculate_diagonal_distance(neighbour_pos, goal), parent=current_node)

                # Add neighbour to open list
                heapq.heappush(open_list, (neighbour['f'], neighbour_pos))

                # Add neighbour to open dict
                open_dict[neighbour_pos] = neighbour

            elif tentative_g < open_dict[neighbour_pos]['g']:
                # Found a better path to the neighbour
                neighbour = open_dict[neighbour_pos]
                neighbour['g'] = tentative_g
                # Apply Chen and Sturvetant’s AAAI 2021 paper Necessary and Sufficient
                # Conditions for Avoiding Reopenings in Best First Suboptimal Search
                # with General Bounding Functions to scale the heuristic
                neighbour['f'] = tentative_g + neighbour['h'] if tentative_g < neighbour['h'] else (tentative_g + (2 * 1.1 - 1) * neighbour['h']) / 1.1
                # neighbour['f'] = tentative_g + neighbour['h']
                neighbour['parent'] = current_node

    return [], []  # No path found
