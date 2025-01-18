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
        if 0 <= nx < rows and 0 <= ny < cols
        and grid[nx, ny] == 0  # Not an obstacle
    ]
