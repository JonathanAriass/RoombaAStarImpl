import numpy as np

GRID_SIZE = 40  # Define a smaller grid size (e.g., 40x40)
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 800  # Screen dimensions
CELL_SIZE = SCREEN_WIDTH // GRID_SIZE  # Size of each cell in the grid


def map_to_grid(x, y):
    """
    Map screen coordinates to grid coordinates.
    """
    return x // CELL_SIZE, y // CELL_SIZE


def map_to_screen(grid_x, grid_y):
    """
    Map grid coordinates back to screen coordinates.
    """
    return grid_x * CELL_SIZE, grid_y * CELL_SIZE


def track_distance(robot_x, robot_y, origin_x, origin_y):
    """
        Calculate and return an array of points representing the path of the
        robot to the origin point. Check collision with obstacles and return
        the path of the robot to the origin point surrounded by obstacles.

        Args:
            robot_x: x-coordinate of the robot
            robot_y: y-coordinate of the robot
            origin_x: x-coordinate of the origin point
            origin_y: y-coordinate of the origin point

        Returns:
            Array of points representing the path of the robot to the origin point.
    """
    # Coords to track path
    coords = []
    dx = origin_x - robot_x
    dy = origin_y - robot_y
    steps = max(abs(dx), abs(dy))
    step_x = dx / steps
    step_y = dy / steps

    # Transform steps into integers
    steps = int(steps)

    # Generate the path of the robot to the origin point
    for i in range(steps):
        x = robot_x + i * step_x
        y = robot_y + i * step_y
        coords.append((x, y))
    return coords


def transform_screen_to_grid(robot_x, robot_y, origin_x, origin_y, obstacles):
    """
        Transform the screen into a grid of GRID_SIZE x GRID_SIZE.

        Args:
            robot_x: x-coordinate of the robot
            robot_y: y-coordinate of the robot
            origin_x: x-coordinate of the origin point
            origin_y: y-coordinate of the origin point
            obstacles: List of obstacles

        Returns:
            Grid with obstacles, origin point, and player marked
    """
    # Initialize grid with zeros
    grid = np.zeros((GRID_SIZE, GRID_SIZE))

    # Add obstacles to grid
    for x, y in obstacles:
        grid_x, grid_y = map_to_grid(x, y)
        grid[grid_x:grid_x + 1, grid_y:grid_y + 1] = 1

    # Add origin point to grid
    origin_grid_x, origin_grid_y = map_to_grid(origin_x, origin_y)
    grid[origin_grid_x, origin_grid_y] = 3

    # Add player to grid
    player_grid_x, player_grid_y = map_to_grid(robot_x, robot_y)
    grid[player_grid_x:player_grid_x + 1, player_grid_y:player_grid_y + 1] = 2

    return grid


def update_robot_movement_grid(robot_x, robot_y, grid: np.ndarray, previous_robot_coords):
    """
    Update the grid when robot moves. Mark the robot's position in the grid.
    """
    # Get previous robot grid position
    prev_grid_x, prev_grid_y = map_to_grid(*next(iter(previous_robot_coords)))
    grid[prev_grid_x, prev_grid_y] = 0  # Clear previous robot position

    # Get current robot grid position
    new_grid_x, new_grid_y = map_to_grid(robot_x, robot_y)
    grid[new_grid_x, new_grid_y] = 2  # Set new robot position

    # Update robot coordinates
    new_robot_coords = {(robot_x, robot_y)}
    return grid, new_robot_coords


def update_grid_with_added_obstacles(grid: np.ndarray, obstacles):
    """
    Update the grid with added obstacles.
    """
    for x, y in obstacles:
        grid_x, grid_y = map_to_grid(x, y)
        grid[grid_x, grid_y] = 1  # Set obstacle position
    return grid
