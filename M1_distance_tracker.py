import numpy as np

def track_distance(robot_x, robot_y, origin_x, origin_y, obstacles, screen_width=800, screen_heigth=800):
    """
        Calculate and return an array of points representing the path of the robot to the origin point.
        Check colission with obstacles and return the path of the robot to the origin point surrounded by obstacles.
    """
    # Coords to track path
    coords = []

    # Calculate the distance between the robot and the origin point
    dx = origin_x - robot_x
    dy = origin_y - robot_y

    # Calculate the number of steps needed to reach the origin point
    steps = max(abs(dx), abs(dy))

    # Calculate the step size for each axis
    step_x = dx / steps
    step_y = dy / steps

    # Transform steps into integers
    steps = int(steps)

    # Generate the path of the robot to the origin point
    for i in range(steps):
        x = robot_x + i * step_x
        y = robot_y + i * step_y
        coords.append((x, y))

    # TODO: check colission with obstacles and apply some kind of algorithm to find best path possible

    return coords


def transform_screen_to_grid(robot_x, robot_y, origin_x, origin_y, obstacles) -> np.ndarray:
    """
        Transform the screen into a grid of 800x800 where:
            - Obstacles (1)
            - Player (2)
            - Origin point (3)
            - Empty space (0)
    """
    grid = np.zeros((800, 800))  

    # Add obstacles to grid
    for x, y in obstacles:
        for i in range(x, x + 25):
            for j in range(y, y + 25):
                grid[i, j] = 1

    # Add origin point to grid
    for i in range(int(origin_x) - 10, int(origin_x) + 10):
        for j in range(int(origin_y) - 10, int(origin_y) + 10):
            grid[i, j] = 3

    # Add player to grid
    for i in range(int(robot_x) - 25, int(robot_x) + 25):
        for j in range(int(robot_y) - 25, int(robot_y) + 25):
            grid[i, j] = 2

    return grid


def update_robot_movement_grid(robot_x, robot_y, grid: np.ndarray, previous_robot_coords):
    """
        Update the grid when robot moves. So 2's are introduced when robot moves
        and 0 when robot leaves a space.
    """
    # Get previous robot coords
    x_robot_prev, y_robot_prev = next(iter(previous_robot_coords))
    
    # Change 2's to 0's
    for i in range(x_robot_prev, x_robot_prev + 50):
        for j in range(y_robot_prev, y_robot_prev + 50):
            grid[i, j] = 0

    # Update robot position on grid
    for i in range(int(robot_x), int(robot_x) + 50):
        for j in range(int(robot_y) - 50, int(robot_y) + 50):
            grid[i, j] = 2

    # Update previous robot coords
    new_robot_coords = set()
    new_robot_coords.add((robot_x, robot_y))

    return grid, new_robot_coords


def find_best_path_to_origin_a_star(robot_x, robot_y, origin_x, origin_y, grid: np.ndarray,
                                    robot_width=50, robot_height=50, origin_width=20, origin_height=20):
    """
        Find the best path to the origin point using the A* algorithm.
    """
    # TODO: implement A* algorithm
