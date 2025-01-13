def trackDistance(robot_x, robot_y, origin_x, origin_y):
    """
        Calculate and return an array of points representing the path of the robot to the origin point.
    """
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

    return coords