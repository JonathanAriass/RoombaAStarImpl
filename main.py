import pygame
import random
import time
from M1_distance_tracker import track_distance, transform_screen_to_grid, update_robot_movement_grid, update_grid_with_added_obstacles
from M2_a_star_algorithm_functions import find_path
from M3_hpa_functions import HPAStar
import numpy as np

# Initialize Pygame
pygame.init()

# Screen properties
WIDTH, HEIGHT = 800, 800  # Larger screen for bigger maps
FPS = 60

# Colors (RGB)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
CLEAR_GREEN = (127, 250, 127)
BLACK = (0, 0, 0)
GREY = (200, 200, 200)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
PURPLE = (128, 0, 128)
ORANGE = (255, 140, 0)
MAGENTA = (255, 0, 255)

# Debug visualization for HPA*
HPA_DEBUG = True  # Enable to see clusters and entry points

# Grid properties
GRID_SIZE = 50  # Larger grid for testing
CELL_SIZE = WIDTH // GRID_SIZE

# Pathfinding mode: 'astar' or 'hpa'
PATHFINDING_MODE = 'hpa'
HPA_CLUSTER_SIZE = 5  # Size of clusters for HPA*

# Player properties
player_width, player_height = CELL_SIZE, CELL_SIZE
player_x = 5 * CELL_SIZE  # Start at column 5
player_y = 45 * CELL_SIZE  # Start at row 45 (bottom area)
player_speed = CELL_SIZE # Move one cell at a time for easier testing
player_coords = set()
player_coords.add((player_x, player_y))

# Origin point to track (top-right area)
origin_x, origin_y = 45 * CELL_SIZE, 5 * CELL_SIZE
origin_width, origin_height = CELL_SIZE, CELL_SIZE


# Obstacles properties - simple horizontal wall with a gap
obstacles = []
obstacle_width, obstacle_height = CELL_SIZE, CELL_SIZE

# Create a simple horizontal wall at row 25, with a gap at column 35
for col in range(0, 34):  # Wall from col 0 to 33
    obstacles.append((col * CELL_SIZE, 25 * CELL_SIZE))

# Leave gap at columns 34-36, then continue wall
for col in range(37, 50):  # Wall from col 37 to 49
    obstacles.append((col * CELL_SIZE, 25 * CELL_SIZE))


# Initialize grid system
grid = transform_screen_to_grid(player_x, player_y, origin_x, origin_y, obstacles)

def find_path_with_mode(grid, start, goal, hpa_instance=None):
    """Find path using either A* or HPA* based on PATHFINDING_MODE."""
    if PATHFINDING_MODE == 'hpa' and hpa_instance is not None:
        start_time = time.time()
        # Convert (x, y) to (row, col) for HPA*
        hpa_start = (start[1], start[0])
        hpa_goal = (goal[1], goal[0])

        path = hpa_instance.search(hpa_start, hpa_goal, refine=True)

        elapsed_ms = (time.time() - start_time) * 1000

        # Fallback to regular A* if HPA* fails
        if not path:
            return find_path(grid, start, goal)

        # Convert path back from (row, col) to (x, y)
        path = [(p[1], p[0]) for p in path]

        # HPA* doesn't track heat map or expanded nodes the same way
        heat_map = np.zeros((GRID_SIZE, GRID_SIZE))
        expanded_nodes = len(hpa_instance.entry_points)
        return path, heat_map, expanded_nodes, elapsed_ms
    else:
        return find_path(grid, start, goal)


def initialize_hpa(grid, start, goal):
    """Initialize or reinitialize HPA* with current grid state."""
    # HPA* expects (row, col) format, our grid uses (x, y) which is (col, row)
    # Convert coordinates: grid position (x, y) -> HPA position (y, x)
    hpa_start = (start[1], start[0])
    hpa_goal = (goal[1], goal[0])

    return HPAStar(grid.T, hpa_start, hpa_goal, cluster_size=HPA_CLUSTER_SIZE)


# Initialize HPA* if using HPA mode
hpa_instance = None
start_pos = (player_x // CELL_SIZE, player_y // CELL_SIZE)
goal_pos = (origin_x // CELL_SIZE, origin_y // CELL_SIZE)
last_grid_pos = start_pos  # Track last grid cell to avoid unnecessary recalculations

if PATHFINDING_MODE == 'hpa':
    hpa_instance = initialize_hpa(grid, start_pos, goal_pos)

# Find path to origin point and heat map
path, heat_map, expanded_nodes, measure_time = find_path_with_mode(
    grid, start_pos, goal_pos, hpa_instance
)

# Game loop
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Tracker Game")
clock = pygame.time.Clock()

# Create a surface for the heat map
heat_map_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
alpha_value = 128  # Adjust this value as needed for desired transparency
heat_map_surface.set_alpha(alpha_value)


def normalize_heat_map(heat_map):
    """
        Normalize the heat map values to a range between 0 and 255.

        Args:
            heat_map: Heat map to normalize

        Returns:
            Normalized heat map
    """
    if np.max(heat_map) == 0:
        return heat_map
    return (heat_map / np.max(heat_map)) * 255


# Create a surface for the adding obstacles by clicking and hovering over the grid
obstacles_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
obstacles_surface.set_alpha(alpha_value)
temporal_added_obstacles = []
is_adding_obstacle = False

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            is_adding_obstacle = True
        if event.type == pygame.MOUSEBUTTONUP:
            is_adding_obstacle = False
            obstacles_surface.fill((0, 0, 0, 0))  # Clear the obstacles surface
            obstacles.extend(temporal_added_obstacles)
            grid = update_grid_with_added_obstacles(grid, obstacles)
            temporal_added_obstacles = []
            # Reinitialize HPA* when obstacles change (requires rebuilding abstract graph)
            start_pos = (player_x // CELL_SIZE, player_y // CELL_SIZE)
            goal_pos = (origin_x // CELL_SIZE, origin_y // CELL_SIZE)
            if PATHFINDING_MODE == 'hpa':
                hpa_instance = initialize_hpa(grid, start_pos, goal_pos)
            path, heat_map, expanded_nodes, measure_time = find_path_with_mode(
                grid, start_pos, goal_pos, hpa_instance
            )

    # Handle player movement
    keys = pygame.key.get_pressed()
    new_player_x, new_player_y = player_x, player_y
    key_pressed = False

    if keys[pygame.K_LEFT] and player_x > 0:
        new_player_x -= player_speed
        key_pressed = True
    if keys[pygame.K_RIGHT] and player_x < WIDTH - player_width:
        new_player_x += player_speed
        key_pressed = True
    if keys[pygame.K_UP] and player_y > 0:
        new_player_y -= player_speed
        key_pressed = True
    if keys[pygame.K_DOWN] and player_y < HEIGHT - player_height:
        new_player_y += player_speed
        key_pressed = True

    if key_pressed:
        # Update player position and check collisions
        player_rect = pygame.Rect(new_player_x, new_player_y, player_width, player_height)
        origin_rect = pygame.Rect(origin_x, origin_y, origin_width, origin_height)
        obstacles_rect = [pygame.Rect(x, y, obstacle_width, obstacle_height) for x, y in obstacles]

        if not player_rect.colliderect(origin_rect) and not any(player_rect.colliderect(obstacle) for obstacle in obstacles_rect):
            player_x, player_y = new_player_x, new_player_y
            grid, player_coords = update_robot_movement_grid(new_player_x, new_player_y, grid, player_coords)

        # Only recalculate path when player changes grid cell
        current_grid_pos = (player_x // CELL_SIZE, player_y // CELL_SIZE)
        if current_grid_pos != last_grid_pos:
            last_grid_pos = current_grid_pos
            start_pos = current_grid_pos
            goal_pos = (origin_x // CELL_SIZE, origin_y // CELL_SIZE)
            # For HPA*, rebuild graph only when grid cell changes
            if PATHFINDING_MODE == 'hpa':
                hpa_instance = initialize_hpa(grid, start_pos, goal_pos)
            path, heat_map, expanded_nodes, measure_time = find_path_with_mode(
                grid, start_pos, goal_pos, hpa_instance
            )

    # Screen drawing
    screen.fill(WHITE)

    # Draw grid
    for x in range(0, WIDTH, CELL_SIZE):
        pygame.draw.line(screen, GREY, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, CELL_SIZE):
        pygame.draw.line(screen, GREY, (0, y), (WIDTH, y))

    # Calculate path to the origin
    coords = track_distance(player_x + (player_width / 2), player_y + (player_height / 2),
                            origin_x + (origin_width / 2), origin_y + (origin_height / 2))

    # Draw path
    for x, y in coords:
        pygame.draw.rect(screen, BLACK, (x, y, 2, 2))

    # Draw obstacles
    for x, y in obstacles:
        pygame.draw.rect(screen, BLACK, (x, y, obstacle_width, obstacle_height))

    # Draw heat map only for A* mode (HPA* doesn't generate heat map)
    if PATHFINDING_MODE != 'hpa':
        normalized_heat_map = normalize_heat_map(heat_map)
        heat_map_surface.fill((0, 0, 0, 0))  # Clear the heat map surface
        for x in range(heat_map.shape[0]):
            for y in range(heat_map.shape[1]):
                if normalized_heat_map[x][y] != 0:
                    alpha = int(normalized_heat_map[x][y])
                    pygame.draw.rect(heat_map_surface, (255, 0, 0, alpha), (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Debug visualization for HPA*
    if HPA_DEBUG and PATHFINDING_MODE == 'hpa' and hpa_instance is not None:
        # Draw cluster boundaries (thick blue lines)
        for cluster_row in range(0, GRID_SIZE, HPA_CLUSTER_SIZE):
            pygame.draw.line(screen, BLUE, (0, cluster_row * CELL_SIZE), (WIDTH, cluster_row * CELL_SIZE), 2)
        for cluster_col in range(0, GRID_SIZE, HPA_CLUSTER_SIZE):
            pygame.draw.line(screen, BLUE, (cluster_col * CELL_SIZE, 0), (cluster_col * CELL_SIZE, HEIGHT), 2)

        # Draw abstract graph edges (cyan lines)
        for node, neighbors in hpa_instance.graph.items():
            # Convert from (row, col) to screen coordinates
            node_screen_x = node[1] * CELL_SIZE + CELL_SIZE // 2
            node_screen_y = node[0] * CELL_SIZE + CELL_SIZE // 2
            for cost, neighbor in neighbors:
                neighbor_screen_x = neighbor[1] * CELL_SIZE + CELL_SIZE // 2
                neighbor_screen_y = neighbor[0] * CELL_SIZE + CELL_SIZE // 2
                pygame.draw.line(screen, CYAN, (node_screen_x, node_screen_y),
                               (neighbor_screen_x, neighbor_screen_y), 1)

        # Draw entry points (yellow circles)
        for entry_point in hpa_instance.entry_points:
            # Convert from (row, col) to screen coordinates
            ep_screen_x = entry_point[1] * CELL_SIZE + CELL_SIZE // 2
            ep_screen_y = entry_point[0] * CELL_SIZE + CELL_SIZE // 2
            pygame.draw.circle(screen, YELLOW, (ep_screen_x, ep_screen_y), 4)

        # Draw start and goal with different colors
        start_screen_x = hpa_instance.start_node[1] * CELL_SIZE + CELL_SIZE // 2
        start_screen_y = hpa_instance.start_node[0] * CELL_SIZE + CELL_SIZE // 2
        pygame.draw.circle(screen, RED, (start_screen_x, start_screen_y), 6)

        goal_screen_x = hpa_instance.goal_node[1] * CELL_SIZE + CELL_SIZE // 2
        goal_screen_y = hpa_instance.goal_node[0] * CELL_SIZE + CELL_SIZE // 2
        pygame.draw.circle(screen, GREEN, (goal_screen_x, goal_screen_y), 6)

    # Draw refined path (always visible, regardless of debug mode)
    if path and len(path) > 0:
        # Draw thick magenta line connecting path points
        if len(path) > 1:
            path_points = [(x * CELL_SIZE + CELL_SIZE // 2, y * CELL_SIZE + CELL_SIZE // 2) for x, y in path]
            pygame.draw.lines(screen, MAGENTA, False, path_points, 6)
        # Draw orange circles at each path node
        for x, y in path:
            center_x = x * CELL_SIZE + CELL_SIZE // 2
            center_y = y * CELL_SIZE + CELL_SIZE // 2
            pygame.draw.circle(screen, ORANGE, (center_x, center_y), 6)

    # Blit the surface with the heat map onto the screen (A* mode only)
    if PATHFINDING_MODE != 'hpa':
        screen.blit(heat_map_surface, (0, 0))

    # Draw player
    pygame.draw.rect(screen, RED, (player_x, player_y, player_width, player_height))
    font = pygame.font.Font(None, 16)
    text = font.render(f"({player_x // CELL_SIZE}, {player_y // CELL_SIZE})", True, BLACK)
    screen.blit(text, (player_x, player_y + 20))

    # Draw origin point
    pygame.draw.rect(screen, GREEN, (origin_x, origin_y, origin_width, origin_height))
    font = pygame.font.Font(None, 16)
    text = font.render(f"Origin({origin_x // CELL_SIZE}, {origin_y // CELL_SIZE})", True, BLACK)
    screen.blit(text, (origin_x, origin_y - 20))

    # Draw obstacles surface (for adding obstacles)
    if is_adding_obstacle:
        mouse_x, mouse_y = pygame.mouse.get_pos()
        grid_x = mouse_x // CELL_SIZE * CELL_SIZE
        grid_y = mouse_y // CELL_SIZE * CELL_SIZE
        pygame.draw.rect(obstacles_surface, BLACK, (grid_x, grid_y, CELL_SIZE, CELL_SIZE))
        screen.blit(obstacles_surface, (0, 0))
        temporal_added_obstacles.append((grid_x, grid_y))

    # Draw FPS counter
    font = pygame.font.Font(None, 36)
    fps = font.render(f"{int(clock.get_fps())} FPS", True, BLACK)
    screen.blit(fps, (10, 10))

    # Draw nodes expanded, path cost, and time to find path
    font = pygame.font.Font(None, 24)
    mode_label = "HPA*" if PATHFINDING_MODE == 'hpa' else "A*"
    text = font.render(f"Mode: {mode_label} (cluster size: {HPA_CLUSTER_SIZE})" if PATHFINDING_MODE == 'hpa' else f"Mode: {mode_label}", True, BLACK)
    screen.blit(text, (10, 40))
    text = font.render(f"Entry points: {expanded_nodes}" if PATHFINDING_MODE == 'hpa' else f"Nodes expanded: {expanded_nodes}", True, BLACK)
    screen.blit(text, (10, 70))
    text = font.render(f"Path length: {len(path)} cells", True, BLACK)
    screen.blit(text, (10, 100))
    text = font.render(f"Time to find path: {measure_time:.2f} ms", True, BLACK)
    screen.blit(text, (10, 130))

    # Draw HPA* debug legend
    if HPA_DEBUG and PATHFINDING_MODE == 'hpa':
        font_small = pygame.font.Font(None, 20)
        pygame.draw.rect(screen, BLUE, (10, 160, 12, 12))
        screen.blit(font_small.render("Cluster boundaries", True, BLACK), (26, 160))
        pygame.draw.circle(screen, YELLOW, (16, 185), 5)
        screen.blit(font_small.render("Entry points", True, BLACK), (26, 180))
        pygame.draw.line(screen, CYAN, (10, 200), (22, 200), 2)
        screen.blit(font_small.render("Abstract graph edges", True, BLACK), (26, 195))
        pygame.draw.line(screen, MAGENTA, (10, 215), (22, 215), 3)
        pygame.draw.circle(screen, ORANGE, (16, 215), 4)
        screen.blit(font_small.render("Refined path", True, BLACK), (26, 210))

    pygame.display.update()
    clock.tick(FPS)

pygame.quit()
