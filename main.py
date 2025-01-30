import pygame
import random
from M1_distance_tracker import track_distance, transform_screen_to_grid, update_robot_movement_grid, update_grid_with_added_obstacles
from M2_a_star_algorithm_functions import find_path
import numpy as np

# Initialize Pygame
pygame.init()

# Screen properties
WIDTH, HEIGHT = 800, 800
FPS = 60

# Colors (RGB)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
CLEAR_GREEN = (127, 250, 127)
BLACK = (0, 0, 0)
GREY = (200, 200, 200)

# Grid properties
GRID_SIZE = 40
CELL_SIZE = WIDTH // GRID_SIZE

# Player properties
# player_width, player_height = CELL_SIZE * 4, CELL_SIZE * 4
player_width, player_height = CELL_SIZE, CELL_SIZE
player_x = (WIDTH - player_width) // 2
player_y = HEIGHT - player_height - 10
player_speed = 3
player_coords = set()
player_coords.add((player_x, player_y))

# Origin point to track
# origin_x = random.randint(0, GRID_SIZE - 1) * CELL_SIZE
# origin_y = random.randint(0, GRID_SIZE - 1) * CELL_SIZE
origin_x, origin_y = 600, 400
origin_width, origin_height = CELL_SIZE, CELL_SIZE

print(origin_x, origin_y)

# Obstacles properties
obstacles = []
obstacle_width, obstacle_height = CELL_SIZE, CELL_SIZE
# Generate obstacles in front of the origin point

for i in range(300):
    x = 250 + i
    y = 500
    obstacles.append((x, y))

for i in range(300):
    x = 375 + i
    y = 600
    obstacles.append((x, y))


# Initialize grid system
grid = transform_screen_to_grid(player_x, player_y, origin_x, origin_y, obstacles)

import sys
import numpy
numpy.set_printoptions(threshold=sys.maxsize)

# Find path to origin point and heat map
path, heat_map = find_path(grid, (player_x // CELL_SIZE, player_y // CELL_SIZE), (origin_x // CELL_SIZE, origin_y // CELL_SIZE))

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
            path, heat_map = find_path(grid, (player_x // CELL_SIZE, player_y // CELL_SIZE), (origin_x // CELL_SIZE, origin_y // CELL_SIZE))

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

        # Find path to origin only when player moves (measure time to find path)
        start_time = pygame.time.get_ticks()
        path, heat_map = find_path(grid, (player_x // CELL_SIZE, player_y // CELL_SIZE), (origin_x // CELL_SIZE, origin_y // CELL_SIZE))
        measure_time = pygame.time.get_ticks() - start_time

        # Add this time into a log file
        with open("log.txt", "a") as file:
            file.write(f"{measure_time} ms\n")

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

    # Normalize the heat map
    normalized_heat_map = normalize_heat_map(heat_map)

    # Draw A* heat map path (numpy array)
    heat_map_surface.fill((0, 0, 0, 0))  # Clear the heat map surface
    for x in range(heat_map.shape[0]):
        for y in range(heat_map.shape[1]):
            if normalized_heat_map[x][y] != 0:
                alpha = int(normalized_heat_map[x][y])
                pygame.draw.rect(heat_map_surface, (255, 0, 0, alpha), (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Draw A* path
    for x, y in path:
        pygame.draw.rect(screen, CLEAR_GREEN, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Blit the surface with the heat map onto the screen
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

    pygame.display.update()
    clock.tick(FPS)

pygame.quit()
