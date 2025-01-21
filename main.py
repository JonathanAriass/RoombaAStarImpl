import pygame
import random
from M1_distance_tracker import track_distance, transform_screen_to_grid, update_robot_movement_grid
from M2_a_star_algorithm_functions import find_path

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
    y = 550
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

# print(grid)
path = find_path(grid, (player_x // CELL_SIZE, player_y // CELL_SIZE), (origin_x // CELL_SIZE, origin_y // CELL_SIZE))

# Game loop
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Tracker Game")
clock = pygame.time.Clock()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

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

    # Draw player
    pygame.draw.rect(screen, RED, (player_x, player_y, player_width, player_height))
    font = pygame.font.Font(None, 16)
    text = font.render(f"({player_x // CELL_SIZE}, {player_y // CELL_SIZE})", True, BLACK)
    screen.blit(text, (player_x, player_y + 20))

    # Draw origin point
    pygame.draw.rect(screen, GREEN, (origin_x, origin_y, origin_width, origin_height))

    # Draw obstacles
    for x, y in obstacles:
        pygame.draw.rect(screen, BLACK, (x, y, obstacle_width, obstacle_height))

    # Draw A* path
    for x, y in path:
        pygame.draw.rect(screen, CLEAR_GREEN, (x * CELL_SIZE + (player_width / 2), y * CELL_SIZE + (player_width / 2), CELL_SIZE, CELL_SIZE))

    # Draw FPS counter
    font = pygame.font.Font(None, 36)
    fps = font.render(f"{int(clock.get_fps())} FPS", True, BLACK)
    screen.blit(fps, (10, 10))

    path = find_path(grid, (player_x // CELL_SIZE, player_y // CELL_SIZE), (origin_x // CELL_SIZE, origin_y // CELL_SIZE))

    pygame.display.update()
    clock.tick(FPS)

    # path = find_path(grid, (player_x // CELL_SIZE, player_y // CELL_SIZE), (origin_x // CELL_SIZE, origin_y // CELL_SIZE))

pygame.quit()
