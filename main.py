import pygame
import random
from M1_distance_tracker import track_distance, transform_screen_to_grid, update_robot_movement_grid


# TODO: Create a singleton class to store the game data and avoid global variables
# class GameDataSingleton:
#     _instance = None

#     def __new__(cls, *args, **kwargs):
#         if not cls._instance:
#             cls._instance = super(GameDataSingleton, cls).__new__(cls, *args, **kwargs)
#             cls._instance.player_x = 0
#             cls._instance.player_y = 0
#             cls._instance.origin_x = 0
#             cls._instance.origin_y = 0
#             cls._instance.obstacles = []
#             cls._instance.grid = []
#         return cls._instance

pygame.init()

# Screen properties
WIDTH, HEIGHT = 800, 600
FPS = 60

# Colors (RGB)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

# Screen setup
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Tracker Game")

# Player properties
player_width = 50
player_height = 50
player_x = (WIDTH - player_width) // 2
player_y = HEIGHT - player_height - 10
player_speed = 5
player_coords = set()
player_coords.add((player_x, player_y))  # Initial position

# Origin point to track
origin_x = random.randint(0, WIDTH - player_width)
origin_y = random.randint(0, HEIGHT - player_height)
origin_width = 20
origin_height = 20

# Obstacles properties
obstacles = []
obstacle_width = 25
obstacle_height = 25
# Make random points
for i in range(5):
    x = random.randint(0, WIDTH - obstacle_width)
    y = random.randint(0, HEIGHT - obstacle_height)
    obstacles.append((x, y))

# Game loop
clock = pygame.time.Clock()

# Initialize grid system
grid = transform_screen_to_grid(player_x, player_y, origin_x, origin_y, obstacles)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Player movement and check colission with origin point 
    # Handle player movement
    keys = pygame.key.get_pressed()
    new_player_x = player_x
    new_player_y = player_y
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
        # Update player position and check collision with origin point and obstacles
        player_rect = pygame.Rect(new_player_x, new_player_y, player_width, player_height)
        origin_rect = pygame.Rect(origin_x, origin_y, origin_width, origin_height)
        obstacles_rect = [pygame.Rect(x, y, obstacle_width, obstacle_height) for x, y in obstacles]
        if not player_rect.colliderect(origin_rect) and not any(player_rect.colliderect(obstacle) for obstacle in obstacles_rect):
            player_x = new_player_x
            player_y = new_player_y
            # Update robot position on grid
            grid, player_coords = update_robot_movement_grid(new_player_x, new_player_y, grid, player_coords)

    # Screen drawing with white background
    screen.fill(WHITE)

    # Calculate the path to the origin point
    coords = track_distance(player_x + (player_width / 2 ), player_y + (player_height) / 2, origin_x + ( origin_width / 2), origin_y + (origin_height / 2), obstacles)

    # Draw path
    for x, y in coords:
        pygame.draw.rect(screen, BLACK, (x, y, 2, 2))

    # Draw player
    pygame.draw.rect(screen, RED,
                     (player_x, player_y, player_width, player_height))

    # Draw origin point
    pygame.draw.rect(screen, BLACK,
                     (origin_x, origin_y, origin_width, origin_height))

    # Draw obstacles
    for x, y in obstacles:
        pygame.draw.rect(screen, BLACK, (x, y, obstacle_width, obstacle_height))

    # Draw fps counter on screen
    font = pygame.font.Font(None, 36)
    fps = font.render(f"{int(clock.get_fps())} FPS", True, BLACK)
    screen.blit(fps, (10, 10))

    pygame.display.update()

    clock.tick(FPS)

pygame.quit()
