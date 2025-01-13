import pygame
import random
from M1_distance_tracker import trackDistance

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

# Origin point to track
origin_x = random.randint(0, WIDTH - player_width)
origin_y = random.randint(0, HEIGHT - player_height)
origin_width = 20
origin_height = 20

# Game loop
clock = pygame.time.Clock()

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

    if keys[pygame.K_LEFT] and player_x > 0:
        new_player_x -= player_speed
    if keys[pygame.K_RIGHT] and player_x < WIDTH - player_width:
        new_player_x += player_speed
    if keys[pygame.K_UP] and player_y > 0:
        new_player_y -= player_speed
    if keys[pygame.K_DOWN] and player_y < HEIGHT - player_height:
        new_player_y += player_speed

    # Update player position and check collision
    player_rect = pygame.Rect(new_player_x, new_player_y, player_width, player_height)
    origin_rect = pygame.Rect(origin_x, origin_y, origin_width, origin_height)
    if not player_rect.colliderect(origin_rect):
        player_x = new_player_x
        player_y = new_player_y

    # Screen drawing with white background
    screen.fill(WHITE)

    # Calculate the path to the origin point
    coords = trackDistance(player_x + (player_width / 2 ), player_y + (player_height) / 2, origin_x + ( origin_width / 2), origin_y + (origin_height / 2))

    # Draw path
    for x, y in coords:
        pygame.draw.rect(screen, BLACK, (x, y, 2, 2))

    # Draw player
    pygame.draw.rect(screen, RED,
                     (player_x, player_y, player_width, player_height))

    # Draw origin point
    pygame.draw.rect(screen, BLACK,
                     (origin_x, origin_y, origin_width, origin_height))


    pygame.display.update()

    clock.tick(FPS)

pygame.quit()
