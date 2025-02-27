import pygame
import math

pygame.init()

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Car in Motion")

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

L, W = 60, 30
x, y = WIDTH // 2, HEIGHT // 2
d = 50
theta = 0
velocity = 200
steering_angle = math.radians(30)
dt = 1/60

def update():
    global x, y, theta, steering_angle, d, dt

    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        steering_angle = math.radians(30)
    if keys[pygame.K_RIGHT]:
        steering_angle = math.radians(-30)
    if not keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
        steering_angle = 0
    
    x += -dt * velocity * math.sin(theta)
    y += -dt * velocity * math.cos(theta)
    theta += (dt * velocity * math.tan(steering_angle))/d
    if (theta > 2*math.pi):
        theta -= 2*math.pi
    if (theta < 0):
        theta += 2*math.pi

# Main loop
running = True
clock = pygame.time.Clock()

while running:
    clock.tick(60)
    screen.fill(WHITE)

    update()

    car_surface = pygame.Surface((W, L), pygame.SRCALPHA)
    car_surface.fill(BLACK)
    front_width = 10
    front_rect = pygame.Rect(0, 0, W, front_width)
    pygame.draw.rect(car_surface, RED, front_rect)

    rotated_car = pygame.transform.rotate(car_surface, math.degrees(theta))
    rotated_rect = rotated_car.get_rect(center=(x, y))

    screen.blit(rotated_car, rotated_rect.topleft)

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()