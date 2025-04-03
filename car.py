import pygame
import math
import numpy as np

from tracks import training_track, WIDTH, HEIGHT

track = training_track

L, W = 60, 30
d = 50
velocity = 200
dt = 1/60

# initial state
x, y = WIDTH // 2, HEIGHT // 2
theta = 0
steering_angle = 0

# sensor information:
ray_angles = [math.radians(60), math.radians(30), 0, math.radians(-30), math.radians(-60)]
source1 = None
first_intersects = [0] * len(ray_angles)
ray_lengths = [0] * len(ray_angles)    

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
    
    update_rays()

def update_rays():
    global x, y, L, theta, source1, ray_angles, first_intersects, ray_lengths, track

    source1 = (x - (L//2)*math.sin(theta), y - (L//2)*math.cos(theta))
    for i in range(len(ray_angles)):
        ray_angle = ray_angles[i]
        v_1 = (-math.sin(theta+ray_angle), -math.cos(theta+ray_angle))

        first_intersection_length = math.inf
        first_intersection_point = None

        for line in track:
            v_2 = (line[2]-line[0], line[3]-line[1])
            try:
                line_params = np.linalg.inv(np.array([[v_1[0], -v_2[0]], [v_1[1], -v_2[1]]])) @ (np.array([line[0], line[1]]) - np.array(source1))
            except np.linalg.LinAlgError:
                # if the matrix is invertible then the lines don't intersect
                continue
            sensor_ray_param = line_params[0]
            track_line_param = line_params[1]
            # check if there is an intersection by checking if the line params are both within bounds
            # and if the intersection is shorter than the current one, update it. Note that
            # the intersection length is just the sensor_ray_param because v_1 is a unit vector
            if 0 <= sensor_ray_param and 0 <= track_line_param and track_line_param <= 1 and sensor_ray_param <= first_intersection_length:
                first_intersection_length = sensor_ray_param
                first_intersection_point = (source1[0] + sensor_ray_param*v_1[0], source1[1] + sensor_ray_param*v_1[1])
        
        first_intersects[i] = first_intersection_point
        ray_lengths[i] = first_intersection_length


def collision():
    global x, y, L, W, theta, track

    # this is a ccw rotation matrix by theta in a coordinate frame where i points to the right and j points up
    # since the frame of the pygame canvas has j pointing down, we have to use -y for the offset. We're essentially
    # going to do everything in the j pointing up frame
    Rot = np.array([[math.cos(theta), -math.sin(theta), x],[math.sin(theta), math.cos(theta), -y]])
    # we then convert everything back to the j pointing down frame by left-multiplying by [[1,0],[0,-1]] and flatten them
    # into row vectors
    top_left = (np.array([[1, 0],[0, -1]]) @ Rot @ np.array([[-W/2], [L/2], [1]])).flatten()
    top_right = (np.array([[1, 0],[0, -1]]) @ Rot @ np.array([[W/2], [L/2], [1]])).flatten()
    bot_left = (np.array([[1, 0],[0, -1]]) @ Rot @ np.array([[-W/2], [-L/2], [1]])).flatten()
    bot_right = (np.array([[1, 0],[0, -1]]) @ Rot @ np.array([[W/2], [-L/2], [1]])).flatten()

    for car_boundary in [(top_left, bot_left), (top_right, top_left), (bot_right, top_right), (bot_left, bot_right)]:
        v_1unnorm = car_boundary[1] - car_boundary[0]
        v_1norm = np.linalg.norm(v_1unnorm)
        v_1 = v_1unnorm / v_1norm
        for line in track:
            v_2 = (line[2]-line[0], line[3]-line[1])
            try:
                line_params = np.linalg.inv(np.array([[v_1[0], -v_2[0]], [v_1[1], -v_2[1]]])) @ (np.array([line[0], line[1]]) - car_boundary[0])
            except np.linalg.LinAlgError:
                # if the matrix is invertible then the lines don't intersect
                continue
            if 0 <= line_params[0] and line_params[0] <= v_1norm and 0 <= line_params[1] and line_params[1] <= 1:
                return line
            
    return False

# this is the reward function. Runs a single simulation and returns the number of ticks the car drives without crashing
# where the car starts from a fixed initial state
def simulate(model_params, display=True):

    global x, y, theta, WIDTH, HEIGHT
    
    if display:
        WHITE = (255, 255, 255)
        RED = (255, 0, 0)
        BLACK = (0, 0, 0)

        pygame.init()

        screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("2D Car in Motion")

        running = True
        clock = pygame.time.Clock()

    x = WIDTH / 2
    y = 4*HEIGHT/5
    num_ticks = 0
    running = True
    while running:
        num_ticks += 1
        update()
    
        if display:
            clock.tick(60)
            screen.fill(WHITE)

            car_surface = pygame.Surface((W, L), pygame.SRCALPHA)
            car_surface.fill(BLACK)
            front_width = 10
            front_rect = pygame.Rect(0, 0, W, front_width)
            pygame.draw.rect(car_surface, RED, front_rect)

            rotated_car = pygame.transform.rotate(car_surface, math.degrees(theta))
            rotated_rect = rotated_car.get_rect(center=(x, y))

            pygame.draw.circle(screen, BLACK, source1, 3)

            screen.blit(rotated_car, rotated_rect.topleft)

            # draw sensor rays
            for first_intersect in first_intersects:
                pygame.draw.line(screen, RED, source1, first_intersect, 3)
            
            # draw tracks
            for wall in track:
                pygame.draw.line(screen, BLACK, (wall[0], wall[1]), (wall[2], wall[3]), 1)

            pygame.display.flip()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
        
        if collision():
            running = False
    
    return num_ticks

print(simulate(None, True))
    



# Main loop
# WHITE = (255, 255, 255)
# RED = (255, 0, 0)
# BLACK = (0, 0, 0)
# BLUE = (0, 0, 255)

# pygame.init()

# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("2D Car in Motion")

# running = True
# clock = pygame.time.Clock()

# while running:
#     clock.tick(60)
#     screen.fill(WHITE)

#     update()
#     if collision():
#         break

#     car_surface = pygame.Surface((W, L), pygame.SRCALPHA)
#     car_surface.fill(BLACK)
#     front_width = 10
#     front_rect = pygame.Rect(0, 0, W, front_width)
#     pygame.draw.rect(car_surface, RED, front_rect)

#     rotated_car = pygame.transform.rotate(car_surface, math.degrees(theta))
#     rotated_rect = rotated_car.get_rect(center=(x, y))

#     pygame.draw.circle(screen, BLACK, source1, 3)

#     screen.blit(rotated_car, rotated_rect.topleft)

#     # draw sensor rays
#     for first_intersect in first_intersects:
#         pygame.draw.line(screen, RED, source1, first_intersect, 3)
    
#     # draw tracks
#     for wall in track:
#         pygame.draw.line(screen, BLACK, (wall[0], wall[1]), (wall[2], wall[3]), 1)

#     pygame.display.flip()

#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

# pygame.quit()