import car
import math

car.theta = math.radians(15)
car.x = 500
car.y = 499
car.update_rays()
print(car.first_intersects)
print(car.ray_lengths)