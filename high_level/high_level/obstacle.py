class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

KEEPOUT_RADIUS = 10

def get_obstacles():
    obstacles = [
        Obstacle(-120, -50, 25 + KEEPOUT_RADIUS),
        Obstacle(-152, -6, 50 + KEEPOUT_RADIUS),
        Obstacle(110, 130, 50 + KEEPOUT_RADIUS),
        Obstacle(12, -102, 25 + KEEPOUT_RADIUS),
        Obstacle(92, 170, 25 + KEEPOUT_RADIUS),
        Obstacle(-92, 176, 30 + KEEPOUT_RADIUS),
        Obstacle(-40, 220, 30 + KEEPOUT_RADIUS),
        Obstacle(-44, -95, 30 + KEEPOUT_RADIUS),
        Obstacle(-30, -150, 30 + KEEPOUT_RADIUS),
    ]
    return obstacles