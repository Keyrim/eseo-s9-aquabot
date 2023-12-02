KEEPOUT_RADIUS = 10

def get_obstacles():
    obstacles = [
        [120, -50, 25 + KEEPOUT_RADIUS],
        [-152, -6, 50 + KEEPOUT_RADIUS],
        [110, 130, 50 + KEEPOUT_RADIUS],
        [12, -102, 25 + KEEPOUT_RADIUS],
        [92, 170, 25 + KEEPOUT_RADIUS],
        [-92, 176, 30 + KEEPOUT_RADIUS],
        [-40, 220, 30 + KEEPOUT_RADIUS],
        [-44, -95, 30 + KEEPOUT_RADIUS],
        [-30, -150, 30 + KEEPOUT_RADIUS],
    ]
    return obstacles