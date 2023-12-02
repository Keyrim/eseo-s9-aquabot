KEEPOUT_RADIUS = 10

def get_obstacles():
    obstacles = [
        [-157, 0, 35 + KEEPOUT_RADIUS],
        [-147, -20, 35 + KEEPOUT_RADIUS],

        [120, 130, 35 + KEEPOUT_RADIUS],
        [90, 150, 35 + KEEPOUT_RADIUS],
        [100, 170, 35 + KEEPOUT_RADIUS],
        [110, 150, 30 + KEEPOUT_RADIUS],
        
        [120, -50, 25 + KEEPOUT_RADIUS],
        [12, -102, 25 + KEEPOUT_RADIUS],
        [-92, 176, 30 + KEEPOUT_RADIUS],
        [-40, 220, 30 + KEEPOUT_RADIUS],
        [-44, -95, 30 + KEEPOUT_RADIUS],
        [-30, -150, 30 + KEEPOUT_RADIUS],
    ]
    return obstacles