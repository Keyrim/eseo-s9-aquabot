import heapq
from matplotlib import pyplot as plt

class PathFinder:

    GRID_SIZE = 20

    def __init__(self, fix_obstacles = []):
        self.current_pos = (0, 0)
        self.fix_obstacles = fix_obstacles  # List of obstacles, each represented as (x, y, radius)
        self.moveable_obstacles = []  # List of obstacles, each represented as (x, y, radius)

    def set_position(self, x, y):
        """Set the current position."""
        self.current_pos = self.get_scaled_down(x, y)

    def get_scaled_down(self, x, y):
        """Scale down the coordinates to the grid."""
        return x // self.GRID_SIZE, y // self.GRID_SIZE

    def get_scaled_up(self, x, y):
        """Scale up the coordinates from the grid."""
        return x * self.GRID_SIZE, y * self.GRID_SIZE

    def add_moveable_obstacle(self, x, y, radius):
        """Add an obstacle represented by its center (x, y) and radius."""
        x = x // self.GRID_SIZE
        y = y // self.GRID_SIZE
        radius = radius // self.GRID_SIZE
        self.moveable_obstacles.append((x, y, radius))

    def delete_all_moveable_obstacle(self):
        self.moveable_obstacles = []

    def add_list_fix_obstacle(self, list_obstacle):
        for obstacle in list_obstacle:
            self.add_fix_obstacle(obstacle.x, obstacle.y, obstacle.radius)

    def add_fix_obstacle(self, x, y, radius):
        """Add an obstacle represented by its center (x, y) and radius."""
        x = x // self.GRID_SIZE
        y = y // self.GRID_SIZE
        radius = radius // self.GRID_SIZE
        self.fix_obstacles.append((x, y, radius))

    def find_path(self, target_pos):
        """Find a path to the target position, avoiding obstacles using A* algorithm."""
        target_x, target_y = target_pos
        target_pos = self.get_scaled_down(target_x, target_y)
        obstacles = self.fix_obstacles + self.moveable_obstacles
        print(obstacles)
        astar = AStar(self.current_pos, target_pos, obstacles)
        points =  astar.search()
        # Scale up the points from the grid
        return [self.get_scaled_up(x, y) for x, y in points]

class AStar:
    def __init__(self, start, goal, obstacles):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.open_set = []
        heapq.heappush(self.open_set, (0, start))
        self.came_from = {}
        self.g_score = {start: 0}
        self.f_score = {start: self.heuristic(start, goal)}

    def heuristic(self, current, goal):
        """Simple heuristic to estimate the distance from the current point to the goal."""
        return ((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2) ** 0.5

    def is_collision(self, point):
        """Check if a point is within any of the obstacles."""
        for obstacle in self.obstacles:
            ox, oy, radius = obstacle
            if (point[0] - ox) ** 2 + (point[1] - oy) ** 2 < radius ** 2:
                return True
        return False

    def neighbors(self, current):
        """Generate neighbors of the current point. This needs to be adapted based on your environment."""
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.is_collision(neighbor):
                    yield neighbor

    def reconstruct_path(self):
        """Reconstruct the path from the start to the goal."""
        total_path = [self.goal]
        while total_path[-1] in self.came_from:
            total_path.append(self.came_from[total_path[-1]])
        return total_path[::-1]

    def search(self):
        """Search for a path using the A* algorithm."""
        while self.open_set:
            current = heapq.heappop(self.open_set)[1]

            if current == self.goal:
                return self.reconstruct_path()

            for neighbor in self.neighbors(current):
                tentative_g_score = self.g_score[current] + 1  # Assumes a distance of 1 between neighbors
                if neighbor not in self.g_score or tentative_g_score < self.g_score[neighbor]:
                    self.came_from[neighbor] = current
                    self.g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(neighbor, self.goal)
                    if neighbor not in [i[1] for i in self.open_set]:
                        heapq.heappush(self.open_set, (f_score, neighbor))
                    self.f_score[neighbor] = f_score

        return []  # Path not found

# Example usage
if __name__ == '__main__':
    path_finder = PathFinder()
    path_finder.set_position(0, 0)  # Set current position
    path_finder.add_fix_obstacle(200, 200, 300)  # Add an obstacle
    path_finder.add_fix_obstacle(300, 600, 300)  # Add an obstacle
    path = path_finder.find_path((600, 600))  # Find path to the target
    # Plot the path
    print(path[0])
    plt.plot([x for x, y in path], [y for x, y in path])
    plt.show()

