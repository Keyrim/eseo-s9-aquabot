import matplotlib.pyplot as plt

class GraphPlotter:
    def __init__(self, title='Points Plotter', plot_size=(500, 500)):
        self.points = []  # Initialize an empty list for points

        # Set up the matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'o-', label='Connected Points')  # Line object to plot points
        height, width = plot_size
        self.ax.set_xlim(-height, height)  # Set x-axis limits
        self.ax.set_ylim(-width, width)  # Set y-axis limits
        self.ax.legend()

    def set_points(self, points):
        self.points = points

    def update_plot(self):
        xs, ys = zip(*self.points) if self.points else ([], [])
        self.line.set_data(xs, ys)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def show(self):
        plt.ion()  # Enable interactive mode
        self.fig.show()

# Example usage
if __name__ == '__main__':
    plotter = GraphPlotter()  # Points plotter
    plotter.show()

    def timer_callback():
        # Set and update points as needed
        points = [(5, 5), (-3, 2), (4, -4)]
        plotter.set_points(points)
        plotter.update_plot()

    # You can integrate this with a ROS2 timer in a similar way as before
    # node.create_timer(1.0, timer_callback)  # Update every second
