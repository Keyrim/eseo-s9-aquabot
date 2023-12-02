import matplotlib.pyplot as plt

class Vector2DPlotter:
    def __init__(self, num_vectors, title='Vector Plotter', plot_size=(500, 500), vector_labels=None):
        self.num_vectors = num_vectors
        self.vectors = [[0, 0] for _ in range(num_vectors)]  # Initialize vectors

        # Set up the matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [], 'o-', label=f'Vector {i}' if vector_labels is None else vector_labels[i])[0] for i in range(num_vectors)]
        height, width = plot_size
        self.ax.set_xlim(-height, height)  # Set x-axis limits
        self.ax.set_ylim(-width, width)  # Set y-axis limits
        self.ax.legend()

    def update_vector(self, id_vec, x, y):
        if 0 <= id_vec < self.num_vectors:
            self.vectors[id_vec] = [x, y]

    def update_plot(self):
        for i, line in enumerate(self.lines):
            line.set_data([0, self.vectors[i][0]], [0, self.vectors[i][1]])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def show(self):
        plt.ion()  # Enable interactive mode
        self.fig.show()

# Example usage in your ROS2 node
if __name__ == '__main__':
    plotter = Vector2DPlotter(num_vectors=3)  # 3 vectors
    plotter.show()

    def timer_callback():
        # Update vectors as needed
        plotter.update_vector(0, 5, 5)
        plotter.update_vector(1, -3, 2)
        plotter.update_vector(2, 4, -4)
        plotter.update_plot()

    # Create and start a ROS2 timer in your node
    # node.create_timer(1.0, timer_callback)  # Update every second
