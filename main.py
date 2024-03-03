import matplotlib

from prm import PRM
from visualization import visualize_graph
from obstacles import create_obstacles
from graph import Graph


def main():
    matplotlib.use('TkAgg')  # Replace 'TkAgg' with a backend appropriate for your system
    # Initialize the graph and obstacles
    graph = Graph()
    obstacles = create_obstacles()

    # Initialize PRM with the graph and obstacles
    # prm = PRM(graph, obstacles)

    # Generate random nodes and connect them
    # prm.generate_random_nodes()
    # prm.connect_nodes()

    # Visualize the final graph
    visualize_graph(graph, obstacles)
    # visualize_obstacles(obstacles)


if __name__ == "__main__":
    main()
