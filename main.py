import matplotlib

from prm import PRM
from visualization import visualize_graph
from obstacles import create_obstacles
from graph import Graph, nodes


def main():
    matplotlib.use('TkAgg')  # Replace 'TkAgg' with a backend appropriate for your system
    # Initialize the graph and obstacles
    graph = Graph()
    obstacles = create_obstacles()
    start = nodes.get("start")
    goal = nodes.get("B")

    # Initialize PRM with the graph and obstacles
    prm = PRM(graph, obstacles, start, goal)

    # Generate random nodes and connect them
    prm.run()

    # Visualize the final graph
    visualize_graph(graph, obstacles)
    # visualize_obstacles(obstacles)


if __name__ == "__main__":
    main()
