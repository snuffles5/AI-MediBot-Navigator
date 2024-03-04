import matplotlib
from prm import PRM
from visualization import visualize_graph
from obstacles import create_obstacles
from graph import Graph, graph_rooms_nodes, graph_general_nodes


def main():
    matplotlib.use('TkAgg')  # Replace 'TkAgg' with a backend appropriate for your system
    # Initialize the graph and obstacles
    graph = Graph(nodes={**graph_rooms_nodes, **graph_general_nodes})
    obstacles = create_obstacles()
    start = graph_general_nodes.get("start")
    goal = graph_rooms_nodes.get("B")

    # Initialize PRM with the graph and obstacles
    prm = PRM(graph=graph, obstacles=obstacles, start=start, goal=goal, num_random_nodes=50)

    # Generate random nodes and connect them
    final_path = prm.run()

    visualize_graph(graph, obstacles, final_path)


if __name__ == "__main__":
    main()
