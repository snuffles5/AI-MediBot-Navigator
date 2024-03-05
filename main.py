import uuid

import matplotlib
from prm import PRM, SearchType
from visualization import visualize_graph
from obstacles import create_obstacles
from graph import Graph, graph_rooms_nodes, graph_general_nodes

RUN_TIMES = 5
NUM_RANDOM_NODES = 500


def duplicate_graph(original_graph):
    new_graph = Graph(nodes=dict(original_graph.nodes).copy())  # Create a new graph with the same nodes
    new_graph.edges = dict(original_graph.edges).copy()  # Copy the edges
    return new_graph


def main():
    matplotlib.use('TkAgg')  # Replace 'TkAgg' with a backend appropriate for your system
    # Initialize the graph and obstacles
    obstacles = create_obstacles()
    found_shortest_path = False
    start = graph_general_nodes.get("start")
    goal = graph_rooms_nodes.get("L")

    while not found_shortest_path:
        a_start_graph = Graph(nodes={**graph_rooms_nodes, **graph_general_nodes})
        a_star_prm = PRM(graph=a_start_graph, obstacles=obstacles, start=start, goal=goal,
                         num_random_nodes=NUM_RANDOM_NODES,
                         search_type=SearchType.A_STAR_SEARCH)
        a_star_prm.generate_random_nodes()
        a_star_prm.connect_nodes()
        dijkstra_graph = duplicate_graph(a_start_graph)
        a_start_shortest_path = a_star_prm.find_path()

        dijkstra_prm = PRM(graph=dijkstra_graph, obstacles=obstacles, start=start, goal=goal,
                         num_random_nodes=NUM_RANDOM_NODES,
                         search_type=SearchType.DIJKSTRA)
        dijkstra_shortest_path = dijkstra_prm.find_path()
        found_shortest_path = a_star_prm.shortest_path_cost != -1 and dijkstra_prm.shortest_path_cost != -1
        if found_shortest_path:
            suffix = uuid.uuid4().hex[-5:]
            visualize_graph(a_start_graph, obstacles, a_start_shortest_path, a_star_prm,
                            filename=f'graph_visualization_a*_{suffix}.pdf')
            visualize_graph(dijkstra_graph, obstacles, dijkstra_shortest_path, dijkstra_prm,
                            filename=f'graph_visualization_dijkstra_{suffix}.pdf')


if __name__ == "__main__":
    main()
