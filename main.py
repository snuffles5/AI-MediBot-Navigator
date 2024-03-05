import time
import uuid

import matplotlib

from log_model import logger
from prm import PRM, SearchType
from visualization import visualize_graph
from obstacles import create_obstacles
from graph import Graph, graph_rooms_nodes, graph_general_nodes

RUN_TIMES = 5


def duplicate_graph(original_graph):
    new_graph = Graph(nodes=dict(original_graph.nodes).copy())
    new_graph.edges = dict(original_graph.edges).copy()
    return new_graph


def main():
    matplotlib.use('TkAgg')
    # Initialize the graph and obstacles
    obstacles = create_obstacles()
    start = graph_general_nodes.get("start")
    goal = graph_rooms_nodes.get("H")
    num_random_nodes_list = [20, 50, 100, 200, 500]

    for num_random_nodes in num_random_nodes_list:
        found_shortest_path = False
        failed_attempts = 0
        while not found_shortest_path:
            a_start_graph = Graph(nodes={**graph_rooms_nodes, **graph_general_nodes})
            a_star_prm = PRM(graph=a_start_graph, obstacles=obstacles, start=start, goal=goal,
                             num_random_nodes=num_random_nodes,
                             search_type=SearchType.A_STAR_SEARCH)
            a_star_prm.generate_random_nodes()
            a_star_prm.connect_nodes()
            dijkstra_graph = duplicate_graph(a_start_graph)
            start_time = time.time()
            a_start_shortest_path = a_star_prm.find_path()
            end_time = time.time()
            a_star_time = end_time - start_time

            dijkstra_prm = PRM(graph=dijkstra_graph, obstacles=obstacles, start=start, goal=goal,
                             num_random_nodes=num_random_nodes,
                             search_type=SearchType.DIJKSTRA)
            start_time = time.time()
            dijkstra_shortest_path = dijkstra_prm.find_path()
            end_time = time.time()
            dijkstra_time = end_time - start_time

            found_shortest_path = a_star_prm.shortest_path_cost != -1 and dijkstra_prm.shortest_path_cost != -1
            if found_shortest_path:
                logger.info(f"Time taken for A*: {round(a_star_time, 5)} seconds")
                logger.info(f"Failed attempts {failed_attempts}")
                logger.info(f"Time taken for Dijkstra: {round(dijkstra_time, 5)} seconds")
                logger.info(f"Number of nodes {num_random_nodes}")
                run_prefix = uuid.uuid4().hex[-5:]
                visualize_graph(a_start_graph, obstacles, a_start_shortest_path, a_star_prm, num_random_nodes=num_random_nodes, failed_attempts=failed_attempts,
                                filename=f'{run_prefix}_graph_visualization_a*.pdf', is_available_shortest_path=True)
                visualize_graph(dijkstra_graph, obstacles, dijkstra_shortest_path, dijkstra_prm, num_random_nodes=num_random_nodes, failed_attempts=failed_attempts,
                                filename=f'{run_prefix}_graph_visualization_dijkstra.pdf')
            else:
                failed_attempts += 1


if __name__ == "__main__":
    main()
