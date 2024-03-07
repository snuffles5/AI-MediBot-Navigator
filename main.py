import time
import uuid
import tracemalloc

import matplotlib
import numpy as np

from visualization import visualize_graph, \
    generate_comparison_charts  # Assuming generate_comparison_charts is implemented
from log_model import logger
from prm import PRM, SearchType
from obstacles import create_obstacles
from graph import Graph, graph_rooms_nodes, graph_general_nodes, GOAL_POINT_ID

matplotlib.use('TkAgg')

RUN_TIMES = 5


def duplicate_graph(original_graph):
    new_graph = Graph(nodes=dict(original_graph.nodes).copy())
    new_graph.edges = dict(original_graph.edges).copy()
    return new_graph


def main():
    # Initialize performance metrics dictionaries
    matplotlib.use('TkAgg')
    performance_metrics = {
        'A*': {'execution_time': [], 'iterations': [], 'memory_usage': []},
        'Dijkstra': {'execution_time': [], 'iterations': [], 'memory_usage': []}
    }

    obstacles = create_obstacles()
    start = graph_general_nodes.get("start")
    goal = graph_general_nodes.get("goal")
    num_random_nodes_list = [20]

    for num_random_nodes in num_random_nodes_list:
        temp_metrics = {
            'A*': {'execution_time': [], 'iterations': [], 'memory_usage': []},
            'Dijkstra': {'execution_time': [], 'iterations': [], 'memory_usage': []}
        }
        found_shortest_path = False
        failed_attempts = 0
        while not found_shortest_path:
            a_star_graph = Graph(nodes={**graph_rooms_nodes, **graph_general_nodes})
            a_star_prm = PRM(graph=a_star_graph, obstacles=obstacles, start=start, goal=goal,
                             num_random_nodes=num_random_nodes,
                             search_type=SearchType.A_STAR_SEARCH)
            a_star_prm.generate_random_nodes()
            a_star_prm.connect_nodes()

            dijkstra_graph = duplicate_graph(a_star_graph)
            dijkstra_prm = PRM(graph=dijkstra_graph, obstacles=obstacles, start=start, goal=goal,
                               num_random_nodes=num_random_nodes,
                               search_type=SearchType.DIJKSTRA)

            # For A* search
            tracemalloc.start()
            start_time = time.time()
            a_star_result = a_star_prm.find_path()
            a_star_time = time.time() - start_time
            a_star_memory_usage = tracemalloc.get_traced_memory()[1]
            tracemalloc.stop()

            # For Dijkstra search
            tracemalloc.start()
            start_time = time.time()
            dijkstra_result = dijkstra_prm.find_path()
            dijkstra_time = time.time() - start_time
            dijkstra_memory_usage = tracemalloc.get_traced_memory()[1]
            tracemalloc.stop()

            # Update metrics collection
            temp_metrics['A*']['execution_time'].append(a_star_time)
            temp_metrics['A*']['iterations'].append(
                a_star_result.iterations)  # Assuming iterations is part of the result
            temp_metrics['A*']['memory_usage'].append(a_star_memory_usage)

            temp_metrics['Dijkstra']['execution_time'].append(dijkstra_time)
            temp_metrics['Dijkstra']['iterations'].append(dijkstra_result.iterations)  # Same assumption
            temp_metrics['Dijkstra']['memory_usage'].append(dijkstra_memory_usage)

            found_shortest_path = a_star_prm.shortest_path_cost != -1 or dijkstra_prm.shortest_path_cost != -1
            if found_shortest_path:
                logger.info(f"Time taken for A*: {round(a_star_time, 5)} seconds")
                logger.info(f"Failed attempts {failed_attempts}")
                logger.info(f"Time taken for Dijkstra: {round(dijkstra_time, 5)} seconds")
                logger.info(f"Number of nodes {num_random_nodes}")
                run_prefix = uuid.uuid4().hex[-5:]
                visualize_graph(a_star_graph, obstacles, a_star_prm.shortest_path, a_star_prm,
                                num_random_nodes=num_random_nodes, failed_attempts=failed_attempts,
                                filename=f'{run_prefix}_graph_visualization_a*.pdf')
                visualize_graph(dijkstra_graph, obstacles, dijkstra_prm.shortest_path, dijkstra_prm,
                                num_random_nodes=num_random_nodes, failed_attempts=failed_attempts,
                                filename=f'{run_prefix}_graph_visualization_dijkstra.pdf')
            else:
                failed_attempts += 1
        for method in ['A*', 'Dijkstra']:
            for metric in ['execution_time', 'iterations', 'memory_usage']:
                average_value = np.mean(temp_metrics[method][metric])
                performance_metrics[method][metric].append(average_value)

    generate_comparison_charts(performance_metrics['A*'], performance_metrics['Dijkstra'], num_random_nodes_list)


if __name__ == "__main__":
    main()
