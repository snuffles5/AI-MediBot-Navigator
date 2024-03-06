import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon
from shapely.geometry import LineString, Polygon
from log_model import logger


def add_linestring(ax, linestring, color='r'):
    x, y = linestring.xy
    ax.plot(x, y, color=color)


def draw_obstacles(ax, obstacles):
    for obstacle in obstacles:
        if isinstance(obstacle, LineString):
            x, y = obstacle.xy
            ax.plot(x, y, color='r', linewidth=0.5)
        elif isinstance(obstacle, Polygon):
            x, y = obstacle.exterior.xy
            ax.add_patch(MplPolygon(list(zip(x, y)), closed=True, color='r', fill=True, linewidth=0.5))
        elif isinstance(obstacle, MplCircle):  # Adjusted for custom Circle objects
            center = (obstacle.center[0], obstacle.center[1])  # Adjust if necessary
            circle = MplCircle(center, obstacle.radius, edgecolor='r', facecolor='r', linewidth=0.5)
            ax.add_patch(circle)


def draw_all_paths(ax, graph, show_all_paths_costs):
    for from_node_id, edges in graph.edges.items():
        from_node = graph.nodes[from_node_id]
        for to_node_id, cost in edges:
            to_node = graph.nodes[to_node_id]
            ax.plot([from_node.x, to_node.x], [from_node.y, to_node.y], '#808080', linewidth=0.5)  # Gray for paths
            if show_all_paths_costs:  # Display cost for each path if enabled
                midpoint = ((from_node.x + to_node.x) / 2, (from_node.y + to_node.y) / 2)
                ax.text(midpoint[0], midpoint[1], f'{cost:.2f}', fontsize=4, ha='center', va='center',
                        bbox=dict(boxstyle="round,pad=0.1", facecolor='gray', alpha=0.5))


def highlight_shortest_path(ax, graph, shortest_path, show_shortest_path_costs):
    if shortest_path:
        for i in range(len(shortest_path) - 1):
            start_coords = graph.nodes[shortest_path[i].id]
            end_coords = graph.nodes[shortest_path[i + 1].id]
            # Draw the path edge
            ax.plot([start_coords.x, end_coords.x], [start_coords.y, end_coords.y], 'g-', linewidth=2)
            if show_shortest_path_costs:  # If showing costs is enabled, display the cost on the edge
                edge_cost = graph.cost(shortest_path[i].id, shortest_path[i + 1].id)
                midpoint = ((start_coords.x + end_coords.x) / 2, (start_coords.y + end_coords.y) / 2)
                ax.text(midpoint[0], midpoint[1], f'{edge_cost:.2f}', fontsize=6, ha='center', va='center',
                        bbox=dict(boxstyle="round,pad=0.1", facecolor='yellow', alpha=1))


def draw_nodes(ax, graph, start_node, goal_node):
    room_nodes_ids = [n.id for n in graph.room_nodes]
    for node_id, point in graph.nodes.items():
        if point.id in room_nodes_ids:
            ax.plot(point.x, point.y, 'o', markersize=2, label=point.id, color='k')  # Nodes as dots
        else:
            ax.plot(point.x, point.y, 'o', markersize=0.5, color='b')  # Nodes as dots

    # Draw start and goal nodes distinctly

    if start_node:
        ax.plot(start_node.x, start_node.y, 'go')  # Green for start

    if goal_node:
        ax.plot(goal_node.x, goal_node.y, 'ro')  # Red for goal


def print_plt(ax, start_node, goal_node, prm, filename, fig, is_available_shortest_path, num_random_nodes, failed_attempts):
    if is_available_shortest_path:
        text_str = f"Path from {start_node} to {goal_node}\nTotal Cost: {round(prm.shortest_path_cost, 3)}\n" \
                   f"Search Type: {prm.search_type.value}, Total Nodes: {num_random_nodes}, {failed_attempts} Failed attempts"
        ax.text(0.05, 0.95, text_str, transform=ax.transAxes, fontsize=12,
                verticalalignment='top', bbox=dict(boxstyle="round", facecolor='white', alpha=0.5))

        plt.savefig(f"{filename}", format='pdf')
        plt.close(fig)  # Close the figure


def visualize_graph(graph, obstacles, shortest_path, prm, num_random_nodes, failed_attempts, filename='graph_visualization.pdf',
                    show_shortest_path_costs=True, show_all_paths_costs=True, is_available_shortest_path=False, ):
    logger.info("Visualizing graph...")
    if not is_available_shortest_path:
        if prm.shortest_path_cost != -1:
            is_available_shortest_path = True
        else:
            return

    # Initial Setup for Obstacles and All Paths Visualization
    # fig_0, ax_0 = plt.subplots(figsize=(10, 10))
    # ax_0.set_aspect('equal')
    # draw_obstacles(ax_0, obstacles)
    # draw_all_paths(ax_0, graph, num_random_nodes < 200)
    # draw_nodes(ax_0, graph, graph.nodes.get('start'), graph.nodes.get('goal'))
    # print_plt(ax_0, graph.nodes.get('start'), graph.nodes.get('goal'), prm, f"exports/0_{num_random_nodes}_{filename}", fig_0,
    #           is_available_shortest_path, num_random_nodes)

    # Setup for Highlighting the Shortest Path Visualization
    fig_1, ax_1 = plt.subplots(figsize=(10, 10))
    ax_1.set_aspect('equal')
    draw_obstacles(ax_1, obstacles)
    draw_all_paths(ax_1, graph, num_random_nodes < 200)
    highlight_shortest_path(ax_1, graph, shortest_path, show_shortest_path_costs)
    draw_nodes(ax_1, graph, graph.nodes.get('start'), graph.nodes.get('goal'))
    print_plt(ax_1, graph.nodes.get('start'), graph.nodes.get('goal'), prm, f"exports/1_{num_random_nodes}_{filename}", fig_1,
              is_available_shortest_path, num_random_nodes, failed_attempts)


def generate_comparison_charts(a_star_data: dict, dijkstra_data: dict, num_random_nodes_list, filename="performance_comparison.png"):
    logger.info("generate comparison charts...")
    metrics = ['execution_time', 'iterations', 'memory_usage']
    titles = ['Execution Time (seconds)', 'Number of Iterations', 'Memory Usage (bytes)']
    fig, axs = plt.subplots(len(metrics), 1, figsize=(10, 15))
    bar_width = 0.35  # Width of the bars

    # Calculate positions for each set of bars
    index = np.arange(len(num_random_nodes_list))

    for i, metric in enumerate(metrics):
        a_star_metrics = a_star_data.get(metric, [])
        dijkstra_metrics = dijkstra_data.get(metric, [])

        # Check to ensure dimensions match. If not, skip plotting for this metric.
        if len(a_star_metrics) != len(num_random_nodes_list) or len(dijkstra_metrics) != len(num_random_nodes_list):
            logger.error(f"Dimension mismatch for {metric}. Check data aggregation.")
            continue

        # Plotting the bar chart
        axs[i].bar(index - bar_width/2, a_star_metrics, bar_width, label='A* ' + metric.replace('_', ' ').title())
        axs[i].bar(index + bar_width/2, dijkstra_metrics, bar_width, label='Dijkstra ' + metric.replace('_', ' ').title())

        axs[i].set_xlabel('Number of Random Nodes')
        axs[i].set_ylabel(titles[i])
        axs[i].set_title(titles[i] + ' Comparison')
        axs[i].set_xticks(index)
        axs[i].set_xticklabels(num_random_nodes_list)
        axs[i].legend()

    plt.tight_layout()
    plt.savefig("exports/" + filename, transparent=True)
    plt.close()
