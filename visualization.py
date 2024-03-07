import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon
from shapely.geometry import LineString, Polygon

from graph import GraphPoint
from log_model import logger
from prm import SearchType


def add_linestring(ax, linestring, color='r'):
    x, y = linestring.xy
    ax.plot(x, y, color=color)


def draw_obstacles(ax, obstacles):
    color = 'b'  # blue
    for obstacle in obstacles:
        if isinstance(obstacle, LineString):
            x, y = obstacle.xy
            ax.plot(x, y, color=color, linewidth=0.5)
        elif isinstance(obstacle, Polygon):
            x, y = obstacle.exterior.xy
            ax.add_patch(MplPolygon(list(zip(x, y)), closed=True, color=color, fill=True, linewidth=0.5))
        elif isinstance(obstacle, MplCircle):  # Adjusted for custom Circle objects
            center = (obstacle.center[0], obstacle.center[1])  # Adjust if necessary
            circle = MplCircle(center, obstacle.radius, edgecolor=color, facecolor='b', linewidth=0.5)
            ax.add_patch(circle)


def calculate_text_angle(from_node, to_node):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    angle = np.degrees(np.arctan2(dy, dx))
    if dx < 0:  # If the line goes from right to left
        angle += 180  # Rotate text to avoid upside-down orientation
    return angle


def draw_all_paths(ax, graph, prm, show_all_paths_costs, h_costs=None, is_mark_visited=True):
    drawn_edges = set()
    visited_edges = prm.visited_edges if is_mark_visited else set()  # Access visited edges from PRM instance
    for from_node_id, edges in graph.edges.items():
        from_node = graph.nodes[from_node_id]
        for to_node_id, cost in edges:
            to_node = graph.nodes[to_node_id]
            if (to_node_id, from_node_id) in drawn_edges:
                logger.debug("found duplicate edge")
                continue  # Skip drawing if edge has already been drawn in the opposite direction

            angle = calculate_text_angle(from_node, to_node)
            midpoint = ((from_node.x + to_node.x) / 2, (from_node.y + to_node.y) / 2)
            edge_color = '#808080'  # Default gray color for paths
            edge_width = 0.3

            # If marking visited paths and this path is visited, adjust color and width
            if is_mark_visited and ((from_node_id, to_node_id) in visited_edges or (to_node_id, from_node_id) in visited_edges):
                edge_color = 'orange'  # Or any color that signifies a visited path
                edge_width = 1  # Make visited paths more prominent

            ax.plot([from_node.x, to_node.x], [from_node.y, to_node.y], edge_color, linewidth=edge_width,
                    zorder=1)  # Gray for paths
            drawn_edges.add((from_node_id, to_node_id))  # Mark this edge as drawn


            if show_all_paths_costs and h_costs is None:  # Display g-cost for each path if enabled
                ax.text(midpoint[0], midpoint[1], f'{cost:.2f}', fontsize=3, ha='center', va='center', rotation=angle,
                        bbox=dict(boxstyle="round,pad=0.1", facecolor='gray', alpha=1, linewidth=0.1), zorder=5)
            elif h_costs:  # Optionally display h-costs for A* search visualization
                h_cost = h_costs.get(from_node_id, 0)  # Assuming h_costs is a dict with node_id as keys
                ax.text(from_node.x, from_node.y, f'h={h_cost:.2f}', fontsize=3, color='blue', ha='center', zorder=5,
                        bbox=dict(boxstyle="round,pad=0.1", facecolor='gray', alpha=1, linewidth=0.1))


def highlight_shortest_path(ax, graph, shortest_path, show_shortest_path_costs):
    if shortest_path:
        for i in range(len(shortest_path) - 1):
            start_coords = graph.nodes[shortest_path[i].id]
            end_coords = graph.nodes[shortest_path[i + 1].id]
            # Draw the path edge
            ax.plot([start_coords.x, end_coords.x], [start_coords.y, end_coords.y], 'g-', linewidth=3, zorder=3)
            if show_shortest_path_costs:  # If showing costs is enabled, display the cost on the edge
                edge_cost = graph.cost(shortest_path[i].id, shortest_path[i + 1].id)
                midpoint = ((start_coords.x + end_coords.x) / 2, (start_coords.y + end_coords.y) / 2)
                ax.text(midpoint[0], midpoint[1], f'{edge_cost:.2f}', fontsize=5, ha='center', va='center',
                        bbox=dict(boxstyle="round,pad=0.1", facecolor='yellow', alpha=1, zorder=5))


def draw_nodes(ax, graph, prm, start_node: GraphPoint, goal_node: GraphPoint, is_mark_visited=True):
    room_nodes_ids = [n.id for n in graph.room_nodes]
    visited_nodes_ids = set(prm.visited_nodes)
    for node_id, point in graph.nodes.items():
        if is_mark_visited and node_id in visited_nodes_ids:
            ax.plot(point.x, point.y, 'o', markersize=5, color='orange', zorder=1)  # Mark visited nodes in orange
        if not (goal_node.same_coords(point) or start_node.same_coords(point)):
            if point.id in room_nodes_ids:
                ax.plot(point.x, point.y, 'o', markersize=2, label=point.id, color='k')  # Nodes as dots
            else:
                ax.plot(point.x, point.y, 'o', markersize=0.5, color='b')  # Nodes as dots

    # Draw start and goal nodes distinctly

    if start_node:
        ax.plot(start_node.x, start_node.y, 'go', markersize=10, zorder=1)  # Increase zorder=1or visibility
        ax.text(start_node.x + 30, start_node.y, 'Start', fontsize=12, ha='center', color='green', zorder=5)

    if goal_node:
        ax.plot(goal_node.x, goal_node.y, 'ro', markersize=10, zorder=1)  # Increase zorder=1or visibility
        ax.text(goal_node.x + 30, goal_node.y, 'Goal', fontsize=12, ha='center', color='red', zorder=5)


def print_plt(ax, start_node, goal_node, prm, filename, fig, is_available_shortest_path, num_random_nodes,
              failed_attempts):
    if is_available_shortest_path:
        text_str = f"Path from {start_node} to {goal_node}\nTotal Cost: {round(prm.shortest_path_cost, 3)}\n" \
                   f"Search Type: {prm.search_type.value}, Total Nodes: {num_random_nodes}, {failed_attempts} Failed attempts"
        ax.text(0.05, 0.95, text_str, transform=ax.transAxes, fontsize=12,
                verticalalignment='top', bbox=dict(boxstyle="round", facecolor='white', alpha=0.5, zorder=5))

        plt.savefig(f"{filename}", format='pdf')
        plt.close(fig)


def visualize_graph(graph, obstacles, shortest_path, prm, num_random_nodes, failed_attempts,
                    filename='graph_visualization.pdf',
                    show_shortest_path_costs=True, show_all_paths_costs=True, is_available_shortest_path=False, ):
    logger.info("Visualizing graph...")
    if not is_available_shortest_path:
        if prm.shortest_path_cost != -1:
            is_available_shortest_path = True
        else:
            return

    # Setup for Highlighting the Shortest Path Visualization
    fig_1, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    draw_obstacles(ax, obstacles)

    # Check if using A* and if h-costs are available
    if prm.search_type == SearchType.A_STAR_SEARCH and hasattr(prm, 'h_costs'):
        draw_all_paths(ax, graph, prm, show_all_paths_costs, prm.h_costs)
        highlight_shortest_path(ax, graph, shortest_path, show_all_paths_costs)
    elif prm.search_type == SearchType.DIJKSTRA:
        draw_all_paths(ax, graph, prm, show_all_paths_costs)
        highlight_shortest_path(ax, graph, shortest_path, show_all_paths_costs)
    else:
        return

    draw_nodes(ax, graph, prm, graph.nodes.get('start'), graph.nodes.get('goal'))
    print_plt(ax, graph.nodes.get('start'), graph.nodes.get('goal'), prm, f"exports/1_{num_random_nodes}_{filename}",
              fig_1, is_available_shortest_path, num_random_nodes, failed_attempts)


def generate_merged_comparison_charts(a_star_data: dict, dijkstra_data: dict, filename_prefix="comparison"):
    logger.info("Generating comparison charts...")
    metrics = ['execution_time', 'iterations', 'memory_usage']
    titles = ['Average Execution Time (seconds)', 'Average Number of Iterations', 'Average Memory Usage (bytes)']
    algorithms = ['A*', 'Dijkstra']

    # Setting up the figure for the bar charts
    for metric, title in zip(metrics, titles):
        # Compute average values for each algorithm for the current metric
        a_star_avg = np.mean(a_star_data.get(metric, []))
        dijkstra_avg = np.mean(dijkstra_data.get(metric, []))

        # Data preparation for plotting
        values = [a_star_avg, dijkstra_avg]
        fig, ax = plt.subplots(figsize=(8, 6))

        # Create bar chart
        bars = ax.bar(algorithms, values, color=['blue', 'green'])
        ax.set_ylabel(title)
        ax.set_title(title)
        ax.set_xticks(range(len(algorithms)))
        ax.set_xticklabels(algorithms)

        # Adding the text labels on the bars
        for bar in bars:
            yval = bar.get_height()
            ax.text(bar.get_x() + bar.get_width() / 2, yval, round(yval, 2), va='bottom')  # Adjust formatting as needed

        # Save each chart with a metric-specific filename
        plt.tight_layout()
        plt.savefig(f"exports/{filename_prefix}_{metric}.png", transparent=True)
        plt.close()


def generate_comparison_charts(a_star_data: dict, dijkstra_data: dict, num_random_nodes_list,
                               filename="performance_comparison.png"):
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
        axs[i].bar(index - bar_width / 2, a_star_metrics, bar_width, label='A* ' + metric.replace('_', ' ').title(), color=['blue'])
        axs[i].bar(index + bar_width / 2, dijkstra_metrics, bar_width, color=['green'],
                   label='Dijkstra ' + metric.replace('_', ' ').title())

        axs[i].set_xlabel('Number of Random Nodes')
        axs[i].set_ylabel(titles[i])
        axs[i].set_title(titles[i] + ' Comparison')
        axs[i].set_xticks(index)
        axs[i].set_xticklabels(num_random_nodes_list)
        axs[i].legend()

    plt.tight_layout()
    plt.savefig("exports/" + filename, transparent=True)
    plt.close()
