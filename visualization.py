import matplotlib.pyplot as plt
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon
from shapely.geometry import LineString, Polygon

from graph import MapCircle, GraphPoint
from log_model import logger


def add_linestring(ax, linestring, color='r'):
    x, y = linestring.xy
    ax.plot(x, y, color=color)


def visualize_graph(graph, obstacles, shortest_path, filename='graph_visualization.pdf'):
    logger.info("Visualizing graph...")
    fig, ax = plt.subplots(figsize=(10, 10))  # Adjust figure size as needed
    ax.set_aspect('equal')

    # Draw obstacles (assuming obstacles are shapely geometries for simplicity)
    for obstacle in obstacles:
        if isinstance(obstacle, LineString):
            x, y = obstacle.xy
            ax.plot(x, y, color='r', linewidth=0.5)
        elif isinstance(obstacle, Polygon):
            x, y = obstacle.exterior.xy
            ax.add_patch(MplPolygon(list(zip(x, y)), closed=True, color='r', fill=False, linewidth=0.5))
        elif isinstance(obstacle, MapCircle):  # Adjusted for custom Circle objects
            # Draw Circle using matplotlib.patches.Circle
            center = (obstacle.center[0], obstacle.center[1])  # Adjust if necessary
            circle = MplCircle(center, obstacle.radius, edgecolor='r', facecolor='none', linewidth=0.5)
            ax.add_patch(circle)

    #  Draw all paths in the graph (for demonstration purposes)
    for from_node_id, edges in graph.edges.items():
        if from_node_id:
            from_node = graph.nodes[from_node_id]
            for to_node_id, _ in edges:
                to_node = graph.nodes[to_node_id]
                ax.plot([from_node.x, to_node.x], [from_node.y, to_node.y], '#808080', linewidth=0.5)#, alpha=0.5)

    #  Highlight the path from start to goal if provided
    if shortest_path:
        for i in range(len(shortest_path) - 1):
            start_coords = graph.nodes[shortest_path[i].id]
            end_coords = graph.nodes[shortest_path[i + 1].id]
            ax.plot([start_coords.x, end_coords.x], [start_coords.y, end_coords.y], 'b-', linewidth=2)

    # Draw nodes
    room_nodes_ids = [n.id for n in graph.room_nodes]
    for node_id, point in graph.nodes.items():
        if point.id in room_nodes_ids:
            ax.plot(point.x, point.y, 'o', markersize=2, label=point.id, color='k')  # Nodes as dots
        else:
            ax.plot(point.x, point.y, 'o', markersize=0.5, color='b')  # Nodes as dots

    # Draw start and goal nodes distinctly
    start_node: GraphPoint = graph.nodes.get('start')
    goal_node: GraphPoint = graph.nodes.get('goal')
    if start_node:
        ax.plot(start_node.x, start_node.y, 'go')  # Green for start

    if goal_node:
        ax.plot(goal_node.x, goal_node.y, 'ro')  # Red for goal

    plt.savefig(filename, format='pdf')
    plt.close(fig)  # Close the figure
