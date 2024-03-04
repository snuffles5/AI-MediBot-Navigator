import matplotlib.pyplot as plt
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon, Circle
from shapely.geometry import Point, LineString, Polygon

from graph import MapCircle


def add_linestring(ax, linestring, color='r'):
    x, y = linestring.xy
    ax.plot(x, y, color=color)


def visualize_graph(graph, obstacles, final_path):
    fig, ax = plt.subplots(figsize=(10, 10))  # Adjust figure size as needed
    ax.set_aspect('equal')

    # Draw nodes
    for node_id, coords in graph.nodes.items():
        if node_id.split("_")[0] not in ["random", "start", "goal"]:
            ax.plot(coords[0], coords[1], 'o', markersize=5, label=node_id)  # Nodes as dots
        else:
            ax.plot(coords[0], coords[1], 'o', markersize=3, color='b')  # Nodes as dots

    # Draw edges
    # for from_node, edges in graph.edges.items():
    #     from_coords = graph.nodes[from_node]
    #     for to_node, cost in edges:
    #         to_coords = graph.nodes[to_node]
    #         ax.plot([from_coords[0], to_coords[0]], [from_coords[1], to_coords[1]], 'k-', linewidth=0.2)  # Edges as lines
    # Draw all edges in the graph (for demonstration purposes)
    for from_node, edges in graph.edges.items():
        from_coords = graph.nodes[from_node]
        for to_node, _ in edges:
            to_coords = graph.nodes[to_node]
            ax.plot([from_coords[0], to_coords[0]], [from_coords[1], to_coords[1]], 'k-', linewidth=0.5, alpha=0.5)

    # Highlight the path from start to goal if provided
    if final_path:
        for i in range(len(final_path) - 1):
            start_coords = graph.nodes[final_path[i]]
            end_coords = graph.nodes[final_path[i+1]]
            ax.plot([start_coords[0], end_coords[0]], [start_coords[1], end_coords[1]], 'b-', linewidth=2)

    # Draw start and goal nodes distinctly
    if graph.start:
        start_coords = graph.nodes[graph.start]
        ax.plot(start_coords[0], start_coords[1], 'go')  # Green for start

    if graph.goal:
        goal_coords = graph.nodes[graph.goal]
        ax.plot(goal_coords[0], goal_coords[1], 'ro')  # Red for goal

    plt.show()


    # Draw obstacles (assuming obstacles are shapely geometries for simplicity)
    for obstacle in obstacles:
        if isinstance(obstacle, LineString):
            x, y = obstacle.xy
            ax.plot(x, y, color='r')
        elif isinstance(obstacle, Polygon):
            x, y = obstacle.exterior.xy
            ax.add_patch(MplPolygon(list(zip(x, y)), closed=True, color='r', fill=False))
        elif isinstance(obstacle, MapCircle):  # Adjusted for custom Circle objects
            # Draw Circle using matplotlib.patches.Circle
            center = (obstacle.center[0], obstacle.center[1])  # Adjust if necessary
            circle = MplCircle(center, obstacle.radius, edgecolor='r', facecolor='none')
            ax.add_patch(circle)

    plt.legend()
    plt.show()
