import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from shapely.geometry import LineString


def add_polygon(ax, polygon, color='r'):
    x, y = polygon.exterior.xy
    ax.add_patch(Polygon(list(zip(x, y)), closed=True, color=color))


def add_linestring(ax, linestring, color='r'):
    x, y = linestring.xy
    ax.plot(x, y, color=color)


def visualize_graph(graph, obstacles):
    fig, ax = plt.subplots()
    # Draw nodes
    for node, coords in graph.nodes.items():
        ax.plot(coords[0], coords[1], 'go')  # 'go' for green dots

    # Draw edges
    for edge in graph.edges:
        node1, node2, _ = edge
        coords1, coords2 = graph.nodes[node1], graph.nodes[node2]
        ax.plot([coords1[0], coords2[0]], [coords1[1], coords2[1]], 'k-')  # 'k-' for black lines

    # Draw obstacles
    for obstacle in obstacles:
        if isinstance(obstacle, Polygon):
            add_polygon(ax, obstacle)
        elif isinstance(obstacle, LineString):
            add_linestring(ax, obstacle)

    plt.show()
