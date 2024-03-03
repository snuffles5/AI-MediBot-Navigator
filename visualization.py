import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.patches import Circle


def add_circle(ax, point, radius, color='r'):
    circle = Circle((point.x, point.y), radius, color=color, fill=True)
    ax.add_patch(circle)


def add_polygon(ax, polygon, color='r'):
    x, y = polygon.exterior.xy
    mpl_polygon = MplPolygon(list(zip(x, y)), closed=True, color=color)

    # Add the patch to the axes
    ax.add_patch(mpl_polygon)


def add_linestring(ax, linestring, color='r'):
    x, y = linestring.xy
    ax.plot(x, y, color=color)


def visualize_graph(graph, obstacles, multiplier=2):
    current_size = plt.rcParams["figure.figsize"]
    new_size = (current_size[0] * multiplier, current_size[1] * multiplier)
    fig, ax = plt.subplots(figsize=new_size)
    ax.set_aspect('equal')
    # Draw nodes
    for node, coords in graph.nodes.items():
        if node != 'Start':
            ax.plot(coords[0], coords[1], '2')
        else:
            ax.plot(coords[0], coords[1], 'bo')

    # Draw edges
    for edge in graph.edges:
        node1, node2, _ = edge
        coords1, coords2 = graph.nodes[node1], graph.nodes[node2]
        ax.plot([coords1[0], coords2[0]], [coords1[1], coords2[1]], 'k-')  # 'k-' for black lines

    # Draw obstacles
    for obstacle in obstacles:
        if isinstance(obstacle, Circle):
            add_circle(ax, Point(obstacle.center), obstacle.radius)
        elif isinstance(obstacle, Polygon):
            add_polygon(ax, obstacle)
        elif isinstance(obstacle, LineString):
            add_linestring(ax, obstacle)

    plt.show()
