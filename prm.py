import random
from shapely.geometry import Point, LineString, Polygon
from matplotlib.patches import Circle
from graph import MapCircle, MapPoint
from scipy.spatial import KDTree
import numpy as np

import heapq


def heuristic(a, b):
    """Calculate the Manhattan distance between two points."""
    return abs(b[0] - a[0]) + abs(b[1] - a[1])


def a_star_search(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        current = heapq.heappop(frontier)[1]

        if current == goal:
            break

        for next_node in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next_node)
            if new_cost == float('inf'):  # Skip if no direct path exists
                continue
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(graph.nodes[next_node], graph.nodes[goal])
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current

    return came_from, cost_so_far


class PRM:
    def __init__(self, graph, obstacles, start, goal, num_random_nodes=100):
        self.graph = graph
        self.obstacles = obstacles
        self.start: MapPoint = start
        self.goal: MapPoint = goal
        self.num_random_nodes = num_random_nodes
        self.x_bounds = (0, 600)  # Set to your area's bounds
        self.y_bounds = (0, 600)
        self.kd_tree = None
        self.node_list = [start]  # Include start in node list
        self.shortest_path = []

    def is_point_within_circle(self, point, circle):
        point = Point(point)
        circle_center = Point(circle.center)
        return point.distance(circle_center) <= circle.radius

    def does_line_intersect_circle(self, line, circle: MapCircle):
        circle_center = Point(circle.center)
        distance = line.distance(circle_center)
        return distance <= circle.radius

    def is_within_obstacle(self, point):
        point_shapely = Point(point)
        for obstacle in self.obstacles:
            if isinstance(obstacle, (LineString, Polygon)):
                if obstacle.contains(point_shapely):
                    return True
            elif isinstance(obstacle, Circle):  # Assuming Circle is defined
                if self.is_point_within_circle(point, obstacle):
                    return True
        return False

    def is_clear_path(self, start, end):
        line = LineString([start, end])
        for obstacle in self.obstacles:
            if isinstance(obstacle, (LineString, Polygon)):
                if line.intersects(obstacle):
                    return False
            elif isinstance(obstacle, Circle):
                if self.does_line_intersect_circle(line, obstacle):
                    return False
        return True

    def get_random_coords(self):
        """Generate random coordinates within bounds that are not within obstacles."""
        while True:
            x = random.uniform(*self.x_bounds)
            y = random.uniform(*self.y_bounds)
            point = (x, y)
            if not self.is_within_obstacle(point):
                return point

    def generate_random_nodes(self):
        """Generate random nodes within bounds, excluding obstacle areas."""
        print("Generating random nodes...")
        for _ in range(self.num_random_nodes):
            x, y = self.get_random_coords()  # Generate valid coordinates
            node_id = self.generate_node_id(Point(x, y), "random")  # Generate a unique node ID
            self.graph.add_node(node_id, MapPoint(x, y))
            self.node_list.append(MapPoint(x, y))  # Update node_list with the new node
            if len(self.graph.nodes) % 10 == 0:
                print(f"{len(self.graph.nodes)} nodes generated.")

    def generate_node_id(self, point, prefix=None):
        """Generate a unique node identifier based on a Point."""
        # Assuming point is a Point object
        if prefix:
            return f"{prefix}_{point.x}_{point.y}"
        return f"{point.x}_{point.y}"

    def connect_nodes(self):
        """Connect nodes with edges, considering obstacles."""
        print("Connecting nodes...")
        # Convert node points to a format compatible with KDTree and subsequent logic
        node_points = [(node.x, node.y) for node in self.node_list]
        self.kd_tree = KDTree(node_points)
        for i, node_coords in enumerate(node_points[:-1]):  # Adjust for direct connections
            distances, indices = self.kd_tree.query(node_coords, k=5)
            if i % 10 == 0:
                print(f"Connected {i} nodes.")
            for dist, index in zip(distances[1:], indices[1:]):  # Skip self
                if self.is_clear_path(Point(node_coords), Point(node_points[index])):
                    from_node_id = self.get_node_id_from_coords(node_coords)
                    to_node_id = self.get_node_id_from_coords(node_points[index])
                    self.graph.add_edge(from_node_id, to_node_id, dist)

    def get_node_id_from_coords(self, coords):
        """Helper method to find node ID based on coordinates."""
        # This method assumes each set of coordinates is unique to each node
        for node_id, node_coords in self.graph.nodes.items():
            if node_coords == coords:
                return node_id
        return None  # Consider how to handle the case where no node matches the coordinates

    def find_path(self):
        start_node = self.get_closest_node(self.start)
        goal_node = self.get_closest_node(self.goal)

        # Ensure A* search function is defined to use Graph's neighbors and cost methods
        came_from, _ = a_star_search(self.graph, start_node, goal_node)

        # Reconstruct the path from the output of A* search
        current = goal_node
        path = []
        while current != start_node:
            path.append(current)
            current = came_from.get(current, start_node)  # Fallback to start_node to prevent infinite loop
        path.append(start_node)
        path.reverse()

        self.shortest_path = [self.graph.nodes[node_id] for node_id in path]  # Convert node_ids to coordinates
        return self.shortest_path

    def get_closest_node(self, point):
        """Find the graph node closest to a given point."""
        point = Point(point)
        closest_node = None
        min_distance = np.inf
        for node_id, node_point in self.graph.nodes.items():
            node_point = Point(node_point)
            distance = point.distance(node_point)
            if distance < min_distance:
                closest_node = node_id
                min_distance = distance
        return closest_node

    def run(self):
        """Execute PRM to find a path from start to goal."""
        self.generate_random_nodes()
        # Add start and goal nodes
        start_id = self.generate_node_id(self.start, prefix="start")  # self.start is a Point object
        goal_id = self.generate_node_id(self.goal, prefix="goal")
        self.graph.add_node(start_id, self.start)
        self.graph.add_node(goal_id, self.goal)
        self.connect_nodes()
        # Ensure start and goal are part of self.node_list if needed
        path = self.find_path()  # (start_id, goal_id)
        return path
