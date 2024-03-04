import math
import random

from shapely.geometry import Point, LineString, Polygon
from matplotlib.patches import Circle
from graph import MapCircle, MapPoint, GraphPoint
from scipy.spatial import KDTree
import numpy as np

import heapq

from log_model import logger

K_RADIUS = 100


def euclidean_heuristic(a, b):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)


def a_star_search(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        current_priority, current = heapq.heappop(frontier)
        logger.debug(f"Current node: {current}, Priority: {current_priority}")

        if current == goal:
            logger.debug("Goal reached!")
            break

        for next_node in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next_node)
            logger.debug(f"Considering: {next_node}, New cost: {new_cost}")

            if new_cost < cost_so_far.get(next_node, float('inf')):
                cost_so_far[next_node] = new_cost
                priority = new_cost + euclidean_heuristic(graph.nodes[next_node], graph.nodes[goal])
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
                logger.debug(f"Updating node {next_node} with new cost and priority.")

    logger.info(f"Final came_from: {came_from}")
    return came_from, cost_so_far


class PRM:
    def __init__(self, graph, obstacles, start, goal, num_random_nodes=100):
        self.graph = graph
        self.obstacles = obstacles
        self.start: GraphPoint = start
        self.goal: GraphPoint = goal
        self.num_random_nodes = num_random_nodes
        self.x_bounds = (0, 600)  # Set to your area's bounds
        self.y_bounds = (0, 600)
        self.kd_tree = None
        self.node_list = [start]  # Include start in node list
        self.node_list.extend([point for node_id, point in graph.nodes.items()])
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
            x = round(random.uniform(*self.x_bounds), 3)
            y = round(random.uniform(*self.y_bounds), 3)
            point = (x, y)
            if not self.is_within_obstacle(point):
                return point

    def generate_random_nodes(self):
        """Generate random nodes within bounds, excluding obstacle areas."""
        logger.info("Generating random nodes...")
        for _ in range(self.num_random_nodes):
            x, y = self.get_random_coords()  # Generate valid coordinates
            point = GraphPoint(x, y, point_type="random")
            self.graph.add_node(point)
            self.node_list.append(point)  # Update node_list with the new node
            if len(self.graph.nodes) % 10 == 0:
                logger.debug(f"{len(self.graph.nodes)} nodes generated.")

    def connect_nodes(self):
        """Connect nodes with edges, considering obstacles."""
        logger.info("Connecting nodes...")
        # Convert node points to a format compatible with KDTree and subsequent logic
        node_points = [(node.x, node.y) for node in self.node_list]
        self.kd_tree = KDTree(node_points)
        for i, node_coords in enumerate(node_points):
            # Use query_ball_point for a radius search
            indices = self.kd_tree.query_ball_point(node_coords, r=K_RADIUS)
            for index in indices:
                if index != i:  # Avoid connecting the node to itself
                    from_node = self.node_list[i]
                    to_node = self.node_list[index]
                    if self.is_clear_path(Point(from_node.x, from_node.y), Point(to_node.x, to_node.y)):
                        # Connect the nodes in the graph
                        self.graph.add_edge(from_node.id, to_node.id, euclidean_heuristic(from_node, to_node))
                        logger.debug(f"Connected {from_node.id} to {to_node.id}")

    def get_node_id_from_coords(self, coords):
        """Helper method to find node ID based on coordinates."""
        # This method assumes each set of coordinates is unique to each node
        for node_id, point in self.graph.nodes.items():
            node_coords = (point.x, point.y)
            if node_coords == coords:
                return node_id
        return None  # Consider how to handle the case where no node matches the coordinates

    def find_path(self):
        logger.info("Finding shortest path...")
        start_node = self.start.id
        goal_node = self.goal.id
        if start_node == goal_node:
            # Handle the case where the start node and goal node are the same
            return [self.graph.nodes[start_node]]

        came_from, _ = a_star_search(self.graph, start_node, goal_node)
        if goal_node not in came_from:
            # Handle the case where no path is found
            return []

        # Reconstruct the path
        current = goal_node
        path = []
        while current != start_node:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                # Handle the case where a path to the current node wasn't found
                return []
        path.append(start_node)
        path.reverse()

        self.shortest_path = [self.graph.nodes[node_id] for node_id in path]
        return self.shortest_path

    def get_closest_node(self, point: GraphPoint):
        """Find the graph node closest to a given point."""
        closest_node = None
        min_distance = np.inf
        for node_id, node_point in self.graph.nodes.items():
            if point.same_coords(node_point):
                continue
            distance = euclidean_heuristic(node_point, point)
            if distance < min_distance:
                closest_node = node_id
                min_distance = distance
        logger.info(f"Closest node to {point}: {closest_node} at distance {min_distance}")
        return closest_node

    def run(self):
        """Execute PRM to find a path from start to goal."""
        self.generate_random_nodes()
        # Add start and goal nodes
        self.graph.add_node(self.start)
        self.graph.add_node(self.goal)
        self.connect_nodes()
        # Ensure start and goal are part of self.node_list if needed
        shortest_path = self.find_path()  # (start_id, goal_id)
        return shortest_path
