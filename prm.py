import math
import random
from enum import Enum

from shapely.geometry import Point, LineString, Polygon
from matplotlib.patches import Circle
from graph import MapCircle, MapPoint, GraphPoint
from scipy.spatial import KDTree
import numpy as np

import heapq

from log_model import logger

K_RADIUS = 100


class SearchType(Enum):
    A_STAR_SEARCH = "a_star_search"
    DIJKSTRA = "dijkstra"


def euclidean_distance(a, b):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)


def manhattan_distance(a, b):
    """Calculate the Manhattan distance between two points."""
    return abs(b.x - a.x) + abs(b.y - a.y)


def a_star_search(graph, start, goal, type_of_heuristic=manhattan_distance):
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
                priority = new_cost + type_of_heuristic(graph.nodes[next_node], graph.nodes[goal])
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
                logger.debug(f"Updating node {next_node} with new cost and priority.")

    logger.debug(f"Final came_from: {came_from}")
    return came_from, cost_so_far


def dijkstra_search(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))  # Priority queue; cost from start to node
    came_from = {start: None}  # Track the path
    cost_so_far = {start: 0}  # Cost from start to the node

    while frontier:
        current_priority, current = heapq.heappop(frontier)
        if current == goal:
            break

        for next_node in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next_node)

            if new_cost < cost_so_far.get(next_node, float('inf')):
                cost_so_far[next_node] = new_cost
                priority = new_cost  # Priority is just the new cost, no heuristic
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current

    return came_from, cost_so_far


class PRM:
    shortest_path_cost = -1

    def __init__(self, graph, obstacles, start, goal, num_random_nodes=100, search_type=SearchType.A_STAR_SEARCH):
        self.node_list = None
        self.graph = graph
        self.obstacles = obstacles
        self.start: GraphPoint = start
        self.goal: GraphPoint = goal
        self.num_random_nodes = num_random_nodes
        self.x_bounds = (0, 600)
        self.y_bounds = (0, 600)
        self.kd_tree = None
        self.initialize_nodes_list()
        self.shortest_path = []
        self.search_type = search_type
        self.graph.add_node(self.start)
        self.graph.add_node(self.goal)

    def is_point_within_circle(self, point, circle):
        point = Point(point)
        circle_center = Point(circle.center)
        return point.distance(circle_center) <= circle.radius

    def does_line_intersect_circle(self, line, circle):
        circle_center = Point(circle.center)
        distance = line.distance(circle_center)
        return distance <= circle.radius

    def is_within_obstacle(self, point):
        point_shapely = Point(point)
        for obstacle in self.obstacles:
            if isinstance(obstacle, (LineString, Polygon)):
                if obstacle.contains(point_shapely):
                    return True
            elif isinstance(obstacle, Circle):
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

    def connect_nodes(self, type_of_distance=manhattan_distance):
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
                        self.graph.add_edge(from_node.id, to_node.id, type_of_distance(from_node, to_node))
                        logger.debug(f"Connected {from_node.id} to {to_node.id}")

    def get_node_id_from_coords(self, coords):
        """Helper method to find node ID based on coordinates."""
        for node_id, point in self.graph.nodes.items():
            node_coords = (point.x, point.y)
            if node_coords == coords:
                return node_id
        return None

    def find_path(self):
        logger.info(f"Finding shortest path with "
                    f"{'A*' if self.search_type == SearchType.A_STAR_SEARCH else 'Dijkstra'} algorithm...")
        start_node = self.start.id
        goal_node = self.goal.id
        if start_node == goal_node:
            # Handle the case where the start node and goal node are the same
            return [self.graph.nodes[start_node]]

        if self.search_type == SearchType.A_STAR_SEARCH:
            came_from, cost_so_far = a_star_search(self.graph, start_node, goal_node)
        else:
            came_from, cost_so_far = dijkstra_search(self.graph, start_node, goal_node)
        if goal_node not in came_from:
            # Handle the case where no path is found
            logger.warn(f"No shortest path found")
            return []

        # Reconstruct the path
        current = goal_node
        path = []
        while current != start_node:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                # Handle the case where a path to the current node wasn't found
                logger.warn(f"No shortest path found")
                return []
        path.append(start_node)
        path.reverse()

        self.shortest_path_cost = cost_so_far[goal_node]
        logger.info(f"Found shortest path, {self.shortest_path_cost=}")
        self.shortest_path = [self.graph.nodes[node_id] for node_id in path]
        return self.shortest_path

    def get_closest_node(self, point: GraphPoint, type_of_distance=manhattan_distance):
        """Find the graph node closest to a given point."""
        closest_node = None
        min_distance = np.inf
        for node_id, node_point in self.graph.nodes.items():
            if point.same_coords(node_point):
                continue
            distance = type_of_distance(node_point, point)
            if distance < min_distance:
                closest_node = node_id
                min_distance = distance
        logger.info(f"Closest node to {point}: {closest_node} at distance {min_distance}")
        return closest_node

    def run(self):
        """Execute PRM to find a path from start to goal."""
        self.generate_random_nodes()
        self.connect_nodes()
        shortest_path = self.find_path()  # (start_id, goal_id)
        return shortest_path

    def initialize_nodes_list(self):
        self.node_list = [self.start]
        self.node_list.extend([point for node_id, point in self.graph.nodes.items()])