import random
from shapely.geometry import Point, LineString, Polygon
from matplotlib.patches import Circle
from graph import MapCircle
from scipy.spatial import KDTree
import numpy as np


class PRM:
    def __init__(self, graph, obstacles, start, goal):
        self.graph = graph
        self.obstacles = obstacles
        self.start = Point(start)
        self.goal = Point(goal)
        self.num_random_nodes = 100  # Adjust based on your requirement
        self.x_bounds = (0, 600)  # Set to your area's bounds
        self.y_bounds = (0, 600)
        self.kd_tree = None
        self.node_list = [start]  # Include start in node list

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

    def generate_random_nodes(self):
        """Generate random nodes within bounds, excluding obstacle areas."""
        print("Generating random nodes...")
        while len(self.node_list) < self.num_random_nodes + 1:  # +1 for goal node
            x, y = random.uniform(*self.x_bounds), random.uniform(*self.y_bounds)
            point = Point(x, y)
            if not self.is_within_obstacle(point):
                self.node_list.append((x, y))
                if len(self.node_list) % 10 == 0:  # Example: print a message every 10 nodes
                    print(f"{len(self.node_list)} nodes generated.")
        self.node_list.append((self.goal.x, self.goal.y))  # Add goal to node list
        print(f"Done generating random nodes... {len(self.node_list)}")

    def connect_nodes(self):
        """Connect nodes with edges, considering obstacles."""
        print("Connecting nodes...")
        self.kd_tree = KDTree(self.node_list)
        for i, node in enumerate(self.node_list[:-1]):  # Exclude goal for connection
            distances, indices = self.kd_tree.query(node, k=5)  # Adjust k based on density
            if i % 10 == 0:  # Example: visualize every 10 connections
                print(f"Connected {i} nodes.")
            for dist, index in zip(distances[1:], indices[1:]):  # Exclude self
                if self.is_clear_path(Point(node), Point(self.node_list[index])):
                    self.graph.add_edge(i, index, dist)

    def find_path(self):
        """Find path from start to goal using A*."""
        # Placeholder for A* algorithm, returning path as a list of points
        return []

    def run(self):
        """Execute PRM to find a path from start to goal."""
        self.generate_random_nodes()
        self.connect_nodes()
        path = self.find_path()
        return path
