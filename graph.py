import math
import random
from collections import namedtuple

from log_model import logger

MapCircle = namedtuple('MapCircle', ['center', 'radius'])
MapPoint = namedtuple('MapPoint', ['x', 'y'])
GOAL_POINT_ID = "H"

def euclidean_distance(a, b):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)


def manhattan_distance(a, b):
    """Calculate the Manhattan distance between two points."""
    return abs(b.x - a.x) + abs(b.y - a.y)


class GraphPoint:
    def __init__(self, x: float = 0.0, y: float = 0.0, point_id: str = None, point_type: str = None):
        self.x = x
        self.y = y
        self.id: str = self.generate_id_name(point_id=point_id, prefix=point_type)
        self.point_type = point_type

    def generate_id_name(self, point_id: str = None, prefix: str = ""):
        """Generate a unique node identifier based on a Point."""
        if point_id:
            return point_id
        if prefix:
            return f"{prefix}_{self.x}_{self.y}"
        return f"{self.x}_{self.y}"

    def same_coords(self, point_to_compare):
        return self.x == point_to_compare.x and self.y == point_to_compare.y

    def __repr__(self):
        return f"<'{self.id}' point {self.x, self.y}>"


graph_rooms_nodes = {
    'A': GraphPoint(60, 416, point_id='A'),
    'B': GraphPoint(138, 564, point_id='B'),
    'C': GraphPoint(215, 438, point_id='C'),
    'D': GraphPoint(313, 438, point_id='D'),
    'E': GraphPoint(490, 406, point_id='E'),
    'F': GraphPoint(75, 162, point_id='F'),
    'G': GraphPoint(262, 196, point_id='G'),
    'H': GraphPoint(260, 130, point_id='H'),
    'I': GraphPoint(360, 176, point_id='I'),
    'J': GraphPoint(534, 196, point_id='J'),
    'K': GraphPoint(534, 110, point_id='K'),
    'L': GraphPoint(534, 27, point_id='L'),
    'M': GraphPoint(420, 27, point_id='M'),
    'N': GraphPoint(75, 45, point_id='N'),
    'O': GraphPoint(312, 338, point_id='O'),
}


def graph_general_nodes(goal_id: str = None):
    goal_id = goal_id or random.choice(list(graph_rooms_nodes.keys()))
    return {
        'start': GraphPoint(202, 0, point_id='start'),
        'goal': GraphPoint(graph_rooms_nodes.get(goal_id).x, graph_rooms_nodes.get(goal_id).y, point_id='goal'),
    }


class Graph:
    def __init__(self, nodes: dict[GraphPoint]):
        self.nodes: dict[GraphPoint] = nodes
        self.room_nodes = self.get_room_nodes()
        self.edges = {}

    def get_room_nodes(self):
        return [v for k, v in self.nodes.items() if len(v.id) == 1]

    def add_edge(self, from_node_id, to_node_id, cost):
        if from_node_id and to_node_id:
            if from_node_id == to_node_id:
                # Avoid adding an edge from a node to itself
                logger.debug(f"Avoiding self-connection for {from_node_id}")
                return
            if from_node_id in self.edges:
                self.edges[from_node_id].append((to_node_id, cost))
            else:
                self.edges[from_node_id] = [(to_node_id, cost)]

    def neighbors(self, node_id):
        return [neighbor for neighbor, _ in self.edges.get(node_id, [])]

    def cost(self, from_node, to_node):
        if from_node in self.edges:
            for neighbor, cost in self.edges[from_node]:
                if neighbor == to_node:
                    logger.debug(f"Cost found from {from_node} to {to_node}: {cost}")
                    return cost
        logger.error(f"No edge found from {from_node} to {to_node}")
        return float('inf')

    def add_node(self, point: GraphPoint):
        # Check if node with same id or coordinates already exists
        existing_node = self.nodes.get(point.id)
        if existing_node and (existing_node.x == point.x and existing_node.y == point.y):
            logger.warn(f"Node {point.id} with coordinates ({point.x}, {point.y}) already exists.")
            return

        # Check if any node with same coordinates exists
        if any(node.x == point.x and node.y == point.y for node in self.nodes.values()):
            logger.warn(f"Node with coordinates ({point.x}, {point.y}) already exists.")
            return

        logger.debug(f"Adding node: {point.id}")
        node_id = point.id
        self.nodes[node_id] = point
        if node_id not in self.edges:
            self.edges[node_id] = []

    def remove_node(self, point):
        if point:
            to_remove = [node_id for node_id, node in self.nodes.items() if node.x == point.x and node.y == point.y]
            for node_id in to_remove:
                logger.info(f"Removing node: {node_id}")
                del self.nodes[node_id]
                # Also remove any edges associated with this node
                self.edges = {from_node: edges for from_node, edges in self.edges.items() if from_node != node_id}
                self.edges = {from_node: [(to_node, cost) for to_node, cost in edges if to_node != node_id] for
                              from_node, edges in self.edges.items()}
