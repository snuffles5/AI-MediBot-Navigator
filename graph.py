from collections import namedtuple

MapCircle = namedtuple('MapCircle', ['center', 'radius'])
MapPoint = namedtuple('MapPoint', ['x', 'y'])

graph_general_nodes = {
    'start': MapPoint(202, 0),
}
graph_rooms_nodes = {
    'A': MapPoint(60, 416),
    'B': MapPoint(138, 564),
    'C': MapPoint(215, 438),
    'D': MapPoint(313, 438),
    'E': MapPoint(490, 406),
    'F': MapPoint(75, 162),
    'G': MapPoint(262, 196),
    'H': MapPoint(260, 130),
    'I': MapPoint(360, 176),
    'J': MapPoint(534, 196),
    'K': MapPoint(534, 110),
    'L': MapPoint(534, 27),
    'M': MapPoint(420, 27),
    'N': MapPoint(75, 45),
    'O': MapPoint(312, 338),
}


class Graph:
    def __init__(self, nodes: dict[tuple]):
        self.nodes = nodes  # node_id: (x, y)
        self.edges = {}  # node_id: [(neighbor_id, cost), ...]

    def add_edge(self, from_node, to_node, cost):
        if from_node in self.edges:
            self.edges[from_node].append((to_node, cost))
        else:
            self.edges[from_node] = [(to_node, cost)]

    def neighbors(self, node_id):
        return self.edges[node_id]

    def cost(self, from_node, to_node):
        if from_node in self.edges:
            for neighbor, cost in self.edges[from_node]:
                if neighbor == to_node:
                    return cost
        return float('inf')  # Return infinity if there's no direct path

    def add_node(self, node_id, coordinates: MapPoint):
        self.nodes[node_id] = coordinates
        if node_id not in self.edges:
            self.edges[node_id] = []  # Initialize with an empty list of edges

