nodes = {
    'A': (60, 416),
    'B': (138, 564),
    'C': (215, 438),
    'D': (313, 438),
    'E': (490, 406),
    'F': (75, 162),
    'G': (262, 196),
    'H': (260, 130),
    'I': (360, 176),
    'J': (534, 196),
    'K': (534, 110),
    'L': (534, 27),
    'M': (420, 27),
    'N': (75, 45),
    'O': (312, 338)
}


class Graph:
    def __init__(self):
        self.nodes = nodes
        self.edges = []

    def add_node(self, label, coordinates):
        self.nodes[label] = coordinates

    def add_edge(self, node1, node2, weight):
        self.edges.append((node1, node2, weight))
