class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = []

    def add_node(self, label, coordinates):
        self.nodes[label] = coordinates

    def add_edge(self, node1, node2, weight):
        self.edges.append((node1, node2, weight))
