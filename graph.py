import numpy as np
import math
from position import Position


class Graph(object):
    def __init__(self):
        self._nodes = dict()
        self._edges = dict()

    def add_node(self, node):
        node_id = node.get_id()
        if self._nodes.get(node_id) is None:
            self._nodes[node_id] = node

    def add_edge(self, node_1, node_2):
        new_edge_1 = Edge(node_1, node_2)
        new_edge_2 = Edge(node_2, node_1)
        edge_id_1 = new_edge_1.get_id()
        edge_id_2 = new_edge_2.get_id()
        if self._edges.get(edge_id_1) is None:
            self._edges[edge_id_1] = new_edge_1
            self._edges[edge_id_2] = new_edge_2
            self._nodes.get(node_1.get_id()).add_neighbor(node_2)
            self._nodes.get(node_2.get_id()).add_neighbor(node_1)

    def get_edge(self, edge_id):
        return self._edges.get(edge_id)

    def get_node(self, node_id):
        return self._nodes.get(node_id)

    def delete_node(self, node_id):
        node = self._nodes.get(node_id)
        for neighbor_id, neighbor in node.get_neighbors():
            neighbor.delete_neighbor(node_id)
            edge_id_1 = int(str(node) + str(neighbor))
            edge_id_2 = int(str(neighbor) + str(node))
            self._edges.pop(edge_id_1, None)
            self._edges.pop(edge_id_2, None)
        self._nodes.pop(node, None)

    def update_weight(self, edge_id, new_weight):
        edge = self._edges.get(edge_id)
        if edge is not None:
            edge.set_weight(new_weight)

    def get_edges(self):
        return self._edges.items()

    def get_nodes(self):
        return self._nodes.items()

    def shortest_path(self, initial_node_id, destination_node_id):
        visited = {initial_node_id: 0}
        path = {}

        nodes = set(self._nodes)

        while nodes:
            min_node = None
            for node in nodes:
                if node in visited:
                    if min_node is None:
                        min_node = node
                    elif visited[node] < visited[min_node]:
                        min_node = node

            if min_node is None:
                break

            nodes.remove(min_node)
            current_weight = visited[min_node]

            for neighbor_id, neighbor in self._nodes.get(min_node).get_neighbors():
                edge = self._edges.get(min_node + '_' + neighbor_id)
                weight = current_weight + edge.get_weight()
                if weight != math.inf and (neighbor_id not in visited or weight < visited[neighbor_id]):
                    visited[neighbor_id] = weight
                    path[neighbor_id] = min_node
                    if neighbor_id == destination_node_id:
                        return visited, path
        return visited, path

    # Create nodes to map all graph, the difference would be dif on x-axis or y-axis (may be changed)
    def create_nodes1(self, destination):
        dif = 30
        x = 60
        # Different logic for 60<x<120 and -200<x<208 (may be split for more cases)
        while x <= 125:
            y = -125
            while y <= 100:
                new_node = Node(str(x) + ',' + str(y), np.array((x, y)))
                # print("new node coordinates are: " + str(new_node.get_coordinates()))
                self.add_node(new_node)
                y += dif
            x += dif
        # case 2: -200<x<60 , -208<y<255
        dif = 30
        x1 = -215
        while x1 <= 125:
            y1 = -250
            while y1 <= 255:
                new_node = Node(str(x1) + ',' + str(y1), np.array((x1, y1)))
                self.add_node(new_node)
                # print("new node coordinates are: " + str(new_node.get_coordinates()))
                y1 += dif
            x1 += dif
        # case 3: -200<x<0, y
        destination_node = Node(str(destination.x) + ',' + str(destination.y), (destination.x, destination.y))
        self.add_node(destination_node)
        self.add_node(Node(str(-110) + ',' + str(230), np.array((-110, 230))))
        self.add_node(Node(str(-100) + ',' + str(220), np.array((-100, 220))))
        print("final destination id is " + str(destination_node.get_id()))
        return destination_node.get_id(), destination_node # return destination node id

    # Create edges between all the nodes that are within distance 30 (may be changed by max_distance value)
    def create_edges1(self,dest_node):
        max_distance = 50
        for node_i in self.get_nodes():
            for node_j in self.get_nodes():
                if node_i != node_j:
                    coordinates_i = node_i[1].get_coordinates()
                    coordinates_j = node_j[1].get_coordinates()
                    distance = np.linalg.norm(np.array(coordinates_i) - np.array(coordinates_j))
                    if distance < max_distance:
                        if node_i[1] == dest_node:
                            print("Connected destination node to node {}".format(node_j[1].get_coordinates()))
                        self.add_edge(node_i[1], node_j[1])


class Node(object):
    def __init__(self, node_id=None, coordinates=None):
        self._id = node_id
        self._coordinates = coordinates
        self._neighbors = dict()

    def get_id(self):
        return self._id

    def get_coordinates(self):
        return self._coordinates

    def get_neighbors(self):
        return self._neighbors.items()

    def add_neighbor(self, node):
        node_id = node.get_id()
        if self._neighbors.get(node_id) is None:
            self._neighbors[node_id] = node

    def delete_neighbor(self, node_id):
        self._neighbors.pop(node_id, None)


class Edge(object):
    def __init__(self, node_1=None, node_2=None):
        self._id = str(node_1.get_id()) + '_' + str(node_2.get_id())
        self._weight = np.linalg.norm(np.array(node_1.get_coordinates()) - np.array(node_2.get_coordinates()))
        self._coord_1 = node_1.get_coordinates()
        self._coord_2 = node_2.get_coordinates()

    def get_id(self):
        return self._id

    def get_weight(self):
        return self._weight

    def get_coordinates(self):
        return self._coord_1, self._coord_2

    def set_weight(self, weight):
        # print("Setting new weight to edge {}".format(self._id))
        self._weight = weight