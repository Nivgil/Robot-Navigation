from udpclient import RClient
from position import Position
import navigation
import math
import time
import numpy as np
from graph import Node, Edge, Graph


def get_robot(mode, *args, **kwargs):
    return {'simple': RobotSimple, 'test_sensing': ObstaclesTesting, 'road_map': RobotRoadMap}[mode](*args, **kwargs)


def valid_position(position):
    if position.x > 150 or position.x < -240 or position.y < -300 or position.y > 280:
        return False
    if position.x == 0 and position.y == 0:
        return False
    return True


class Robot(object):
    def __init__(self, ip_address="192.168.1.155", default_speed=450):
        self._robot = RClient(ip_address, 2777)
        self._robot.connect()
        self._default_speed = default_speed
        self._position = Position(x=0, y=0, dx=0, dy=0)
        self.get_position()

    def get_position(self):
        sample = self._robot.sense()
        self._position.set_position(x=sample[0], y=sample[1], dx=sample[2], dy=sample[3])
        while valid_position(self._position) is False:
            self._robot.drive(-self._default_speed, -self._default_speed)
            sample = self._robot.sense()
            self._position.set_position(x=sample[0], y=sample[1], dx=sample[2], dy=sample[3])
            time.sleep(0.1)
            print('Invalid Position')
        return self._position

    def get_sensing(self):
        sample = self._robot.sense()
        return sample[4:]

    @staticmethod
    def hard_turn(alpha):
        return abs(alpha) >= 50

    def _wheel_speed(self, k, alpha, distance, sign):
        if distance < 100:
            speed_factor = 0.3
        else:
            speed_factor = 1
        if sign == '+':
            speed = self._default_speed * speed_factor + k * abs(alpha)
        if sign == '-':
            speed = self._default_speed * speed_factor - k * abs(alpha)
        speed = speed if speed > 300 else 300
        speed = speed if speed < 1000 else 1000
        return speed

    def get_speed(self, k, alpha, distance):
        if abs(alpha) >= 50:  # deviation is bigger than 90 degrees
            if alpha > 0:
                # Turn right
                left_motor = self._default_speed * 0.8 + k * abs(alpha) / 180
                right_motor = -self._default_speed * 0.8 - k * abs(alpha) / 180
            else:
                # Turn left
                left_motor = -self._default_speed * 0.8 - k * abs(alpha) / 180
                right_motor = self._default_speed * 0.8 + k * abs(alpha) / 180
        elif alpha > 0:
            # Turn right
            left_motor = self._wheel_speed(k, alpha, distance, '+')
            right_motor = self._wheel_speed(k, alpha, distance, '-')
        else:
            # Turn left
            left_motor = self._wheel_speed(k, alpha, distance, '-')
            right_motor = self._wheel_speed(k, alpha, distance, '+')
        return left_motor, right_motor

    def navigate(self, destination):
        raise NotImplementedError

    def terminate(self):
        self._robot.terminate()


class RobotSimple(Robot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def navigate(self, destination):
        k = 25
        threshold = 8

        current_position = self.get_position()
        distance, direction = navigation.calc_vector(current_position, destination)
        while distance > threshold:
            current_position = self.get_position()
            distance, direction = navigation.calc_vector(current_position, destination)
            if distance < threshold:
                self._robot.drive(-450, -450)
                return True
            alpha = navigation.angle((current_position.dx, current_position.dy), direction)
            # print('-------------------------')
            # print('distance [{}], alpha [{}] '.format(distance, alpha))
            left_motor, right_motor = self.get_speed(k, alpha, distance)
            self._robot.drive(left_motor, right_motor)
            time.sleep(0.25)
            if self.hard_turn(alpha) is True:
                time.sleep(0.25)
            # print('Left Motor [{}], Right Motor [{}]'.format(left_motor, right_motor))
            distance, direction = navigation.calc_vector(current_position, destination)
        self._robot.drive(-500, -500)
        return distance < threshold


class ObstaclesTesting(Robot):
    def __init__(self):
        super().__init__()

    def navigate(self, destination):
        counter = 200
        while counter > 0:
            sensing = self.get_sensing()
            counter -= 1
            print(sensing)
            time.sleep(0.5)


class RobotRoadMap(Robot):
    def __init__(self):
        super().__init__()

    def navigate(self, final_destination):

        # Init graph with current location and final destination
        current_position = self.get_position()
        current_position_id = str(np.floor(current_position.x)) + ',' + str(np.floor(current_position.y))
        final_destination_id = str(np.floor(final_destination.x)) + ',' + str(np.floor(final_destination.y))
        current_position_node = Node(current_position_id, np.array((current_position.x, current_position.y)))
        final_destination_node = Node(final_destination_id, np.array((final_destination.x, final_destination.y)))

        road_map = Graph()
        road_map.add_node(final_destination_node)
        road_map.add_node(current_position_node)
        road_map.add_edge(current_position_node, final_destination_node)

        k = 25 # speed factor in turn with movement
        threshold = 8
        collision_th = 15
        radius = 50 # radius for creating new nodes
        detour_number = 1 # number of nodes to be created
        final_distance, direction = navigation.calc_vector(current_position, final_destination)

        while final_distance > threshold:
            radius_factor = 2
            current_position = self.get_position()
            closest_node_id, dist = self.find_closest_node(road_map, current_position)
            visited, path = road_map.shortest_path(closest_node_id, final_destination_id)
            while path.get(final_destination_id) is None:  # no path exists
                print('No Path Found, Creating Detour Nodes')
                self.add_detour_nodes(road_map, road_map.get_node(closest_node_id), final_destination_node,
                                      radius_factor * radius, detour_number)
                self.update_graph(road_map, obstacles_coord, collision_th)
                visited, path = road_map.shortest_path(closest_node_id, final_destination_id)
                radius_factor += 0.5
                if radius_factor == 10:
                    return False
            print('Path is {}'.format(path))
            closest_node_id, dist = self.find_closest_node(road_map, current_position)
            next_node_position, departure_node_position = self.get_mid_destination(path, final_destination_id,
                                                                                   closest_node_id)
            distance, direction = navigation.calc_vector(current_position, next_node_position)
            while distance > threshold:
                current_position = self.get_position()
                distance, direction = navigation.calc_vector(current_position, next_node_position)
                alpha = navigation.angle((current_position.dx, current_position.dy), direction)
                print('-------------------------')
                print('Navigating to Vertex {}, Distance {}, Alpha {}'.format(next_node_position, distance, alpha))
                # print('distance [{}], alpha [{}] '.format(distance, alpha))
                left_motor, right_motor = self.get_speed(k, alpha, distance)
                self._robot.drive(left_motor, right_motor)
                time.sleep(0.25)
                if self.hard_turn(alpha) is True:
                    time.sleep(0.25)
                sonar_sample = self.get_sensing()
                radius = max(sonar_sample)
                current_position = self.get_position()
                obstacles_coord = navigation.get_obstacles(sonar_sample, current_position)
                if len(obstacles_coord) > 0:
                    collision_detected = self.update_graph(road_map, obstacles_coord, collision_th)
                    if collision_detected is True:
                        current_position = self.get_position()
                        closest_node_id, dist = self.find_closest_node(road_map, current_position)
                        print('Collision Detected closest node in {} - '.format(dist), end='')
                        if dist > collision_th:
                            current_position_id = str(np.floor(current_position.x)) + ',' + str(
                                np.floor(current_position.y))
                            current_position_node = Node(current_position_id,
                                                         np.array((current_position.x, current_position.y)))
                            road_map.add_node(current_position_node)
                            departure_node_id = navigation.coordinates_to_node_id(departure_node_position)
                            road_map.add_edge(current_position_node, road_map.get_node(departure_node_id))
                            road_map.add_edge(current_position_node, final_destination_node)
                            print('Creating Node at: {}'.format(current_position_id))
                        self.update_graph(road_map, obstacles_coord, collision_th)
                        break
                # print('Left Motor [{}], Right Motor [{}]'.format(left_motor, right_motor))
                current_position = self.get_position()
                distance, direction = navigation.calc_vector(current_position, next_node_position)
            print('Arrived to Road Vertex at: {}'.format(current_position))
            # self._robot.drive(-450, -450)
            current_position = self.get_position()
            final_distance, direction = navigation.calc_vector(current_position, final_destination)
        self._robot.drive(-450, -450)
        return final_distance < threshold

    @staticmethod
    def get_mid_destination(path, destination_id, closest_node_id):
        temp_id = destination_id
        while temp_id is not None:
            mid_destination_id = temp_id
            temp_id = path.get(mid_destination_id)
            if temp_id == closest_node_id:
                mid_destination = navigation.node_id_to_coordinates(mid_destination_id)
                temp_coord = navigation.node_id_to_coordinates(temp_id)
                return Position(mid_destination[0], mid_destination[1]), Position(temp_coord[0], temp_coord[1])

    @staticmethod
    def update_graph(graph, obstacles, collision_th):
        collision_detected = False
        for edge_id, edge in graph.get_edges():
            coord_1, coord_2 = edge.get_coordinates()
            for obstacle in obstacles:
                dist = navigation.distance_point_line(obstacle[0], obstacle[1], coord_1[0], coord_1[1], coord_2[0],
                                                      coord_2[1])
                if dist <= collision_th:
                    edge.set_weight(math.inf)
                    collision_detected = True
                    break
        return collision_detected

    @staticmethod
    def find_closest_node(graph, position):
        min_dist = math.inf
        closest_node_id = None
        for node_id, node in graph.get_nodes():
            dist = np.linalg.norm(np.array((position.x, position.y) - np.array(node.get_coordinates())))
            if dist < min_dist:
                min_dist = dist
                closest_node_id = node_id
        return closest_node_id, min_dist

    @staticmethod
    def add_detour_nodes(graph, current_node, final_node, radius, detour_number):
        if radius > 80:
            radius = 80
        x = current_node.get_coordinates()[0]
        y = current_node.get_coordinates()[1]
        for angle in np.linspace(0, 360 - 360 / detour_number, detour_number):
            x_coord = x - radius * math.sin((((angle + 90) % 360) * math.pi) / 180)
            y_coord = y + radius * math.cos((((angle + 90) % 360) * math.pi) / 180)
            if valid_position(Position(x_coord, y_coord)):
                node_id = str(np.floor(x_coord)) + ',' + str(np.floor(y_coord))
                detour_node = Node(node_id, (x_coord, y_coord))
                graph.add_node(detour_node)
                graph.add_edge(detour_node, current_node)
                graph.add_edge(detour_node, final_node)
