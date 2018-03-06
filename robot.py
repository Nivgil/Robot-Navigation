from udpclient import RClient
from position import Position
import navigation
import math
import time
import numpy as np
from graph import Node, Edge, Graph
from obstacles import Obstacles


def get_robot(mode, *args, **kwargs):
    return {'simple': RobotSimple, 'test_sensing': ObstaclesTesting, 'road_map': RobotRoadMap, 'mapped': RobotMapped}[
        mode](*args, **kwargs)


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
            # self._robot.drive(-self._default_speed, -self._default_speed)
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
        deviation_threshold = 50
        if distance < 50:
            deviation_threshold = 20
        if abs(alpha) >= deviation_threshold:  # deviation is bigger than 'deviation_threshold' degrees
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
        self._collision_th = 25

    def navigate(self, final_destination):

        # Init graph with current location and final destination
        current_position = self.get_position()
        current_position_node = Node(current_position.x, current_position.y)
        final_destination_node = Node(final_destination.x, final_destination.y)
        final_destination_id = final_destination_node.get_id()

        road_map = Graph()
        road_map.add_node(final_destination_node)
        road_map.add_node(current_position_node)
        road_map.add_edge(current_position_node, final_destination_node)

        obstacles = Obstacles()

        k = 25  # speed factor in turn with movement
        threshold = 8
        radius = 40  # radius for creating new nodes
        final_distance, direction = navigation.calc_vector(current_position, final_destination)

        while final_distance > threshold:
            radius_factor = 1.5
            angle = 60
            current_position = self.get_position()
            closest_node_id, dist = self.find_closest_node(road_map, current_position)
            visited, path = road_map.shortest_path(closest_node_id, final_destination_id)
            while path.get(final_destination_id) is None:  # no path exists
                print('No Path Found, Creating Detour Nodes')
                new_nodes_flag = self.add_detour_nodes(road_map, road_map.get_node(closest_node_id),
                                                       final_destination_node, angle, radius_factor * radius)
                if new_nodes_flag is False:
                    return False
                self.update_graph(road_map, obstacles.get_obstacles(), self._collision_th)
                visited, path = road_map.shortest_path(closest_node_id, final_destination_id)
                angle = (angle + 10) % 360
            print('Path is {}'.format(path))
            closest_node_id, dist = self.find_closest_node(road_map, current_position)
            next_node_position, departure_node_position = self.get_mid_destination(path, final_destination_id,
                                                                                   closest_node_id)
            distance, direction = navigation.calc_vector(current_position, next_node_position)
            while distance > threshold:

                self.avoid_collision()
                sonar_sample = self.get_sensing()
                current_position = self.get_position()
                obstacles.add_obstacles(current_position, sonar_sample)
                collision_detected = self.update_graph(road_map, obstacles.get_obstacles(), self._collision_th)
                if collision_detected is True:
                    closest_node_id, dist = self.find_closest_node(road_map, current_position)
                    print('Collision Detected closest node in {} - '.format(dist), end='')
                    if dist > self._collision_th:
                        current_position_node = Node(current_position.x, current_position.y)
                        road_map.add_node(current_position_node)
                        departure_node_id = navigation.coordinates_to_node_id(departure_node_position)
                        road_map.add_edge(current_position_node, road_map.get_node(departure_node_id))
                        road_map.add_edge(current_position_node, final_destination_node)
                        print('Creating Node at: {}'.format(current_position_node))
                    break

                distance, direction = navigation.calc_vector(current_position, next_node_position)
                alpha = navigation.angle((current_position.dx, current_position.dy), direction)
                print('----------------------------------------------')
                print('Navigating to Vertex {}, Distance {}, Alpha {}'.format(next_node_position, distance, alpha))
                left_motor, right_motor = self.get_speed(k, alpha, distance)
                self._robot.drive(left_motor, right_motor)
                sleep_time = 0.25 if distance > 100 else 0.5
                time.sleep(sleep_time)
                if self.hard_turn(alpha) is True:
                    time.sleep(sleep_time)

                current_position = self.get_position()
                distance, direction = navigation.calc_vector(current_position, next_node_position)
            current_position = self.get_position()
            final_distance, direction = navigation.calc_vector(current_position, final_destination)
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
            if edge.get_weight() == math.inf:
                continue
            coord_1, coord_2 = edge.get_coordinates()
            for obstacle in obstacles:
                dist = navigation.distance_point_line(obstacle.x, obstacle.y, coord_1[0], coord_1[1], coord_2[0],
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
            dist = np.linalg.norm(np.array((position.x, position.y)) - np.array(node.get_coordinates()))
            if dist < min_dist:
                min_dist = dist
                closest_node_id = node_id
        return closest_node_id, min_dist

    def add_detour_nodes(self, graph, current_node, final_node, angle, radius):
        new_nodes_added = False
        if radius > 100:
            radius = 100
        x, y = current_node.get_coordinates()
        for alpha in [-angle, angle]:
            print('calculating new node in {} degrees'.format(alpha))
            x_coord = x - radius * math.sin((alpha * math.pi) / 180)
            y_coord = y + radius * math.cos((alpha * math.pi) / 180)
            if valid_position(Position(x_coord, y_coord)):
                closest_node, dist = self.find_closest_node(graph, Position(x_coord, y_coord))
                if dist > self._collision_th:
                    new_nodes_added = True
                    detour_node = Node(x_coord, y_coord)
                    print('current position: {}'.format(current_node))
                    print('new node at: {}'.format(detour_node))
                    graph.add_node(detour_node)
                    graph.add_edge(detour_node, current_node)
                    graph.add_edge(detour_node, final_node)

        return new_nodes_added

    def avoid_collision(self):
        collision_dist = [sample if sample > 0 else math.inf for sample in self.get_sensing()]
        print('Sonar Sample:')
        print(self.get_sensing())
        while min(collision_dist) < self._collision_th:
            print('Collision Alert')
            self._robot.drive(-350, - 350)
            time.sleep(0.6)
            collision_dist = [sample if sample > 0 else math.inf for sample in self.get_sensing()]


class RobotMapped(Robot):
    def __init__(self):
        super().__init__()

    def navigate2(self, final_destination_node, road_map, obstacles):

        # Init graph with current location and final destination
        final_destination_id = final_destination_node.get_id()
        final_destination = Position(final_destination_node.get_coordinates()[0],
                                     final_destination_node.get_coordinates()[1])
        current_position = self.get_position()
        print("got position the first time")
        # closest_node_id, dist = self.find_closest_node(road_map, current_position)

        k = 25  # speed factor in turn with movement
        threshold = 14
        threshold_final = 35
        final_distance, direction = navigation.calc_vector(current_position, final_destination)

        while final_distance > threshold:
            current_position = self.get_position()
            closest_node_id, dist = self.find_closest_node(road_map, current_position)
            print("Found closest node: {}".format(closest_node_id))
            visited, path = road_map.shortest_path(closest_node_id, final_destination_id)
            print("Found shortest way")
            try:
                next_node_position, departure_node_position = self.get_mid_destination(path, final_destination_id,
                                                                                       closest_node_id)
            except:
                print("Error. Rebooting")
                self.navigate(final_destination)
                break

            distance, direction = navigation.calc_vector(current_position, next_node_position)
            print("Found next node: {} , in distance: {}".format(next_node_position,distance))
            while distance > threshold:
                current_position = self.get_position()
                distance, direction = navigation.calc_vector(current_position, next_node_position)
                if self.deal_with_obstacles(road_map, obstacles) is True:
                    break
                alpha = navigation.angle((current_position.dx, current_position.dy), direction)
                # print('---------------------------------------------')
                # print('Navigating to Vertex {}, Distance {}, Alpha {}'.format(next_node_position, distance, alpha))
                left_motor, right_motor = self.get_speed(k, alpha, distance)
                self._robot.drive(left_motor, right_motor)
                time.sleep(0.4)
                if self.hard_turn(alpha) is True:
                    time.sleep(0.25)
                current_position = self.get_position()
                distance, direction = navigation.calc_vector(current_position, next_node_position)
            print("Got out of loop, recalculation way")
            current_position = self.get_position()
            final_distance, direction = navigation.calc_vector(current_position, final_destination)
            # self._robot.drive(-300, -300)
            if final_distance < threshold_final:
                # threshold = threshold_final
                return True
        return final_distance < threshold

    def navigate(self, destination):

        # Create graph with already mapped nodes and edges
        road_map = Graph()
        final_destination_id, final_destination_node = road_map.create_nodes1(destination)
        road_map.create_edges1(final_destination_node)
        print("Finished creating graph,nodes and edges")
        obstacles = Obstacles()
        is_arrived = self.navigate2(final_destination_node, road_map, obstacles)
        self.navigate_end(destination)
        return road_map, obstacles

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
    def get_mid_destination(path, destination_id, closest_node_id):

        temp_id = destination_id
        while temp_id is not None:
            mid_destination_id = temp_id
            temp_id = path.get(mid_destination_id)
            if temp_id == closest_node_id:
                mid_destination = navigation.node_id_to_coordinates(mid_destination_id)
                temp_coord = navigation.node_id_to_coordinates(temp_id)
                pos = Position(mid_destination[0], mid_destination[1]), Position(temp_coord[0], temp_coord[1])
                return pos

    def deal_with_obstacles(self, road_map, obstacles):
        collision_th = 18

        sonar_sample = self.get_sensing()
        current_position = self.get_position()
        new_obstacles = obstacles.add_obstacles(current_position, sonar_sample)
        if new_obstacles is True:
            collision_detected = self.update_graph(road_map, obstacles.get_obstacles(), collision_th)
            if collision_detected is True:
                self._robot.drive(-325, -325)
                current_position = self.get_position()
                closest_node_id, dist = self.find_closest_node(road_map, current_position)
                print('Collision Detected closest node in {}'.format(dist))
                self.update_graph(road_map, obstacles.get_obstacles(), collision_th)
            return collision_detected

    @staticmethod
    def update_graph(graph, obstacles, collision_th):
        # updates the graph according to edges segment and obstacle points
        collision_detected = False
        for edge_id, edge in graph.get_edges():
            if edge.get_weight() == math.inf:
                continue
            coord_1, coord_2 = edge.get_coordinates()
            for obstacle in obstacles:
                dist = navigation.distance_point_line(obstacle.x, obstacle.y, coord_1[0], coord_1[1], coord_2[0],
                                                      coord_2[1])
                if dist <= collision_th:
                    edge.set_weight(math.inf)
                    collision_detected = True
                    break
        return collision_detected



    def navigate_end(self, destination):
        k = 25
        threshold = 10
        print("navigating to final position")
        current_position = self.get_position()
        distance, direction = navigation.calc_vector(current_position, destination)
        while distance > threshold:
            current_position = self.get_position()
            distance, direction = navigation.calc_vector(current_position, destination)
            if distance < threshold:
                break
            alpha = navigation.angle((current_position.dx, current_position.dy), direction)
            # print('-------------------------')
            # print('distance [{}], alpha [{}] '.format(distance, alpha))
            left_motor, right_motor = self.get_speed(k/2, alpha, distance)
            self._robot.drive(left_motor, right_motor)
            time.sleep(0.3)
            if self.hard_turn(alpha) is True:
                time.sleep(0.25)
            # print('Left Motor [{}], Right Motor [{}]'.format(left_motor, right_motor))
            distance, direction = navigation.calc_vector(current_position, destination)
        self._robot.drive(-350, -350)
        return distance < threshold