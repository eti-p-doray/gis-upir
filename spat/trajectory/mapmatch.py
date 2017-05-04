import time
import math
import numpy
from scipy import linalg, spatial

from spat import markov, utility, facility, kalman
from spat.trajectory import features


class Segment:
    def __init__(self, edge, coordinates, offset : float, width : float, transition):
        self.origin = numpy.asarray(coordinates[0])
        self.destination = numpy.asarray(coordinates[1])
        self.width = (width / (2.33*2.0))**2.0
        self.edge = edge

        v = self.destination - self.origin
        self.length = spatial.distance.euclidean(self.origin, self.destination)

        self.direction = v / self.length
        self.normal = utility.normal2d(v) / self.length

        def projection_along(v):
            return numpy.asmatrix([
                numpy.hstack((v, numpy.zeros(2))),
                numpy.hstack((numpy.zeros(2), v))])

        self.H = projection_along(self.normal)
        self.D = projection_along(self.direction)

        F, Q = transition
        self.F = self.D * F * self.D.T
        self.Q = self.D * Q * self.D.T
        self.normal_distance = numpy.dot(self.normal, self.origin) + offset
        self.direction_distance = numpy.dot(self.direction, self.origin)

    def project(self, state: kalman.KalmanFilter):
        if self.empty():
            return numpy.inf, None
        try:

            cost = state.measurment_update(
                [self.normal_distance, 0.0],
                self.H,
                numpy.diag([self.width, 1.0]))

            cost += state.ineq_constraint_update(
                self.D,
                [self.direction_distance, 0.0],
                [self.direction_distance + self.length, 50.0])

        except ValueError:
            return numpy.inf, None, None

        projected_state = state.transform(self.D)
        projected_state.x[0] -= self.direction_distance

        return cost, state, projected_state

    def advance(self, projected_state):
        projected_state.time_update(self.F, self.Q)
        return projected_state

    def distance_cost(self, projected_state, constrained_state, travelled_distance):
        return constrained_state.measurment_distance(
            [projected_state.x[0] - travelled_distance + self.direction_distance],
            self.D[0, :],
            [projected_state.P[0, 0] + 2.0]
        )

    def empty(self):
        return self.length == 0.0


class Link:
    def __init__(self, graph: facility.SpatialGraph, transition, edge):
        self.segments = []
        self.length = 0.0
        for coord in utility.pairwise(graph.edge_geometry(*edge)):
            segment = Segment(edge, coord, 0.0, 1.0, transition)
            self.segments.append(segment)
            self.length += segment.length

    def count(self) -> int:
        return len(self.segments)

    def at(self, idx) -> Segment:
        return self.segments[idx]


class LinkManager:
    def __init__(self, graph: facility.SpatialGraph, transition):
        self.graph = graph
        self.transition = transition
        self.link_table = {}

    def at(self, edge) -> Link:
        if edge not in self.link_table:
            self.link_table[edge] = Link(self.graph, self.transition, edge)
        return self.link_table[edge]


def find_edges(state, graph: facility.SpatialGraph, quantile):
    ell = quantile * linalg.sqrtm(state.P[0:2, 0:2])
    height = math.sqrt(ell[0][0] ** 2 + ell[1][0] ** 2)
    width = math.sqrt(ell[0][1] ** 2 + ell[1][1] ** 2)

    for x in graph.search_edges(utility.bb_bounds(state.x[0], state.x[1], width, height)):
        yield x.object


class LinkedNode:
    def __init__(self, edge, offset: int, time: int):
        self.edge = edge
        self.offset = offset
        self.time = time

    def __eq__(self, other):
        return (isinstance(other, self.__class__) and
                self.edge == other.edge and
                self.offset == other.offset and
                self.time == other.time)

    def __hash__(self):
        return hash((self.edge, self.offset, self.time))

    def __str__(self):
        return "LinkedNode: " + str((self.edge, self.offset, self.time))

    def progress(self):
        return self.edge, self.time

    def project(self, states, geometry: LinkManager):
        state = states[self.time].copy()
        link = geometry.at(self.edge)
        segment = link.at(self.offset)
        cost, constrained_state, projected_state = segment.project(state)

        if self.time + 1 < len(states):
            exhausted = (cost + spatial.distance.euclidean(constrained_state.x[0:2], states[self.time+1].x[0:2]) >
                         spatial.distance.euclidean(segment.destination, states[self.time].x[0:2]) +
                         spatial.distance.euclidean(states[self.time].x[0:2], states[self.time + 1].x[0:2]) +
                         projected_state.ineql_constraint_distance([1.0, 0.0], segment.length))
        else:
            exhausted = True

        return LinkedNode.State(self, exhausted, constrained_state, projected_state), cost

    class State:
        def __init__(self, node, exhausted,
                     constrained_state: kalman.KalmanFilter,
                     projected_state: kalman.KalmanFilter):
            self.edge = node.edge
            self.offset = node.offset
            self.time = node.time
            self.constrained_state = constrained_state
            self.projected_state = projected_state
            self.exhausted = exhausted

        def coordinates(self):
            return self.constrained_state.x[0:2]

        def key(self):
            return LinkedNode(self.edge, self.offset, self.time)

        def adjacent_nodes(self, states, graph: facility.SpatialGraph, geometry: LinkManager):
            if self.time + 1 == len(states):
                yield FinalNode()
                return
            #yield LinkedNode(self.edge, self.offset, self.time + 1)
            if not self.exhausted:
                yield LinkedNode(self.edge, self.offset, self.time + 1)
                return
            link = geometry.at(self.edge)
            segment = link.at(self.offset)
            next_projected_state = segment.advance(self.projected_state.copy())
            if self.offset + 1 == link.count():
                for next_link in graph.adjacent(self.edge[1]):
                    yield ForwardingNode(self, segment.length, next_projected_state, next_link, 0)
                yield JumpingNode(self)
            else:
                yield ForwardingNode(self, segment.length, next_projected_state, self.edge, self.offset + 1)

        def distance_to(self, other):
            if isinstance(other, LinkedNode.State):
                return other.projected_state.x[0] - self.projected_state.x[0] # difference on same segment
            elif isinstance(other, ForwardingNode.State):
                return other.segment.length - self.projected_state.x[0] # remaining length of segment
            return 0.0 # For JumpingNode, cost is added lazily through heuristic instead

        def cost_to(self, other, distance_cost_fcn, intersection_cost_fcn):
            if isinstance(other, FinalNode):
                return 0.0
            #cost = distance_cost_fcn(self.distance_to(other), self.edge)
            #if other.edge() != edge():
            #    cost += 0.0
            return distance_cost_fcn(self.distance_to(other), self.edge)

        def heuristic(self, states, distance_cost_fcn, cumulative_distance, greedy_factor):
            if self.time + 1 < len(states):
                return greedy_factor * (spatial.distance.euclidean(self.coordinates(), states[self.time+1].x[0:2]) -
                                        cumulative_distance[self.time+1])
            else:
                return -greedy_factor * cumulative_distance[-1]


class ForwardingNode:
    def __init__(self, anchor: LinkedNode.State, distance: float, projected_state, edge, offset):
        self.anchor = anchor
        self.distance = distance
        self.projected_state = projected_state
        self.edge = edge
        self.offset = offset

    def __eq__(self, other):
        return (isinstance(other, self.__class__) and
                self.anchor.key() == other.anchor.key() and
                self.edge == other.edge and
                self.offset == other.offset)

    def __hash__(self):
        return hash((self.anchor.key(), self.edge, self.offset))

    def __str__(self):
        return "ForwardingNode: " + str((self.anchor.edge, self.anchor.offset, self.anchor.time, self.distance, self.edge, self.offset))

    def progress(self):
        return self.edge, self.anchor.time + 1

    def project(self, states, geometry: LinkManager):
        return ForwardingNode.State(self, geometry.at(self.edge).at(self.offset)), 0.0

    class State:
        def __init__(self, node, segment: Segment):
            self.anchor = node.anchor
            self.edge = node.edge
            self.distance = node.distance
            self.offset = node.offset
            self.projected_state = node.projected_state
            self.segment = segment

        def coordinates(self):
            return self.segment.destination

        def adjacent_nodes(self, states, graph: facility.SpatialGraph, geometry: LinkManager):
            yield LinkedNode(self.edge, self.offset, self.anchor.time + 1)

            distance = self.distance + self.segment.length
            link = geometry.at(self.edge)
            if self.offset + 1 == link.count():
                for edge in graph.adjacent(self.edge[1]):
                    yield ForwardingNode(self.anchor, distance, self.projected_state, edge, 0)
            else:
                yield ForwardingNode(self.anchor, distance, self.projected_state, self.edge, self.offset + 1)

        def distance_to(self, other):
            if isinstance(other, LinkedNode.State):
                return other.projected_state.x[0] # other is always on the same segment as self
            elif isinstance(other, ForwardingNode.State):
                return self.segment.length # other is always on a segment adjacent to self

        def cost_to(self, other, distance_cost_fcn, intersection_cost_fcn):
            cost = distance_cost_fcn(self.distance_to(other), self.edge)
            if isinstance(other, LinkedNode.State):
                cost += self.segment.distance_cost(self.projected_state, other.constrained_state, self.distance)
            return cost

        def heuristic(self, states, distance_cost_fcn, cumulative_distance, greedy_factor):
            return (self.projected_state.ineql_constraint_distance([1.0, 0.0], self.distance) +
                    greedy_factor * (spatial.distance.euclidean(self.coordinates(), states[self.anchor.time+1].x[0:2]) -
                                     cumulative_distance[self.anchor.time+1]))


class FloatingNode:
    def __init__(self, time):
        self.time = time

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.time == other.time

    def __hash__(self):
        return hash(self.time)

    def __str__(self):
        return "FloatingNode: " + str(self.time)

    def project(self, states, geometry: LinkManager):
        return FloatingNode.State(self, states[self.time]), 0.0

    def progress(self):
        return None, self.time

    class State:
        def __init__(self, node, state):
            self.time = node.time
            self.state = state

        def coordinates(self):
            return self.state.x[0:2]

        def adjacent_nodes(self, states, graph: facility.SpatialGraph, geometry: LinkManager):
            if self.time + 1 == len(states):
                yield FinalNode()
                return
            yield FloatingNode(self.time + 1)
            for edge in find_edges(states[0], graph, 20.0):
                link = geometry.at(edge)
                for offset in range(0, link.count()):
                    yield LinkedNode(edge, offset, self.time + 1)

        def distance_to(self, other) -> float:
            # Works for FloatingNode and LinkedNode
            return spatial.distance.euclidean(self.coordinates(), other.coordinates())

        def cost_to(self, other, distance_cost_fcn, intersection_cost_fcn):
            if isinstance(other, FinalNode):
                return 0.0
            return distance_cost_fcn(self.distance_to(other), None)

        def heuristic(self, states, distance_cost_fcn, cumulative_distance, greedy_factor):
            return -greedy_factor * cumulative_distance[self.time]


class JumpingNode:
    def __init__(self, anchor: LinkedNode.State):
        self.anchor = anchor

    def __eq__(self, other):
        return (isinstance(other, self.__class__) and
                self.anchor.key() == other.anchor.key())

    def __hash__(self):
        return hash(self.anchor.key())

    def __str__(self):
        return "JumpingNode: " + str((self.anchor.time))

    def progress(self):
        return None, self.anchor.time + 1

    def project(self, states, geometry: LinkManager):
        return JumpingNode.State(self, states[self.anchor.time+1]), 0.0

    class State:
        def __init__(self, node, state: kalman.KalmanFilter):
            self.anchor = node.anchor
            self.state = state

        def coordinates(self):
            return self.state.x[0:2]

        def adjacent_nodes(self, states, graph: facility.SpatialGraph, geometry: LinkManager):
            yield FloatingNode(self.anchor.time + 1)
            for edge in find_edges(states[0], graph, 10.0):
                link = geometry.at(edge)
                for offset in range(0, link.count()):
                    yield LinkedNode(edge, offset, self.anchor.time + 1)

        def distance_to(self, other) -> float:
            # Works for FloatingNode and LinkedNode
            return spatial.distance.euclidean(self.anchor.coordinates(), other.coordinates())

        def cost_to(self, other, distance_cost_fcn, intersection_cost_fcn):
            return distance_cost_fcn(self.distance_to(other), None)

        def heuristic(self, states, distance_cost_fcn, cumulative_distance, greedy_factor):
            return (distance_cost_fcn(self.distance_to(self), None) -
                    greedy_factor * cumulative_distance[self.anchor.time])


class InitialNode:

    def __eq__(self, other):
        return isinstance(other, self.__class__)

    def __hash__(self):
        return hash(0)

    def __str__(self):
        return "InitialNode"

    def progress(self):
        return None, 0

    def project(self, states, geometry: LinkManager):
        return self, 0.0

    @staticmethod
    def adjacent_nodes(states, graph: facility.SpatialGraph, geometry: LinkManager):
        for edge in find_edges(states[0], graph, 50.0):
            link = geometry.at(edge)
            for offset in range(0, link.count()):
                yield LinkedNode(edge, offset, 0)

    @staticmethod
    def cost_to(other, distance_cost_fcn, intersection_cost_fcn):
        return 0.0

    @staticmethod
    def heuristic(states, distance_cost_fcn, cumulative_distance, greedy_factor):
        return 0.0


class FinalNode:
    def __eq__(self, other):
        return isinstance(other, self.__class__)

    def __hash__(self):
        return hash(0)

    def __str__(self):
        return "FinalNode"

    def project(self, states, geometry: LinkManager):
        return self, 0.0

    @staticmethod
    def heuristic(states, distance_cost_fcn, cumulative_distance, greedy_factor):
        return -greedy_factor * cumulative_distance[-1]


def solve_one(trajectory, graph, distance_cost_fcn, intersection_cost_fcn, greedy_factor):
    print(trajectory['id'])

    states = trajectory['state']
    link_manager = LinkManager(graph, trajectory['transition'])

    cumulative_distance = [0.0]
    total_distance = 0.0
    for s1, s2 in utility.pairwise(states):
        distance = spatial.distance.euclidean(s1.x[0:2], s2.x[0:2])
        total_distance += distance
        cumulative_distance.append(total_distance)

    def adjacent_nodes(key):
        return key.adjacent_nodes(states, graph, link_manager)

    def state_projection(key):
        return key.project(states, link_manager)

    def transition_cost(current_state, next_state):
        return current_state.cost_to(next_state, distance_cost_fcn, intersection_cost_fcn)

    def progress(key):
        return key.progress()

    def heuristic(state):
        return state.heuristic(states, distance_cost_fcn, cumulative_distance, greedy_factor)

    chain = markov.MarkovGraph(adjacent_nodes, state_projection, transition_cost)

    start_time = time.time()
    path = chain.find_best(InitialNode(), FinalNode(), progress, heuristic)
    print('elapsed_time ', time.time() - start_time)
    print(list(path))
    return list(path)


def solve(trajectories, graph, greedy_factor):
    distance_weights = numpy.array([
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    intersection_weights = numpy.array([
        1.0, 0.0])

    def distance_cost(distance, link):
        if link is None:
            return 300.0 * distance
        predicates = [
            lambda link: True,
            features.link_type_predicate(graph, features.any_cycling_link),
            features.link_type_predicate(graph, features.designated_roadway),
            features.link_type_predicate(graph, features.bike_lane),
            features.link_type_predicate(graph, features.seperate_cycling_link),
            features.link_type_predicate(graph, features.offroad_link),
            features.link_type_predicate(graph, features.other_road_type),
        ]
        return distance * numpy.dot(
            numpy.array(list(map(lambda pred: pred(link), predicates))),
            distance_weights)

    end_of_facility = features.match_intersections(
        features.load_discontinuity("data/discontinuity/end_of_facility"), graph)

    def intersection_cost(u, v, k):
        node_predicates = [
            lambda link: True,
            features.intersection_collection(end_of_facility),
        ]
        return numpy.dot(
            numpy.array(map(lambda pred: pred(v), node_predicates)),
            intersection_weights)

    for trajectory in trajectories:
        yield solve_one(trajectory, graph, distance_cost, intersection_cost, greedy_factor)