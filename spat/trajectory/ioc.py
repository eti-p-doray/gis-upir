import logging
import numpy
from scipy import spatial
import shapely.geometry as sg

from spat.trajectory import features, model
from spat import facility, markov, utility


def inverse_optimal_control(data, eval_gradient, weights,
                            learning_rate, precision, nb_epochs):
    b1 = 0.9
    b2 = 0.999
    b1t = b1
    b2t = b2
    e = 1e-8

    m = 0
    v = 0

    for i in range(nb_epochs):
        numpy.random.shuffle(data)
        for j in range(0, len(data)):
            gradient = eval_gradient(weights, data[j])
            if gradient is None:
              continue
            logging.info("gradient: %s", str(gradient))

            m = b1 * m + (1.0 - b1) * gradient
            v = b2 * v + (1.0 - b2) * numpy.square(gradient)
            mc = m / (1.0 - b1t)
            vc = v / (1.0 - b2t)
            b1t *= b1
            b2t *= b2

            weights -= learning_rate * numpy.divide(mc, numpy.sqrt(vc) + e)
            logging.info("weights: %s", str(weights))

    return weights


class BoundNode:
    def __init__(self, coord, graph: facility.SpatialGraph, final: bool=False):
        self.final = final
        self.coord = sg.Point(coord)
        self.edges = {}
        for x in graph.search_edge_nearest(utility.bb_buffer(self.coord, 5.0), 10):
            edge = x.object
            u, v, k = edge
            link = graph.edge_geometry(edge)
            projection = link.project(self.coord)
            self.edges[u, v, k] = (projection, link.length)
            self.edges[v, u, k] = (link.length - projection, link.length)

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.final == other.final

    def __hash__(self):
        return hash(self.final)

    def adjacent_nodes(self, graph: facility.SpatialGraph, goal):
        for edge, (begin, end) in self.edges.items():
            yield Node(edge, begin, end)

    def coordinates(self):
        return self.coord

    def on(self, edge):
        return edge in self.edges

    def cost(self, graph: facility.SpatialGraph, link_cost_fcn):
        return 0.0

    def cost_to(self, other, graph: facility.SpatialGraph, link_cost_fcn, intersection_cost_fcn):
        assert isinstance(other, Node)
        geometry = graph.edge_geometry(other.edge)
        projection = geometry.project(self.coord)
        return link_cost_fcn(geometry.distance(self.coordinates()), self.coord, geometry.interpolate(projection), None)

    def heuristic(self, graph: facility.SpatialGraph, goal, greedy_factor: float):
        return spatial.distance.euclidean(self.coordinates(), goal.coordinates()) * greedy_factor


class Node:
    def __init__(self, edge, begin, end):
        self.edge = edge
        self.begin = begin
        self.end = end

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.edge == other.edge

    def __hash__(self):
        return hash(self.edge)

    def __lt__(self, other):
        return self.edge < other.edge

    def adjacent_nodes(self, graph: facility.SpatialGraph, goal: BoundNode):
        if goal.on(self.edge):
            yield goal
            return
        for next_edge in graph.adjacent(self.edge[1]):
            if goal.on(next_edge):
                yield Node(next_edge, 0.0, goal.edges[next_edge][0])
            else:
                yield Node(next_edge, 0.0, graph.edge_geometry(next_edge).length)

    def length(self):
        return self.end - self.begin

    def cost(self, graph: facility.SpatialGraph, link_cost_fcn):
        geometry = graph.edge_geometry(self.edge)
        return link_cost_fcn(self.length(), geometry.interpolate(self.begin), geometry.interpolate(self.end), self.edge)

    def cost_to(self, other, graph: facility.SpatialGraph, link_cost_fcn, intersection_cost_fcn):
        if isinstance(other, BoundNode):
            return other.cost_to(self, graph, link_cost_fcn, intersection_cost_fcn)
        return intersection_cost_fcn(self.edge, other.edge)

    def heuristic(self, graph: facility.SpatialGraph, goal: BoundNode, greedy_factor: float):
        return graph.node_geometry(self.edge[1]).distance(goal.coordinates()) * greedy_factor


def best_path(weights, trajectory, graph: facility.SpatialGraph, intersection_collections, elevation, dst_proj):
    link_weights = weights[0:14]
    intersection_weights = weights[14:21]

    def distance_cost(length, start, end, link):
        start_elevation = elevation.at((start.x, start.y), dst_proj)
        end_elevation = elevation.at((end.x, end.y), dst_proj)
        cost = numpy.dot(features.link_features(length, start_elevation, end_elevation, link, graph), link_weights)
        if link is None:
            cost += 100.0 * length
        return cost

    def intersection_cost(a, b):
        return numpy.dot(features.intersection_features(a, b, graph, intersection_collections), intersection_weights)

    goal = BoundNode(trajectory['segment'][-1].geometry[-1], graph, final=True)

    def adjacent_nodes(state):
        return state.adjacent_nodes(graph, goal)

    def state_cost(state):
        return state.cost(graph, distance_cost)

    def transition_cost(current_state, next_state):
        return current_state.cost_to(next_state, graph, distance_cost, intersection_cost)

    def heuristic(state):
        return state.heuristic(graph, goal, 2.0)

    chain = markov.MarkovGraph(adjacent_nodes, state_cost, transition_cost)
    path = chain.find_best(BoundNode(trajectory['segment'][0].geometry[0], graph), goal, heuristic)
    if path is None:
      return None, None
    path = list(path)

    feature = numpy.zeros(weights.shape)
    for node in path:
        if isinstance(node, Node):
            geometry = graph.edge_geometry(node.edge)
            start = geometry.interpolate(node.begin)
            end = geometry.interpolate(node.end)
            start_elevation = elevation.at((start.x, start.y), dst_proj)
            end_elevation = elevation.at((end.x, end.y), dst_proj)
            feature[0:14] += features.link_features(node.length(),
                                                    start_elevation, end_elevation,
                                                    node.edge, graph)
    for a, b in utility.pairwise(path):
        if isinstance(a, Node) and isinstance(b, Node):
            feature[14:21] += features.intersection_features(a.edge, b.edge, graph, intersection_collections)

    return path, feature


def feature_expectation(weights, trajectory, graph: facility.SpatialGraph, intersection_collections, elevation, dst_proj):
    path, feature = best_path(weights, trajectory, graph, intersection_collections, elevation, dst_proj)
    return feature


def estimate_gradient(param, example, graph: facility.SpatialGraph, intersection_collections, elevation, dst_proj):
    feature = feature_expectation(param, example[1], graph, intersection_collections, elevation, dst_proj)
    if feature is None:
        return None
    logging.info("example: %s", str(example[0]))
    logging.info("feature: %s", str(feature))
    #logging.info("%s, %s", str(numpy.dot(param, feature)), str(numpy.dot(param, example[0])))
    #if numpy.dot(param, feature) < numpy.dot(param, example[0]) / 2.0:
    #    return None # this example is too bad


    a = example[0] - feature
    b = param / numpy.linalg.norm(param)
    return a - numpy.dot(a, b) * b
