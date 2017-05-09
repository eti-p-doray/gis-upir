import logging
import numpy
from scipy import spatial
import shapely.geometry as sg

from spat.trajectory import features, model
from spat import facility, markov, utility


def inverse_optimal_control(data, batch_size,
                            eval_gradient, weights,
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
        for j in range(0, len(data), batch_size):
            batch = data[j:j+batch_size]
            gradient = eval_gradient(weights, batch)
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
        for x in graph.search_edges(utility.bb_buffer(self.coord, 20.0)):
            u, v = x.object
            link = sg.LineString(graph.edge_geometry(u, v))
            projection = link.project(self.coord)
            self.edges[u, v] = (projection, link.length)
            self.edges[v, u] = (link.length - projection, link.length)

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

    def cost(self, link_cost_fcn):
        return 0.0

    def cost_to(self, other, graph: facility.SpatialGraph, link_cost_fcn, intersection_cost_fcn):
        assert isinstance(other, Node)
        u, v = other.edge
        return sg.LineString(graph.edge_geometry(u, v)).distance(self.coordinates()) * link_cost_fcn(None)

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
            u, v = next_edge
            if goal.on(next_edge):
                yield Node(next_edge, 0.0, goal.edges[next_edge][0])
            else:
                yield Node(next_edge, 0.0, sg.LineString(graph.edge_geometry(u, v)).length)

    def length(self):
        return self.end - self.begin

    def cost(self, link_cost_fcn):
        return self.length() * link_cost_fcn(self.edge)

    def cost_to(self, other, graph: facility.SpatialGraph, link_cost_fcn, intersection_cost_fcn):
        if isinstance(other, BoundNode):
            return other.cost_to(self, graph, link_cost_fcn, intersection_cost_fcn)
        return intersection_cost_fcn(self.edge, other.edge)

    def heuristic(self, graph: facility.SpatialGraph, goal: BoundNode, greedy_factor: float):
        return graph.node_geometry(self.edge[1]).distance(goal.coordinates()) * greedy_factor


def best_path(weights, trajectory, graph: facility.SpatialGraph, intersection_collections):
    link_weights = weights[0:11]
    intersection_weights = weights[11:18]

    def link_cost(link):
        if link is None:
            return 300.0
        return numpy.dot(features.link_features(link, graph), link_weights)

    def intersection_cost(a, b):
        return numpy.dot(features.intersection_features(a, b, graph, intersection_collections), intersection_weights)

    goal = BoundNode(trajectory['segment'][-1].geometry[-1], graph, final=True)

    def adjacent_nodes(state):
        return state.adjacent_nodes(graph, goal)

    def state_cost(state):
        return state.cost(link_cost)

    def transition_cost(current_state, next_state):
        return current_state.cost_to(next_state, graph, link_cost, intersection_cost)

    def heuristic(state):
        return state.heuristic(graph, goal, 2.0)

    chain = markov.MarkovGraph(adjacent_nodes, state_cost, transition_cost)
    path = list(chain.find_best(BoundNode(trajectory['segment'][0].geometry[0], graph), goal, heuristic))

    feature = numpy.ones(weights.shape)
    for node in path:
        if isinstance(node, Node):
            feature[0:11] += features.link_features(node.edge, graph) * node.length()
    for a, b in utility.pairwise(path):
        if isinstance(a, Node) and isinstance(b, Node):
            feature[11:18] += features.intersection_features(a.edge, b.edge, graph, intersection_collections)

    return path, feature


def feature_expectation(weights, trajectory, graph: facility.SpatialGraph, intersection_collections):
    path, feature = best_path(weights, trajectory, graph, intersection_collections)
    return feature


def estimate_gradient(param, examples, graph: facility.SpatialGraph, intersection_collections):
    gradient = numpy.zeros(param.shape)
    for example in examples:
        feature = feature_expectation(param, example[1], graph, intersection_collections)
        #print 'features', feature
        #print 'exemple', example[0]
        gradient += numpy.divide(example[0] - feature, example[0] + 1.0)
    return gradient
