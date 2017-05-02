from scipy import spatial
import numpy as np
import time

from spat.priority_queue import PriorityQueue
from spat import utility


class Segment:
    def __init__(self, edge, coords, offset, width, transition, statemap):
        self.origin = np.asarray(coords[0])
        self.destination = np.asarray(coords[1])
        self.width = (width / (2.33*2))**2
        self.edge = edge

        v = self.destination - self.origin
        self.length = spatial.distance.euclidean(self.origin, self.destination)

        self.direction = v / self.length
        self.normal = utility.normal2d(v) / self.length

        self.H = statemap(self.normal)
        self.D = statemap(self.direction)

        F, Q = transition
        self.F = self.D * F * self.D.T
        self.Q = self.D * Q * self.D.T
        self.normal_distance = np.dot(self.normal, self.origin) + offset
        self.direction_distance = np.dot(self.direction, self.origin)

    def project(self, projection, state, threshold):
        if self.empty():
            return np.inf, None, None
        try:
            if projection is not None:
                projection.time_update(self.F, self.Q)

            cost = state.measurment_update(
                [self.normal_distance, 0.0],
                self.H,
                np.diag([self.width, 1.0]))
            if cost >= threshold:
                return np.inf, None, None
            if projection is not None:
                c2 = state.measurment_update(
                    [projection.x[0] + self.direction_distance],
                    self.D[0,:],
                    [projection.P[0,0]+8.0])
                cost += c2
            if cost >= threshold:
                return np.inf, None, None
            c3 = state.ineq_constraint_update(
                self.D,
                [self.direction_distance, 0.0],
                [self.direction_distance + self.length, 50.0])
            cost += c3
        except ValueError:
            return np.inf, None, None

        projection = state.transform(self.D)
        projection.x[0] -= self.direction_distance

        return cost, projection, state

    def empty(self):
        return self.length == 0.0


class Way:
    def __init__(self, graph, transition, statemap, distance_cost_fn):
        self.segments = []
        self.length = 0.0
        self.transition = transition
        self.statemap = statemap
        self.graph = graph
        self.distance_cost_fn = distance_cost_fn

    def copy(self):
        that = Way(self.graph, self.transition, self.statemap, self.distance_cost_fn)
        that.segments = list(self.segments)
        that.length = self.length
        return that

    def append(self, edge):
        length = 0
        for coords in utility.pairwise(self.graph.edge_geometry(*edge)):
            segment = Segment(edge, coords, 0.0, 1.0, self.transition, self.statemap)
            self.segments.append(segment)
            length += segment.length
        self.length += length

    def tip(self):
        return self.segments[-1].destination

    def saturation(self, projection):
        if projection is not None:
            return projection.ineql_constraint_distance([1.0, 0.0], self.length)
        return np.inf

    def reset(self):
        self.length = 0.0
        self.segments = []

    def project(self, projection, state):
        best_cost = np.inf
        best_cstate = None
        best_pstate = None
        base_cost = 0
        drop = 0
        self.length = 0.0
        if projection is not None:
            projection = projection.copy()

        for i, segment in enumerate(self.segments):
            cost, pstate, cstate = segment.project(projection, state.copy(), best_cost - base_cost)
            cost += base_cost

            if projection is not None and pstate is not None:
                distance = pstate.x[0] - max(projection.x[0], 0)
                cost += self.distance_cost_fn(distance, segment.edge)

            if cost < best_cost:
                best_cost = cost
                best_pstate = pstate
                best_cstate = cstate

                drop = i
                self.length = segment.length
            else:
                self.length += segment.length

            if projection is not None:
                distance = min(segment.length - projection.x[0], segment.length)
                base_cost += self.distance_cost_fn(distance, segment.edge)
                projection.x[0] -= segment.length

        if drop != 0:
            self.segments = self.segments[drop:]

        return best_cost, best_pstate, best_cstate


class Path:
    def __init__(self, graph, states, transition, statemap, coords_fn,
                 distance_cost_fn, intersection_cost_fn, greedy, index):
        self.way = Way(graph, transition, statemap, distance_cost_fn)
        self.graph = graph
        self.states = states
        self.coords = coords_fn
        self.distance_cost_fn = distance_cost_fn
        self.intersection_cost_fn = intersection_cost_fn
        self.greedy = greedy

        self.pstate = None # current constrained state on way
        self.cstate = None # current state projected on way
        self.cost = 0.0
        self.heuristic = 0.0
        self.delta = 0.0
        self.exhausted = True

        self.edge, self.index = None, index
        self.previous = None

    def copy(self):
        that = Path(self.graph, self.states, self.way.transition,
                    self.way.statemap, self.coords,
                    self.distance_cost_fn, self.intersection_cost_fn,
                    self.greedy,
                    self.index)

        that.way = self.way.copy()
        if self.pstate is not None:
            that.pstate = self.pstate.copy()
        that.cstate = self.cstate.copy()
        that.cost = self.cost
        that.heuristic = self.heuristic
        that.delta = self.delta
        that.edge = self.edge
        that.previous = self.previous
        return that

    def priority(self):
        return self.cost + self.heuristic + self.delta

    def current(self):
        return self.edge, self.index + (1 if self.exhausted else 0)

    def state(self, i = 0):
        return self.states[self.index + i]

    def project(self):
        return self.way.project(self.pstate, self.state())

    def distance(self, a, b):
        return self.greedy * spatial.distance.euclidean(a, b)

    def advance(self, cost, pstate, cstate):
        self.cost += cost
        if self.index + 1 < len(self.states):

            self.heuristic -= self.distance(self.coords(self.state()),
                                            self.coords(self.state(1)))
            self.delta = self.distance(self.coords(cstate),
                                       self.coords(self.state(1)))
        else:
            self.delta = 0.0
        self.cstate = cstate
        self.pstate = pstate
        self.index += 1
        return self

    def jump(self):
        self.cost += self.delta
        if self.index + 1 < len(self.states):
            distance = spatial.distance.euclidean(self.coords(self.state()),
                                                  self.coords(self.state(1)))
            self.heuristic -= self.greedy * distance
            self.delta = self.distance_cost_fn(distance, None)
        else:
            self.delta = 0.0
        self.cstate = self.state()
        self.pstate = None
        self.index += 1
        return True

    def try_advance(self, cost, pstate, cstate):
        if cost == np.inf:
            self.cost += cost
            return True

        if self.index + 1 >= len(self.states) or self.exhausted:
            self.advance(cost, pstate, cstate)
            return True

        if (cost + self.distance(self.coords(cstate), self.coords(self.state(1))) <
                        self.distance(self.way.tip(), self.coords(self.state())) +
                        self.distance(self.coords(self.state()), self.coords(self.state(1))) +
                    self.way.saturation(pstate)):
            self.advance(cost, pstate, cstate)
            return True
        else:
            self.delta = (self.distance(self.way.tip(), self.coords(self.state())) +
                          self.way.saturation(pstate))
            return False

    def append(self, edge):
        assert (self.edge is not None or edge is not None)

        if self.edge is not None and edge is not None:
            (u, v) = self.edge
            (v, k) = edge
            self.cost += self.intersection_cost_fn(u, v, k)

        if self.edge is not None or self.previous is not None:
            self.previous = self.current()
        if edge is None:
            self.edge = edge
            self.way.reset()
            self.delta = self.distance_cost_fn(
                spatial.distance.euclidean(self.coords(self.cstate),
                                           self.coords(self.state())), None)
            self.exhausted = True

        elif self.edge is None:
            self.edge = edge
            self.pstate = None
            self.way.append(self.edge)

            cost, pstate, cstate = self.project()
            if self.cstate is not None:

                self.cost += self.distance_cost_fn(
                    spatial.distance.euclidean(self.coords(self.cstate),
                                               self.coords(cstate)), None)
            self.advance(cost, pstate, cstate)
            self.exhausted = False

        else:
            self.edge = edge
            self.way.append(self.edge)
            self.exhausted = not self.try_advance(*self.project())

        return self

    def branch(self, edge):
        return self.copy().append(edge)

    def next(self):
        self.previous = self.current()
        if self.edge is None:
            self.exhausted = not self.jump()
        else:
            self.exhausted = not self.try_advance(*self.project())
        return self


class PathInference:
    """
    The algorithm of path inference works with an A-star graph search.
    Every node of the graph is either :
      - an observation point or 
      - an intersection in the road network, when the current road is exhausted.
    """
    def __init__(self, graph, statemap, coords_fn,
                 distance_cost_fn, intersection_cost_fn, nearby_fn, greedy):
        self.graph = graph
        self.coords = coords_fn
        self.distance_cost_fn = distance_cost_fn
        self.intersection_cost_fn = intersection_cost_fn
        self.statemap = statemap
        self.nearby = nearby_fn
        self.greedy = greedy

    def make_path(self, states, transition, index):
        return Path(self.graph, states, transition,
                    self.statemap, self.coords,
                    self.distance_cost_fn, self.intersection_cost_fn,
                    self.greedy, index)

    def try_enqueue(self, path):
        if path.priority() > 300000:
            return
        if (np.isfinite(path.cost) and
                (path.current() not in self.costs or
                         path.cost < self.costs[path.current()])):

            #print ' ', path.current(), path.cost, path.priority()

            self.states[path.current()] = path.cstate

            self.costs[path.current()] = path.cost
            self.queue.put(path, path.priority())

    def try_append(self, path, edge):
        if path.priority() > 300000:
            return
        if edge in self.visited and path.index < self.visited[edge]:
            return
        self.try_enqueue(path.branch(edge))

    def visit(self, states, path):

        edge, i = path.current()
        if edge in self.visited and i < self.visited[edge]:
            return path
        else:
            self.visited[edge] = i

        #print path.current(), path.cost, path.priority()

        self.previous[path.current()] = path.previous

        if i == len(states):
            return path

        if path.edge is None:
            projections = self.nearby(states[i], 10.0)
            for edge in projections:
                (a, b) = edge.object
                self.try_append(path, (a, b))
                self.try_append(path, (b, a))
            self.try_enqueue(path.next())
        elif not path.exhausted:
            self.try_enqueue(path.next())
        else:
            for next in self.graph.neighbors(edge[1]):
                self.try_append(path, next)
            self.try_append(path, None)
            self.try_enqueue(path.next())
        return path

    def solve(self, states, transition):
        self.costs = {}
        self.previous = {}
        self.states = {}
        self.visited = {}
        self.queue = PriorityQueue()
        start_time = time.time()

        projections = utility.peek(self.nearby(states[0], 50.0))
        if projections == None:
            return None, self.states
        self.costs[None, 0] = (0.0, 0.0, None)
        for p in projections:
            (a, b) = p.object
            self.try_enqueue(self.make_path(states, transition, 0).append((a,b)))
            self.try_enqueue(self.make_path(states, transition, 0).append((b,a)))

        if self.queue.empty():
            return None, self.states

        while not self.queue.empty():
            path = self.visit(states, self.queue.get())
            if path == None:
                return None, self.states
            if path.index == len(states) and path.edge != None:
                break

        if path.index < len(states):
            return None, self.states

        print('elapsed_time ', time.time() - start_time)

        def backtrack():
            r = path.previous
            while r is not None:
                yield r
                r = self.previous[r]

        self.states = { key: self.states[key] for key in backtrack() }

        print('cost ', path.cost, path.priority())

        return reversed(list(backtrack())), self.states


