from scipy import linalg
from scipy import spatial
import numpy as np
import math
import shapely.geometry as sg
import time

from kalman import KalmanFilter
from priority_queue import PriorityQueue
from utility import *

class Segment:
  def __init__(self, coords, offset, width, transition, statemap):
    self.origin = np.asarray(coords[0])
    self.destination = np.asarray(coords[1])
    self.width = (width / (2.33*2))**2

    v = self.destination - self.origin
    self.length = spatial.distance.euclidean(self.origin, self.destination)
    self.direction = v / self.length
    self.normal = normal2d(v) / self.length

    self.H = statemap(self.normal)
    self.D = statemap(self.direction)

    F, Q = transition
    self.F = self.D * F * self.D.T
    self.Q = self.D * Q * self.D.T
    self.normal_distance = np.dot(self.normal, self.origin) + offset
    self.direction_distance = np.dot(self.direction, self.origin)

  def project(self, projection, state, threshold):
    if self.empty():
      return np.inf, None
    cost = state.measurment_update(
        [self.normal_distance, 0.0], 
        self.H, 
        np.diag([self.width, 1.0]))
    if cost >= threshold:
      return np.inf, None, None
    if projection != None:
      cost += state.measurment_update(
          [projection.x[0] + self.direction_distance], 
          self.D[0,:], 
          projection.P[0,0] + 6.0)
    if cost >= threshold:
      return np.inf, None, None
    cost += state.ineq_constraint_update(
        self.D, 
        [self.direction_distance, 0.0], 
        [self.direction_distance + self.length, 50.0])

    projection = state.transform(self.D)
    projection.x[0] -= self.direction_distance
    projection.time_update(self.F, self.Q)

    return cost, projection, state

  def empty(self):
    return self.length == 0.0

class Way:
  def __init__(self, graph, transition, statemap):
    self.segments = []
    self.length = 0.0
    self.transition = transition
    self.statemap = statemap
    self.graph = graph

  def copy(self):
    that = Way(self.graph, self.transition, self.statemap)
    that.segments = list(self.segments)
    that.length = self.length
    return that

  def append(self, edge):
    for coords in pairwise(self.graph.way(edge)):
      segment = Segment(coords, 0.0, 1.0, self.transition, self.statemap)
      self.segments.append(segment)
      self.length += segment.length

  def tip(self):
    return self.segments[-1].destination

  def saturation(self, projection):
    if projection != None:
      return projection.ineql_constraint_distance([1.0, 0.0], self.length)
    return np.inf

  def reset(self):
    self.length = 0.0
    self.segments = []

  def project(self, projection, state):
    best_cost = np.inf
    best_cstate = None
    best_pstate = None
    drop = 0
    self.length = 0.0
    if projection != None:
      projection = projection.copy()

    for i, segment in enumerate(self.segments):
      cost, pstate, cstate = segment.project(projection, state.copy(), best_cost)
      if projection != None:
        projection.x[0] -= segment.length

      if cost < best_cost:
        best_cost = cost
        best_pstate = pstate
        best_cstate = cstate

        drop = i
        self.length = segment.length
      else:
        self.length += segment.length

    if drop != 0:
      self.segments = self.segments[drop:]

    return best_cost, best_pstate, best_cstate

class Path:
  def __init__(self, graph, states, transition, statemap, coords, greedy, hop, index):
    self.way = Way(graph, transition, statemap)
    self.graph = graph
    self.states = states
    self.coords = coords
    self.greedy = greedy
    self.hop = hop

    self.pstate = None # current constrained state on way
    self.cstate = None # current state projected on way
    self.cost = 0.0
    self.heuristic = 0.0
    self.delta = 0.0
    self.exhausted = True

    self.edge, self.index = None, index
    self.previous = self.current()

  def copy(self):
    that = Path(self.graph, self.states, self.way.transition, 
                self.way.statemap, self.coords, self.greedy, self.hop,
                self.index)

    that.way = self.way.copy()
    if self.pstate != None:
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

  def advance(self, (cost, pstate, cstate)):
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

  def jump(self):
    self.cost += self.delta
    if self.index + 1 < len(self.states):
      distance = spatial.distance.euclidean(self.coords(self.state()), 
                                       self.coords(self.state(1)))
      self.heuristic -= self.greedy * distance
      self.delta = self.hop * distance
    else:
      self.delta = 0.0
    self.cstate = self.state()#self.coords(self.state())
    self.pstate = None
    self.index += 1
    return True

  def try_advance(self, (cost, pstate, cstate)):
    if self.index + 1 >= len(self.states):
      self.advance((cost, pstate, cstate))
      return True

    if (cost + self.distance(self.coords(cstate), self.coords(self.state(1))) <
        self.distance(self.way.tip(), self.coords(self.state())) + 
        self.distance(self.coords(self.state()), self.coords(self.state(1))) + 
        self.way.saturation(pstate)):
      self.advance((cost, pstate, cstate))
      return True
    else:
      self.delta = (self.distance(self.way.tip(), self.coords(self.state())) + 
                    self.way.saturation(self.pstate))
      return False

  def append(self, edge):
    self.previous = self.current()
    if edge == 0:
      self.edge = edge
      self.way.reset()
      self.delta = (self.hop * 
        spatial.distance.euclidean(self.coords(self.cstate), 
                                   self.coords(self.state())))
      self.exhausted = True

    elif self.edge == None or self.edge == 0:
      self.edge = edge
      self.pstate = None
      self.way.append(self.edge)

      cost, pstate, cstate = self.project()
      if self.cstate != None:
        self.cost += (self.hop * 
            spatial.distance.euclidean(self.coords(self.cstate), 
                                       self.coords(cstate)))
      self.advance((cost, pstate, cstate))
      self.exhausted = False

    else:
      self.edge = edge
      self.way.append(self.edge)
      self.exhausted = not self.try_advance(self.project())

    return self

  def branch(self, edge):
    return self.copy().append(edge)

  def next(self):
    self.previous = self.current()
    if self.edge == 0:
      self.exhausted = not self.jump()
    else:
      self.exhausted = not self.try_advance(self.project())
    return self

class PathInference:
  """
  The algorithm of path inference works with an A-star graph search.
  Every node of the graph is either :
    - an observation point or 
    - an intersection in the road network, when the current road is exhausted.
  """
  def __init__(self, graph, statemap, coords, nearby, greedy, hop):
    self.graph = graph
    self.coords = coords
    self.statemap = statemap
    self.greedy = greedy
    self.hop = hop
    self.nearby = nearby

  def make_path(self, states, transition, index):
    return Path(self.graph, states, transition, 
        self.statemap, self.coords, self.greedy, self.hop, index)

  def try_enqueue(self, path):

    if (np.isfinite(path.cost) and 
        (path.current() not in self.costs or 
         path.cost < self.costs[path.current()])):

      #ajout d'un flag exhausted
      self.states[path.current()] = path.cstate
        #'state': path.cstate, 
        #'edge': path.edge, # current edge being visited, 0 if not linked
        #'type': 'enqueued', 
        #'cost': path.cost, 
        #'priority': path.priority(), 
        #'index': path.index}
      self.costs[path.current()] = path.cost
      self.queue.put(path, path.priority())

  def try_append(self, path, edge):
    if edge in self.visited and path.index < self.visited[edge]:
      return

    self.try_enqueue(path.branch(edge))

  def visit(self, states, path):
    edge, i = path.current()
    if edge in self.visited and i < self.visited[edge]:
      return path
    else:
      self.visited[edge] = i

    self.previous[path.current()] = path.previous
    
    if i == len(states):
      return path

    if path.edge == 0:
      for edge in self.nearby(states[i]):
        (a, b) = edge.object
        self.try_append(path, (a, b))
        self.try_append(path, (b, a))
      self.try_enqueue(path.next())
    elif not path.exhausted:
      self.try_enqueue(path.next())
    else:
      for next in self.graph.neighbors(edge[1]):
        self.try_append(path, (edge[1], next))
      self.try_append(path, 0)
    return path

  def solve(self, states, transition):
    self.costs = {}
    self.previous = {}
    self.states = {}
    self.visited = {}
    self.queue = PriorityQueue()
    start_time = time.time()

    for i, state in enumerate(states):
      projections = peek(self.nearby(state))
      if projections != None:
        break
    if projections == None:
      return [], []
    self.costs[None, i] = (0.0, 0.0, None)
    for p in projections:
      (a, b) = p.object
      self.try_enqueue(self.make_path(states, transition, i).append((a,b)))
      self.try_enqueue(self.make_path(states, transition, i).append((b,a)))

    if self.queue.empty():
      return [], []

    while not self.queue.empty():
      path = self.visit(states, self.queue.get())
      if path.index == len(states):
        break

    print 'elapsed_time ', time.time() - start_time

    def backtrack():
      r = path.previous
      while r[0] != None:
        yield r
        r = self.previous[r]
    self.states = { key: self.states[key] for key in backtrack() }

    print 'cost ', path.cost, path.priority()

    return reversed(list(backtrack())), self.states


