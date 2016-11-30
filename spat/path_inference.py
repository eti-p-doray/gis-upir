from scipy import linalg
import numpy as np
import math
import shapely.geometry as sg
import time

from kalman import KalmanFilter
from priority_queue import PriorityQueue
from utility import *

def project_feature_on_segment(transition, state, (a,b), offset, width, projection):
  a, b = np.array(a), np.array(b)
  v = b - a
  length = norm2d(v)
  if length == 0.0:
    return np.inf, length, None, None
  n = normal2d(v) / length
  d = v / length
  w = np.dot(n, a) + offset
  z = np.dot(d, a)
  quantile = 2.33

  H = np.asmatrix([[n[0], n[1], 0.0, 0.0],
                   [0.0, 0.0, n[0], n[1]]])
  l = state.measurment_update([w, 0.0], H, np.diag([(width / (quantile*2))**2, 0.2]))
  
  D = np.asmatrix([[d[0], d[1], 0.0, 0.0],
                   [0.0, 0.0, d[0], d[1]]])
  if projection != None:
    l += state.measurment_distance([projection.x[0]+z], D[0,:], projection.P[0,0])
  l += state.ineq_constraint_update(D, [z, 0.0], [z+length, 50.0])

  F, Q = transition
  next_projection = state.transform(D)
  next_projection.x[0] -= z
  next_projection.time_update(D * F * D.T, D * Q * D.T)

  return l, length, state, next_projection


def project_feature_on_way(transition, state, segments, projection, i):
  best_cost = np.inf
  total_length = 0
  best_state = None
  best_projection = None
  if projection != None:
    projection = projection.copy()

  segments = iter(segments)
  for j, (segment, offset, width) in zip(xrange(i+1), segments):
    length = sg.LineString(segment).length
    if projection != None:
      projection.x[0] -= length
    total_length += length

  for k, (segment, offset, width) in enumerate(segments):
    cost, length, new_state, new_projection = project_feature_on_segment(transition, state.copy(), segment, offset, width, projection)
    if projection != None:
      projection.x[0] -= length
    if cost < best_cost:
      new_projection.x[0] += total_length
      best_cost = cost
      best_state = new_state
      best_projection = new_projection
      i = j + k

    total_length += length

  return best_cost, best_state, best_projection, i

def find_feature_projections(graph, state):
  quantile = 20.0
  ell = quantile * linalg.sqrtm(state.P[0:2,0:2])
  height = math.sqrt(ell[0][0]**2 + ell[1][0]**2)
  width = math.sqrt(ell[0][1]**2 + ell[1][1]**2)
  return graph.edge_intersection(bb_bounds(state.x[0], state.x[1], width, height))

def infer_path(states, transition, graph, heuristic_factor):

  def project_features(previous_edge, current_edge, projection, i, j = -1):
    current_way = list(graph.way(current_edge))
    end_point = np.array(current_way[-1])
    previous_length = 0
    if previous_edge != None:
      previous_way = list(graph.way(previous_edge))
      previous_length = sg.LineString(previous_way).length
    current_length = sg.LineString(current_way).length
    total_cost = 0.0
    heuristic = 0.0

    def segments():
      if previous_edge != None:
        for segment in pairwise(graph.way(previous_edge)):
          yield segment, 0.0, 1.0 # should do some logic
      for segment in pairwise(graph.way(current_edge)):
        yield segment, 0.0, 1.0

    if projection != None:
      projection = projection.copy()

    k = 0
    while i < len(states):
      cost, state, new_projection, k = project_feature_on_way(transition, states[i], segments(), projection, k)
      distance_to_next = 0.0
      distance_nexts = 0.0
      if state == None:
        return np.inf, np.inf, np.inf, None, 0
      if i + 1 < len(states):
        distance_to_current = distance2d(state.x[0:2], states[i].x[0:2])
        distance_to_next = distance2d(state.x[0:2], states[i+1].x[0:2])
        distance_nexts = distance2d(states[i].x[0:2], states[i+1].x[0:2])

      finish = np.inf
      if projection != None:
        finish = projection.left_ineq_constraint_distance([1.0, 0.0], previous_length)
      if (cost > finish
          and cost/heuristic_factor + distance_to_next >= 
           distance_nexts + distance2d(end_point, states[i].x[0:2])
          and i > j):
        break


      heuristic -= distance_nexts
      total_cost += cost
      projection = new_projection

      i += 1

    delta = 0.0
    if i < len(states):
      delta += heuristic_factor * (distance2d(end_point, states[i].x) + current_length)
      if projection != None:
        projection.x[0] -= previous_length
        delta += projection.left_ineq_constraint_distance([1.0, 0.0], 0.0)
    return total_cost, heuristic_factor * heuristic, delta, projection, i


  queue = PriorityQueue()
  metrics = {}
  previous = {}
  visited = {}

  def enqueue(previous_edge, new_edge, cost, heuristic, delta, projection, i, j):
    metrics[new_edge, j] = (cost, heuristic, projection)
    previous[new_edge, j] = (previous_edge, i)
    queue.put((new_edge, j), cost + heuristic + delta)

  def try_enqueue(previous_edge, new_edge, cost, heuristic, delta, projection, i, j):
    #print previous_edge, new_edge, cost, heuristic, delta, i, j
    if (((new_edge, j) not in metrics or 
        cost < metrics[new_edge, j][0]) and
        np.isfinite(cost)):
      enqueue(previous_edge, new_edge, cost, heuristic, delta, projection, i, j)

  def try_visit(previous_edge, new_edge, i, j = -1):
    if new_edge in visited and i <= visited[new_edge]:
      return
    cost, heuristic, projection = metrics[previous_edge, i]
    edge_cost, edge_heuristic, delta, projection, j = project_features(previous_edge, new_edge, projection, i, j)
    try_enqueue(previous_edge, new_edge, cost + edge_cost, heuristic + edge_heuristic, delta, projection, i, j)

  def try_revisit(edge, j):
    return try_visit(edge, edge, j, j)

  start_time = time.time()
  visited_count = 0
  discovered_count = 0

  for i, s in enumerate(states):
    projections = peek(find_feature_projections(graph, s))
    if projections != None:
      break
  if projections == None:
    return []
  metrics[None, i] = (0.0, 0.0, None)
  for p in projections:
    (a, b) = p.object
    try_visit(None, (a,b), i, i)
    try_visit(None, (b,a), i, i)
  if queue.empty():
    return []

  while not queue.empty():
    (edge, i) = queue.get()
    if edge in visited and i <= visited[edge]:
      continue
    else:
      visited[edge] = i
      visited_count += 1
    if i == len(states):
      break

    for next in graph.graph.neighbors(edge[1]):
      try_visit(edge, (edge[1], next), i)
      discovered_count += 1

  elapsed_time = time.time() - start_time
  print 'elapsed_time: ', elapsed_time
  print 'visited_count: ', visited_count
  print 'discovered_count: ', discovered_count

  def backward_path():
    r = (edge, i)
    while r[0] != None:
      yield r[0]
      r = previous[r]

  selected_count = sum(1 for x in backward_path())

  print 'selected_count: ', selected_count

  return reversed(list(backward_path()))
