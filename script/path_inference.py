from scipy import linalg
import numpy as np
import math
import shapely.geometry as sg

from kalman import KalmanFilter
from priority_queue import PriorityQueue
from utility import *

def project_feature_on_segment((x, P), (a,b)):
  a, b = np.array(a), np.array(b)
  v = b - a
  n = normal2d(v) / norm2d(v)
  w = np.dot(n, a)

  H = np.matrix([
    [n[0], n[1], 0.0, 0.0],
    [0.0, 0.0, n[0], n[1]]])
  y = np.array([w, 0.0])
  kf = KalmanFilter(x, P)
  (x, P, l) = kf.project_constraint(H, y)
  d = v / sqnorm2d(v)
  t = np.dot(kf.x[0:2]-a, d)
  if t > 1.0:
    (x, P, l2) = kf.project_constraint(np.hstack((d, [0.0, 0.0])), 1.0+np.dot(a, d))
    l += l2
  elif t < 0.0:
    (x, P, l2) = kf.project_constraint(np.hstack((d, [0.0, 0.0])), 0.0+np.dot(a, d))
    l += l2
  if np.dot(kf.x[2:4], v) < 0:
    (x, P, l2) = kf.project_constraint(np.hstack(([0.0, 0.0], d)), 0.0)
    l += l2
  return x, l


def project_feature_on_way(state, way):
  best_cost = Ellipsis
  for a,b in pairwise(way):
    x, cost = project_feature_on_segment(state, (a,b))
    if cost < best_cost:
      best_cost = cost
      best_state = x
  return best_state, best_cost

def find_feature_projections(graph, (x, P)):
  quantile = 50.0
  ell = quantile * linalg.sqrtm(P[0:2,0:2])
  height = math.sqrt(ell[0][0]**2 + ell[1][0]**2)
  width = math.sqrt(ell[0][1]**2 + ell[1][1]**2)
  return graph.edge_intersection(bb_bounds(x[0], x[1], width, height))

def project_features(states, way, i, j = -1):
  way = list(way)
  scale_factor = 100
  end_point = np.array(way[-1])
  total_cost = 0
  heuristic = 0
  if j == -1:
    total_cost = 10 * sg.LineString(way).length

  while i < len(states):
    x, cost = project_feature_on_way(states[i], way)
    distance_to_next = 0
    distance_nexts = 0
    if i + 1 < len(states):
      distance_to_next = distance2d(x, states[i+1][0])
      distance_nexts = distance2d(states[i][0], states[i+1][0])
    if cost + scale_factor * distance_to_next >= scale_factor * (distance_nexts + distance2d(end_point, states[i][0])) and i > j:
      break
    heuristic -= distance_nexts
    total_cost += cost
    i += 1

  delta = 0
  if i < len(states):
    delta = distance2d(end_point, states[i][0])
  return total_cost, scale_factor * heuristic, scale_factor * delta, i

def infer_path(states, graph):
  for i, s in enumerate(states):
    projections = peek(find_feature_projections(graph, s))
    if projections != None:
      break

  queue = PriorityQueue()
  metrics = {}
  previous = {}
  visited = {}

  def enqueue(previous_edge, new_edge, cost, heuristic, delta, i, j):
    metrics[new_edge, j] = (cost, heuristic)
    if previous_edge == new_edge:
      previous[new_edge, j] = previous[new_edge, i]
    else:
      previous[new_edge, j] = (previous_edge, i)
    queue.put((new_edge, i, j), cost + heuristic + delta)

  def try_enqueue(previous_edge, new_edge, cost, heuristic, delta, i, j):
    if ((new_edge, j) not in metrics or 
        cost < metrics[new_edge, j][0]):
      enqueue(previous_edge, new_edge, cost, heuristic, delta, i, j)

  def try_visit(previous_edge, new_edge, i, j = -1):
    if (new_edge, i) in visited:
      return
    edge_cost, edge_heuristic, delta, j = project_features(states, graph.way(new_edge), i, j)
    cost, heuristic = metrics[previous_edge, i]
    new_cost = cost + edge_cost
    new_heuristic = heuristic + edge_heuristic
    try_enqueue(previous_edge, new_edge, new_cost, new_heuristic, delta, i, j)

  def try_revisit(edge, j):
    return try_visit(edge, edge, j, j)

  for p in projections:
    (a, b) = p.object
    enqueue(None, (a,b), 10, 0, 0, 0, i)
    enqueue(None, (b,a), 10, 0, 0, 0, i)

  while not queue.empty():
    (edge, i, j) = queue.get()
    print edge, j
    if (edge, i) in visited:
      continue
    else:
      visited[edge, i] = True
    if j == len(states):
      break


    k = try_revisit(edge, j)
    for next in graph.graph.neighbors(edge[1]):
      k = try_visit(edge, (edge[1], next), j)

  def backward_path():
    r = (edge, j)
    while r[0] != None:
      yield r[0]
      r = previous[r]

  return reversed(list(backward_path()))
