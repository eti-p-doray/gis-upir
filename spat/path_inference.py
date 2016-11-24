from scipy import linalg
import numpy as np
import math
import shapely.geometry as sg

from kalman import KalmanFilter
from priority_queue import PriorityQueue
from utility import *

def project_feature_on_segment((x, P), (a,b), proj):
  a, b = np.array(a), np.array(b)
  v = b - a
  length = norm2d(v)
  if length == 0.0:
    return Ellipsis, length, None, None
  n = normal2d(v) / length
  d = v / norm2d(v)
  w = np.dot(n, a)

  kf = KalmanFilter(x, P)
  H = np.matrix([
    [n[0], n[1], 0.0, 0.0],
    [0.0, 0.0, n[0], n[1]]])
  y = np.array([w, 0.0])
  (x, P, l) = kf.project_constraint(y, H)

  t = np.dot(kf.x[0:2]-a, d)
  if t > length:
    (x, P, l2) = kf.project_constraint(length+np.dot(a, d), np.hstack((d, [0.0, 0.0])))
    l += l2
  elif t < 0.0:
    (x, P, l2) = kf.project_constraint(0.0+np.dot(a, d), np.hstack((d, [0.0, 0.0])))
    l += l2
  if np.dot(kf.x[2:4], d) < 0:
    (x, P, l2) = kf.project_constraint(0.0, np.hstack(([0.0, 0.0], d)))
    l += l2
  #if proj != None:
    #print proj.x[0:2],
    #print proj.x[0], proj.P[0,0],
    #H = np.matrix([
    #[d[0], d[1], 0.0, 0.0],
    #[0.0, 0.0, d[0], d[1]]])
    #x,P,l2 = kf.measurment_update([proj.x[0]+np.dot(a, d), proj.x[1]], H, proj.P)
    #print proj.x[0:2],
    #print l, l2,
    #l += l2
    #print l2

  new_proj = kf.transform(np.matrix([
    [d[0], d[1], 0.0, 0.0],
    [0.0, 0.0, d[0], d[1]]]))
  new_proj.x[0] -= np.dot(a, d)
  #print new_proj.x[0:2]
  new_proj.time_update(np.matrix([[1.0, 1.0], [0.0, 1.0]]), np.diag([0.2, 0.5]))
  
  #print new_proj.x[0], new_proj.P[0,0]
  return l, length, kf, new_proj


def project_feature_on_way(state, way, projection):
  best_cost = Ellipsis
  total_length = 0.0
  if projection != None:
    projection = projection.copy()
  for a,b in pairwise(way):
    cost, length, new_state, new_projection = project_feature_on_segment(state, (a,b), projection)
    if projection != None:
      projection.x[0] -= length
    if cost < best_cost:
      new_projection.x[0] += total_length
      best_cost = cost
      best_state = new_state
      best_projection = new_projection
    total_length += length
  #print best_projection.x[0]
  return best_cost, best_state, best_projection

def find_feature_projections(graph, (x, P)):
  quantile = 50.0
  ell = quantile * linalg.sqrtm(P[0:2,0:2])
  height = math.sqrt(ell[0][0]**2 + ell[1][0]**2)
  width = math.sqrt(ell[0][1]**2 + ell[1][1]**2)
  return graph.edge_intersection(bb_bounds(x[0], x[1], width, height))

def project_features(states, way, projection, i, j = -1):
  way = list(way)
  heuristic_factor = 200.0
  end_point = np.array(way[-1])
  total_cost = 0.0
  heuristic = 0.0
  length = sg.LineString(way).length
  if j == -1 or projection == None:
    total_cost = 10 * length

  if projection != None:
    projection = projection.copy()
    if j != -1:
      projection.x[0] += length
  while i < len(states):
    cost, state, new_projection = project_feature_on_way(states[i], way, projection)
    distance_to_next = 0.0
    distance_nexts = 0.0
    if i + 1 < len(states):
      distance_to_next = distance2d(state.x[0:2], states[i+1][0])
      distance_nexts = distance2d(states[i][0], states[i+1][0])
    if cost + heuristic_factor * distance_to_next >= heuristic_factor * (distance_nexts + distance2d(end_point, states[i][0])) and i > j:
      break
    heuristic -= distance_nexts
    total_cost += cost
    projection = new_projection
    i += 1

  projection.x[0] -= length
  delta = 0
  if i < len(states):
    delta = distance2d(end_point, states[i][0])
  return total_cost, heuristic_factor * heuristic, heuristic_factor * delta, projection, i

def infer_path(states, graph):
  for i, s in enumerate(states):
    projections = peek(find_feature_projections(graph, s))
    if projections != None:
      break

  queue = PriorityQueue()
  metrics = {}
  previous = {}
  visited = {}

  def enqueue(previous_edge, new_edge, cost, heuristic, delta, projection, i, j):
    metrics[new_edge, j] = (cost, heuristic, projection)
    if previous_edge == new_edge:
      previous[new_edge, j] = previous[new_edge, i]
    else:
      previous[new_edge, j] = (previous_edge, i)
    queue.put((new_edge, i, j), cost + heuristic + delta)

  def try_enqueue(previous_edge, new_edge, cost, heuristic, delta, projection, i, j):
    if ((new_edge, j) not in metrics or 
        cost < metrics[new_edge, j][0]):
      enqueue(previous_edge, new_edge, cost, heuristic, delta, projection, i, j)

  def try_visit(previous_edge, new_edge, i, j = -1):
    if new_edge in visited and i < visited[new_edge]:
      return
    cost, heuristic, projection = metrics[previous_edge, i]
    edge_cost, edge_heuristic, delta, projection, j = project_features(states, graph.way(new_edge), projection, i, j)
    new_cost = cost + edge_cost
    new_heuristic = heuristic + edge_heuristic
    try_enqueue(previous_edge, new_edge, new_cost, new_heuristic, delta, projection, i, j)

  def try_revisit(edge, j):
    return try_visit(edge, edge, j, j)

  metrics[None, i] = (0.0, 0.0, None)
  for p in projections:
    (a, b) = p.object
    try_visit(None, (a,b), i, i)
    try_visit(None, (b,a), i, i)

  while not queue.empty():
    (edge, i) = queue.get()
    print edge, i
    if edge in visited and i < visited[edge]:
      continue
    else:
      visited[edge] = j
    if i == len(states):
      break


    try_revisit(edge, i)
    for next in graph.graph.neighbors(edge[1]):
      try_visit(edge, (edge[1], next), i)

  def backward_path():
    r = (edge, j)
    while r[0] != None:
      yield r[0]
      r = previous[r]

  return reversed(list(backward_path()))
