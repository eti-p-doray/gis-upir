import shapefile
import csv
import pyproj as proj
import shapely.geometry as sg
import json
import geojson as gj
import pickle
from scipy import linalg
from itertools import izip
import numpy as np
import math
import sys
sys.path.append('.')
from spatial_graph import SpatialGraph
from a_star import a_star_search
from utility import *
from kalman import *
from priority_queue import *

def import_trajectories(data):
  rows = iter(data)
  header = rows.next()
  latitude_idx = next(i for i,v in enumerate(header) if v == "latitude")
  longitude_idx = next(i for i,v in enumerate(header) if v == "longitude")
  speed_idx = next(i for i,v in enumerate(header) if v == "speed")
  time_idx = next(i for i,v in enumerate(header) if v == "recorded_at")
  hort_acc_idx = next(i for i,v in enumerate(header) if v == "hort_accuracy")
  vert_acc_idx = next(i for i,v in enumerate(header) if v == "vert_accuracy")

  srcProj = proj.Proj(init='epsg:4326')
  dstProj = proj.Proj(init='epsg:2950')

  observations = []
  accuracy = []
  previous_id = -1;
  for row in data:
    current_id = row[0]
    if current_id != previous_id:
      if observations:
        print previous_id
        if previous_id == '1007':
          yield {'observations': observations, 'accuracy':accuracy, 'id': previous_id}
        observations = []
      previous_id = current_id

    #current_time = datetime.strptime(row[time_idx], '%Y-%m-%d %H:%M:%S')
    coord = proj.transform(srcProj, dstProj, float(row[longitude_idx]), float(row[latitude_idx]))
    obs = [coord[0], coord[1], float(row[speed_idx])]
    quantile = 2.33
    acc = [float(row[hort_acc_idx])/quantile, float(row[vert_acc_idx])/quantile]
    observations.append(obs);
    accuracy.append(acc)
  
def obs_transition_speed(x):
  return [x[0], x[1], math.sqrt(x[2]**2 + x[3]**2)]

def obs_transition(state):
  return [state[0], state[1]]


def filter_state(trajectories, F, Q):
  P = np.identity(4) * 1000.0
  for t in trajectories:
    y = t['observations'][0]
    kf = KalmanFilter(y[0:2] + [0.0, 0.0], P)
    r = {'id': t['id'], 'observations':[], 'state':[], 'accuracy':[]}

    for o, a in izip(t['observations'], t['accuracy']):
      err = 0.0
      (xf, Pf) = kf.time_update(F, Q)
      if o[2] >= 0.0:
        R = np.diag([a[0]**2, a[1]**2, 0.5])
        (xf, Pf, err) = kf.unscented_measurment_update(o, obs_transition_speed, R)
      else:
        R = np.diag([a[0]**2, a[1]**2])
        (xf, Pf, err) = kf.unscented_measurment_update(o[0:2], obs_transition, R)
      if err > 200.0**2:
        if len(r['state']) >= 2:
          yield r
        r = {'id': t['id'], 'observations':[], 'state':[], 'accuracy':[]}
      else:
        r['state'].append((xf, Pf))
        r['observations'].append(o)
        r['accuracy'].append(a)

    if len(r['state']) >= 2:
      yield r

def smooth_state(trajectories, F, Q):
  for t in trajectories:
    state = t['state'][-1]
    kf = KalmanFilter(state[0], state[1])

    for i, s in drop(enumerate_reversed(t['state']), 1):
      t['state'][i] = kf.smooth_update(s, F, Q)
    yield t

def estimate_state(trajectories):
  F = np.identity(4)
  F[0][2] = 1.0
  F[1][3] = 1.0
  Q = np.diag([0.2, 0.2, 1.0, 1.0])
  return smooth_state(filter_state(trajectories, F, Q), F, Q)

def normal2d(v):
  return np.array([v[1], -v[0]])
def sqnorm2d(v):
  return v[0]**2 + v[1]**2
def norm2d(v):
  return math.sqrt(sqnorm2d(v))

def project_feature_segment((x, P), (a,b)):
  a, b = np.array(a), np.array(b)
  v = b - a
  n = normal2d(v) / norm2d(v)
  w = np.dot(n, a)

  H = np.matrix([
    [n[0], n[1], 0.0, 0.0],
    [0.0, 0.0, n[0], n[1]]])
  y = np.array([w, w])
  kf = KalmanFilter(x, P)
  (x, P, l) = kf.project_constraint(H, y)
  
  d = v / sqnorm2d(v)
  t = np.dot(kf.x[0:2]-a, d.T)
  D = np.asmatrix(np.hstack((d, [0.0, 0.0])))
  l2 = 0.0
  if t > 1.0:
    (x, P, l2) = kf.project_constraint(D, 1.0+np.dot(a, d))
  elif t < 0.0:
    (x, P, l2) = kf.project_constraint(D, 0.0+np.dot(a, d))
  return t, x, l+l2


def project_feature_edge(state, edge):
  best_l = Ellipsis
  for a,b in pairwise(edge.coords):
    (t, x, l) = project_feature_segment(state, (a,b))
    if l < best_l:
      best_l = l
      best_offset = t
      best_state = x
  return (best_offset, best_state, best_l)


def bb_bounds(x, y, width, height):
  return (x-width, y-width, x+width, y+height)

def project_feature((x, P), graph):
  quantile = 20.0
  ell = quantile * linalg.sqrtm(P[0:2,0:2])
  height = math.sqrt(ell[0][0]**2 + ell[1][0]**2)
  width = math.sqrt(ell[0][1]**2 + ell[1][1]**2)
  edges = graph.edge_intersection(bb_bounds(x[0], x[1], width, height))
  projections = {}

  for e in edges:
    edge = graph.orientate(e.object)
    (offset, state, cost) = project_feature_edge((x, P), graph.way(edge))
    projections[edge] = {'offset':offset, 'cost':cost, 'state':state}
  return projections

def shortest_path(graph, e1, s1, e2, s2, cost, heuristic):
  if e1 == (0,0) or e2 == (0,0):
    return [], 100.0*s1['state'].distance(s2['state'])
  if e1 == e2:
    return [], 0.0

  #if e1[0] == e2[0]:
  #if e1[0] == e2[1]:
  #if e1[1] == e2[0]:
  #if e1[1] == e2[1]: 

  #print e1, e2
  starts = [(e1[0], s1['offset']), (e1[1], graph.way(e1).length - s1['offset'])]
  #print starts, graph.way(e1).length
  goals = [e2[0], e2[1]]
  print 'a star'
  print e1, e2, s1, s2
  (came_from, cost_so_far) = a_star_search(graph, starts, goals, cost, heuristic)
  cost_so_far[e2[0]] += s2['offset']
  cost_so_far[e2[1]] += (graph.way(e2).length - s2['offset'])
  if cost_so_far[e2[0]] < cost_so_far[e2[1]]:
    end = e2[0]
  else:
    end = e2[1]

  path = [end]
  while came_from[path[-1]] != None:
    path.append(came_from[path[-1]])
  print path
  return list(reversed(path)), 0.0


def update_transition_cost(graph, previous_states, next_states):
  for e2, s2 in next_states.iteritems():
    best_cost = Ellipsis
    best_path = []
    def euclidian_distance(g, current):
      return g.graph.node[current]['geometry'].distance(s2['state'])
    def edge_cost(g, current, next):
      return g.way((current, next)).length

    queue = PriorityQueue()
    for e1, s1 in previous_states.iteritems():
      queue.put(e1, s1['cost'])

    while not queue.empty():
      e1 = queue.get()
      s1 = previous_states[e1]
      if (s1['cost'] > best_cost):
        break
      (path, cost) = shortest_path(graph, e1, s1, e2, s2, edge_cost, euclidian_distance)
      cost += s1['cost']
      #print cost
      if cost < best_cost:
        best_cost = cost
        best_path = path
    print best_cost

    #print s2['cost'], best_cost
    next_states[e2]['cost'] += best_cost
    
    #s2[3] = best_path


def map_match(trajectories, graph):
  for t in trajectories:
    for s in t['state']:
      projection = project_feature(s, graph)
      if len(projection) != 0:
        break

    queue = PriorityQueue()
    for e, p in drop(projection.iteritems(), 1):
      queue.put((e, p), p['cost'])
      print e, p
      #queue.put((0, e), s['cost'])

    #while not queue.empty():
      #(i, e1) = queue.get()
      #if i+1 == len(projections):
      #  break
      #for e2,s2 in projections[i+1].iteritems():

    #for p, s in izip(projections, t['state']):
    #  pass
      #next_states = project_feature(s, graph)
      #print next_states
      #update_transition_cost(graph, previous_states, next_states)
      #previous_states = next_states


      

    #last = t['state'][-1]
    #goal = sg.Point(first[0], first[1])
    #goal_nodes = graph.intersection(goal.buffer(distance_threshold))

    yield t



    #a_star_search(n.graph, 0, 10, distance_metric, euclidian_distance)


def make_geojson(data):
  features = []
  for i, t in izip(range(1), data):
    x = []
    for s in t['state']:
      x.append([s[0][0], s[0][1]])
    features.append(gj.Feature(geometry = sg.mapping(sg.LineString(x)), properties = {'type':'f', 'id':t['id']}))

    y = []
    for s in t['observations']:
      y.append([s[0], s[1]]),
    features.append(gj.Feature(geometry = sg.mapping(sg.LineString(y)), properties = {'type':'r', 'id':t['id']}))

  fc = gj.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc



csvr = csv.reader(open('data/bike_path/Chunk_1_mm.csv'))
traj = import_trajectories(csvr)
filtered_traj = estimate_state(traj)


x = np.array([0.5, 0.0, -0.5, 0.5])
P = np.diag([1, 1, 1, 1])
a = sg.Point([0, 1])
b = sg.Point([1, 2])

#project_feature_segment((x,P), (a,b))

#with open('data/osm/small.pickle', 'r') as f:
#  mtl = pickle.load(f)
#graph = SpatialGraph()
#graph.import_osm(mtl)
#graph.compress()
#graph.build_spatial_edge_index()


#mapped_traj = map_match(filtered_traj, graph)

with open('data/bike_path/1_filtered.json', 'w+') as f:
  json.dump(make_geojson(filtered_traj), f, indent=2)
