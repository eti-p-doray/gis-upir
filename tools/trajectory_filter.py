import shapefile, geojson, csv, json, pickle
import pyproj as proj
import shapely.geometry as sg
from itertools import izip
import numpy as np
import math

from spatial_graph import SpatialGraph
from kalman import KalmanFilter
from path_inference import infer_path
from utility import *

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
        if previous_id == '1015' or previous_id == '1013':
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

def infer_paths(trajectories, graph):
  for t in trajectories:
    t['path'] = list(infer_path(t['state'], graph))
    yield t


def make_geojson(data, graph):
  features = []
  for i, t in izip(range(1), data):
    x = []
    for s in t['state']:
      x.append([s[0][0], s[0][1]])
    features.append(geojson.Feature(geometry = sg.mapping(sg.LineString(x)), properties = {'type':'f', 'id':t['id']}))

    y = []
    for s in t['observations']:
      y.append([s[0], s[1]]),
    features.append(geojson.Feature(geometry = sg.mapping(sg.LineString(y)), properties = {'type':'r', 'id':t['id']}))

    z = []
    for e in t['path']:
      z.extend(graph.way(e)),
    features.append(geojson.Feature(geometry = sg.mapping(sg.LineString(z)), properties = {'type':'p'}))

  fc = geojson.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc

from pycallgraph import PyCallGraph
from pycallgraph.output import GraphvizOutput

#with PyCallGraph(output=GraphvizOutput()):


csvr = csv.reader(open('data/bike_path/Chunk_1_mm.csv'))
traj = import_trajectories(csvr)
filtered_traj = estimate_state(traj)


#project_feature_segment((x,P), (a,b))

with open('data/osm/jean-drapeau.pickle', 'r') as f:
  mtl = pickle.load(f)
graph = SpatialGraph()
graph.import_osm(mtl)
graph.compress()
graph.build_spatial_edge_index()
mapped_traj = infer_paths(filtered_traj, graph)

with open('data/bike_path/1_filtered.json', 'w+') as f:
  json.dump(make_geojson(mapped_traj, graph), f, indent=2)
