import sys, os, fnmatch, argparse
import pickle, csv, geojson, json, math, datetime
import pyproj as proj
from itertools import izip
# Note: importing shapely before osgeo triggers an Import error; _GEOSArea not found.
import shapely.geometry as sg 

import numpy as np

from spat.kalman import KalmanFilter
from spat.utility import *

def load_csv(data):
  rows = iter(data)
  header = rows.next()
  #print header
  latitude_idx = next(i for i,v in enumerate(header) if v == "latitude")
  longitude_idx = next(i for i,v in enumerate(header) if v == "longitude")
  speed_idx = next(i for i,v in enumerate(header) if v == "speed")
  altitude_idx = next(i for i,v in enumerate(header) if v == "altitude")
  time_idx = next(i for i,v in enumerate(header) if v == "recorded_at")
  hort_acc_idx = next(i for i,v in enumerate(header) if v == "hort_accuracy")
  vert_acc_idx = next(i for i,v in enumerate(header) if v == "vert_accuracy")
  src_node_idx = next(i for i,v in enumerate(header) if v == "src")
  dst_node_idx = next(i for i,v in enumerate(header) if v == "dst")

  srcProj = proj.Proj(init='epsg:4326')
  dstProj = proj.Proj(init='epsg:2950')

  observations = []
  accuracy = []
  link = []
  previous_id = -1;
  for row in data:
    current_id = row[0]
    if current_id != previous_id:
      if observations:
        print previous_id
        yield {
          'observations': observations, 
          'accuracy': accuracy, 
          'id': previous_id,
          'link': link
        }
        observations = []
        accuracy = []
        link = []
      previous_id = current_id
      previous_time = None

    current_time = datetime.datetime.strptime(row[time_idx], '%Y-%m-%d %H:%M:%S')
    #print row[time_idx]
    while (previous_time != None and 
           previous_time + datetime.timedelta(seconds=1) < current_time):
      observations.append(None)
      accuracy.append(None)
      link.append(None)
      previous_time += datetime.timedelta(seconds=1)

    previous_time = current_time
    try:
      coord = proj.transform(srcProj, dstProj, 
        float(row[longitude_idx]), 
        float(row[latitude_idx]))
    except RuntimeError:
      previous_id = -1
      continue
    obs = [coord[0], coord[1], float(row[speed_idx])]
    quantile = 1.96
    acc = [float(row[hort_acc_idx])/quantile, float(row[vert_acc_idx])/quantile]
    observations.append(obs);
    accuracy.append(acc)
    link.append((int(row[src_node_idx]), int(row[dst_node_idx])))

def load_all(files, max_count):
  for filepath in files:
    with open(filepath) as csvfile:
      data = csv.reader(csvfile)
      for trajectory in load_csv(data):
        yield trajectory
        max_count -= 1
        if max_count == 0:
          return
  
def obs_transition_speed(x):
  return [x[0], x[1], math.sqrt(x[2]**2 + x[3]**2)]

def obs_transition(x):
  return [x[0], x[1]]

def filter_state(trajectories):
  P = np.identity(4) * 10.0
  F = np.identity(4)
  F[0][2] = 1.0
  F[1][3] = 1.0
  Q = np.diag([2.0, 2.0, 2.0, 2.0])

  def init_trajectory(t):
    return {
        'id': t['id'], 
        'observations':[], 
        'link': [],
        'state':[], 
        'accuracy':[],
        'transition': (F, Q)
      }

  for trajectory in trajectories:
    y = trajectory['observations'][0]
    state = KalmanFilter(y[0:2] + [0.0, 0.0], P)
    trajectory['state'] = []
    trajectory['transition'] = (F, Q)

    for observation, accuracy, link in izip(
        trajectory['observations'], 
        trajectory['accuracy'], 
        trajectory['link']):
      error = 0.0
      state.time_update(F, Q)
      if observation != None:
        if observation[2] >= 0.0:
          R = np.diag([accuracy[0]**2, accuracy[1]**2, 1.0])
          error = state.unscented_measurment_update(observation, 
                                                    obs_transition_speed, R)
        else:
          R = np.diag([accuracy[0]**2, accuracy[1]**2])
          error = state.unscented_measurment_update(observation[0:2], 
                                                    obs_transition, R)

        if error > 50.0**2: # trajectory is broken
          print 'trashing ', trajectory['id'], ' due to broken path'
          break
      trajectory['state'].append(state.copy())

    if len(trajectory['state']) >= 2:
      yield trajectory

def smooth_state(trajectories):
  filtered_trajectories = filter_state(trajectories)
  for t in filtered_trajectories:
    (F, Q) = t['transition']
    next_state = t['state'][-1]

    broken = False
    for i, _ in drop(enumerate_reversed(t['state']), 1):
      t['state'][i].smooth_update(next_state, F, Q)
      next_state = t['state'][i]
      if math.log10(np.linalg.det(next_state.P)) > 4*next_state.P.shape[0]:
        print 'trashing ', t['id'], ' due to broken path'
        broken = True
        break

    if not broken:
      yield t


def make_geojson(trajectories):
  features = []
  for trajectory in trajectories:
    
    state = [[s.x[0], s.x[1]] for s in trajectory['state']]
    features.append(geojson.Feature(
      geometry = sg.mapping(sg.LineString(state)), 
      properties = {
        'id':trajectory['id'], 
        'type':'state'}))

  fc = geojson.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc


def main(argv):
  parser = argparse.ArgumentParser(description="""
    Read bike trajectory in csv format and apply a preprocess step
    that includes optimal smoothing with an autoregressive model,
    as well as filtering out broken trajectories.
    """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('-i', '--idir', default = 'data/bike_path', 
    help="""directory containing input data files. 
            All *.csv files will be imported""");
  parser.add_argument('-o', '--ofile', default = 'data/bike_path/smoothed.pickle', 
    help='output pickle file to export serialized result');
  parser.add_argument('--geojson',
    help='output geojson file to export smoothed geometry');
  parser.add_argument('--max', default = np.inf, type=int,
    help='maximum number of trajectory that will be processed');

  args = parser.parse_args()
  print 'input directory:', args.idir
  print 'found input files:'
  for file in os.listdir(args.idir):
    if fnmatch.fnmatch(file, '*.csv'):
      print ' ', args.idir + file
  print

  print 'output file:', args.ofile 

  files = (os.path.join(args.idir, file) 
    for file in os.listdir(args.idir) 
    if fnmatch.fnmatch(file, '*.csv'))

  trajectories = load_all(files, args.max)
  smoothed_trajectories = list(smooth_state(trajectories))

  with open(args.ofile, 'w+') as f:
    pickle.dump(smoothed_trajectories, f)
  if args.geojson != None:
    with open(args.geojson, 'w+') as f:
      json.dump(make_geojson(smoothed_trajectories), f, indent=2)
  print 'done'  

if __name__ == "__main__":
  main(sys.argv[1:])
