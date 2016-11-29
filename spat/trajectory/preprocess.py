import pickle, csv, pyproj as proj, math
import sys, getopt, os, fnmatch
from itertools import izip

import numpy as np

from spat.kalman import KalmanFilter
from spat.utility import *

def load_csv(data):
  rows = iter(data)
  header = rows.next()
  latitude_idx = next(i for i,v in enumerate(header) if v == "latitude")
  longitude_idx = next(i for i,v in enumerate(header) if v == "longitude")
  speed_idx = next(i for i,v in enumerate(header) if v == "speed")
  time_idx = next(i for i,v in enumerate(header) if v == "recorded_at")
  hort_acc_idx = next(i for i,v in enumerate(header) if v == "hort_accuracy")
  vert_acc_idx = next(i for i,v in enumerate(header) if v == "vert_accuracy")
  src_node_idx = next(i for i,v in enumerate(header) if v == "src")
  dst_node_idx = next(i for i,v in enumerate(header) if v == "dst")

  srcProj = proj.Proj(init='epsg:4326')
  dstProj = proj.Proj(init='epsg:2950')

  observations = []
  accuracy = []
  edges = []
  previous_id = -1;
  for row in data:
    current_id = row[0]
    if current_id != previous_id:
      if observations:
        print previous_id
        #if previous_id == '1013':
        yield {
          'observations': observations, 
          'accuracy': accuracy, 
          'id': previous_id,
          'edges': edges
        }
        observations = []
      previous_id = current_id

    #current_time = datetime.strptime(row[time_idx], '%Y-%m-%d %H:%M:%S')
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
    edges.append((int(row[src_node_idx]), int(row[dst_node_idx])))

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
  P = np.identity(4) * 1000.0
  F = np.identity(4)
  F[0][2] = 1.0
  F[1][3] = 1.0
  Q = np.diag([2.0, 2.0, 2.0, 2.0])

  for t in trajectories:
    y = t['observations'][0]
    state = KalmanFilter(y[0:2] + [0.0, 0.0], P)
    result = {
      'id': t['id'], 
      'observations':[], 
      'edges': [],
      'state':[], 
      'accuracy':[],
      'transition': (F, Q)
    }

    for o, a, e in izip(t['observations'], t['accuracy'], t['edges']):
      err = 0.0
      state.time_update(F, Q)
      if o[2] >= 0.0:
        R = np.diag([a[0]**2, a[1]**2, 1.0])
        err = state.unscented_measurment_update(o, obs_transition_speed, R)
      else:
        R = np.diag([a[0]**2, a[1]**2])
        err = state.unscented_measurment_update(o[0:2], obs_transition, R)
      if err > 200.0**2:
        if len(result['state']) >= 2:
          yield result
        result = {
          'id': t['id'], 
          'observations':[],
          'edges': [],
          'state':[], 'accuracy':[],
          'transition': (F, Q)
        }
      else:
        result['state'].append(state.copy())
        result['observations'].append(o)
        result['accuracy'].append(a)
        result['edges'].append(e)

    if len(result['state']) >= 2:
      yield result

def smooth_state(trajectories):
  filtered_trajectories = filter_state(trajectories)
  for t in filtered_trajectories:
    (F, Q) = t['transition']
    next_state = t['state'][-1]

    for i, _ in drop(enumerate_reversed(t['state']), 1):
      t['state'][i].smooth_update(next_state, F, Q)
      next_state = t['state'][i]
    yield t


def main(argv):
  inputdir = 'data/bike_path'
  outputfile = 'data/bike_path/smoothed.pickle'
  max_count = Ellipsis

  try:
    opts, args = getopt.getopt(argv,"hi:o:",["idir=","ofile=","max="])
  except getopt.GetoptError:
    print 'smooth [-i <inputdir>] [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'smooth [-i <inputdir>] [-o <outputfile>]'
      sys.exit()
    elif opt in ("-i", "--idir"):
      inputdir = arg
    elif opt in ("-o", "--ofile"):
      outputfile = arg
    elif opt in ("--max"):
      max_count = int(arg)

  print 'input directory:', inputdir
  print 'output file:', outputfile 

  print 'found input files:'
  for file in os.listdir(inputdir):
    if fnmatch.fnmatch(file, '*.csv'):
      print ' ', inputdir + file
  print

  files = (os.path.join(inputdir, file) 
    for file in os.listdir(inputdir) 
    if fnmatch.fnmatch(file, '*.csv'))

  trajectories = load_all(files, max_count)
  smoothed_trajectories = smooth_state(trajectories)
  result = list(smoothed_trajectories)

  with open(outputfile, 'w+') as f:
    pickle.dump(result, f)
  print 'done'  

if __name__ == "__main__":
  main(sys.argv[1:])
