from sklearn import cluster
from sklearn.neighbors import kneighbors_graph

import numpy as np

from spat.kalman import KalmanFilter
from spat.utility import *

def extract_poi(trajectory):
  (F, Q) = trajectory['transition']
  stopped = True
  previous_state = trajectory['state'][0].copy()
  count = 20
  for state in trajectory['state']:
    if stopped == True:
      y = np.concatenate((state.x[0:2], np.zeros(2)))
      R = np.zeros((4,4))
      R[0:2,0:2] = state.P[0:2,0:2]
      state = previous_state.copy()
      state.time_update(F, Q)
      l = state.measurment_update(y, np.identity(4), R)

      if l > 2.33:
        if count >= 20:
          yield previous_state
        stopped = False
      previous_state = state
      count += 1

    else:
      l = state.constraint_distance(np.zeros(2), np.identity(4)[2:4,:])
      if l < 0.05:
        stopped = True
        count = 0
        previous_state = state.copy()

  yield state.copy()

def cluster_trajectories(trajectories):
  result = {'coord': [], 'trajectories': {}}

  for trajectory in trajectories:
    for state in extract_poi(trajectory):
      coord = state.x[0:2]
      result['coord'].append(coord)
      #result['trajectories'].append(index)

  cluster_algorithm = cluster.AffinityPropagation(preference = -1000000.0)
  result['label'] = cluster_algorithm.fit_predict(result['coord'])
  result['center'] = cluster_algorithm.cluster_centers_indices_

  print result['label']
  print result['center'] 

  for index, trajectory in enumerate(trajectories):
    for state in trajectory['state']:
      coord = state.x[0:2]
      label = cluster_algorithm.predict([coord])
      # possibly set threshold
      if label[0] not in result['trajectories']:
        result['trajectories'][label[0]] = set([])
      result['trajectories'][label[0]].add(index)

  # possibly go over all points in trajectories
  """
  label_map = {}
  for l, i in zip(result['label'], result['trajectories']):
    if l not in label_map:
      label_map[l] = []
    label_map[l].append(i)

  for s in label_map:
    label_map[s] = set(label_map[s])"""

  """od_pair = {}
  for i in label_map:
    for j in label_map:
      if i < j:
        intersection = label_map[i] & label_map[j]
        if intersection:
          print i, j, intersection
          od_pair[i,j] = intersection"""

  return result

  