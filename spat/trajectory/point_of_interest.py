from sklearn import cluster
from sklearn.neighbors import kneighbors_graph
import rtree
import shapely.geometry as sg

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

  poi_to_trajectory = []
  for index, trajectory in enumerate(trajectories):
    for state in extract_poi(trajectory):
      coord = state.x[0:2]
      result['coord'].append(coord)
      poi_to_trajectory.append(index)

  cluster_algorithm = cluster.AffinityPropagation(preference = -100000.0)
  result['label'] = cluster_algorithm.fit_predict(result['coord'])
  result['center'] = cluster_algorithm.cluster_centers_indices_

  for poi, index in enumerate(poi_to_trajectory):
    label = result['label'][poi]
    if label not in result['trajectories']:
      result['trajectories'][label] = set([])
    result['trajectories'][label].add(index)

  spatial_idx = rtree.index.Index()
  for label, center in enumerate(result['center']):
    coord = result['coord'][center]
    spatial_idx.insert(label, sg.Point(coord).bounds)

  print result['label']
  print result['center'] 

  distance_threshold = 100.0
  for index, trajectory in enumerate(trajectories):
    for state in trajectory['state']:
      coord = state.x[0:2]
      point = sg.Point(coord)
      pois = spatial_idx.intersection(point.buffer(distance_threshold).bounds)
      for label in pois:
        center = sg.Point(result['coord'][result['center'][label]])
        if center.distance(point) < distance_threshold:
          if label not in result['trajectories']:
            result['trajectories'][label] = set([])
          result['trajectories'][label].add(index)


  return result

  