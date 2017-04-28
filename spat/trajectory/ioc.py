import sys, argparse
import pickle, geojson, json, math
import numpy as np
import itertools

from spat.trajectory.features import *
from spat.priority_queue import PriorityQueue
from spat.spatial_graph import SpatialGraph


def inverse_optimal_control(data,
                            eval_gradient, weights, 
                            learning_rate, precision, nb_epochs):
  b1 = 0.9
  b2 = 0.999
  b1t = b1
  b2t = b2
  e = 1e-8

  m = 0
  v = 0

  for i in range(nb_epochs):
    np.random.shuffle(data)
    for example in data:
      gradient = eval_gradient(weights, example)
      print 'gradient', gradient

      m = b1 * m + (1.0 - b1) * gradient
      v = b2 * v + (1.0 - b2) * np.square(gradient)
      mc = m / (1.0 - b1t) 
      vc = v / (1.0 - b2t)
      b1t *= b1
      b2t *= b2

      print 'diff', np.divide(mc, np.sqrt(vc) + e)
      weights -= learning_rate * np.divide(mc, np.sqrt(vc) + e)
      print 'weights ', weights
      print

  return weights

def a_star(graph, start, goal, weights, features_fn, heuristic_fn):
  #print start['link'], goal['link']
  queue = PriorityQueue()

  costs = {}
  features = {}
  previous = {}
  visited = {}
  (u, v) = start['link']
  length  = sg.LineString(graph.way((u, v))).length
  features[u, v] = features_fn((None, u, v), length - start['bounds'][0][1])
  features[v, u] = features_fn((None, v, u), start['bounds'][0][1])
  costs[u, v] = np.dot(features[u, v], weights)
  costs[v, u] = np.dot(features[v, u], weights)
  queue.put((u, v), costs[u, v] + heuristic_fn((u, v)))
  queue.put((v, u), costs[v, u] + heuristic_fn((v, u)))
  previous[u, v] = None
  previous[v, u] = None

  while not queue.empty():
    (u, v) = queue.get()
    #print u, v, costs[u, v]
    visited[(u, v)] = True

    if (u, v) == goal['link'] or (v, u) == goal['link']:
      break

    for (v, k) in graph.neighbors(v):
      #print '\t', v, k,
      if (v, k) in visited:
        #print
        continue;
      if (v, k) == goal['link']:
        new_features = features_fn((u, v, k), goal['bounds'][1][1]) + features[(u, v)]
      elif (k, v) == goal['link']:
        length  = sg.LineString(graph.way((v, k))).length
        new_features = features_fn((u, v, k), length - goal['bounds'][1][1]) + features[(u, v)]
      else:
        length  = sg.LineString(graph.way((v, k))).length
        new_features = features_fn((u, v, k), length) + features[(u, v)]

      new_cost = np.dot(new_features, weights)
      if (v, k) not in costs or new_cost < costs[(v, k)]:
        features[v, k] = new_features
        costs[v, k] = new_cost
        if (v, k) == goal['link'] or (k, v) == goal['link']:
          priority = new_cost
        else:
          priority = new_cost + heuristic_fn((v, k))
        #print new_cost, priority
        queue.put((v, k), priority)
        previous[(v, k)] = (u, v)
      #else:
        #print

  if (u, v) != goal['link'] and (v, u) != goal['link']:
    return None, 0

  def backtrack():
    node = (u, v)
    while node != None:
      yield node
      node = previous[node]

  return reversed(list(backtrack())), features[(u, v)]

def length_feature(predicate):
  def fn((u, v, k), length):
    if (predicate((v, k))):
      return length
    return 0.0
  return fn

def intersection_feature(predicate):
  def fn((u, v, k), length):
    if predicate(v):
      return 1
    return 0
  return fn

def turn_feature(graph, predicate):
  def fn((u, v, k), length):
    if u != None and k != None:
      angle = graph.turn_angle((u, v, k))
      if (predicate(math.degrees(angle))):
        return 1
    return 0
  return fn

def make_geojson(trajectories, graph):
  features = []
  for trajectory in trajectories:
    bp = []
    for e in trajectory['ioc']:
      bp.extend(graph.way(e))
    if len(bp) > 1:
      features.append(geojson.Feature(
        geometry = sg.mapping(sg.LineString(bp)), 
        properties = {'id':trajectory['id'], 
                      'features':trajectory['features'],
                      'type':'ioc'}))

  fc = geojson.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc

def main(argv):
  parser = argparse.ArgumentParser(description="""
    Learns weiths associated with a vector of features.
    """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--features', default = 'data/bike_path/features.json', 
    help='input pickle file of preprocessed (with spat.trajectory.preprocess) data.');
  parser.add_argument('--mapmatch', default = 'data/bike_path/mm.pickle', 
    help='input pickle file of preprocessed (with spat.trajectory.preprocess) data.');
  parser.add_argument('--facility', default = 'data/mtl_geobase/mtl.pickle', 
    help="""input pickle file containing the facility graph 
    (with spat.geobase.preprocess) representing the road network""");
  #parser.add_argument('-o', '--ofile', default = 'data/bike_path/weights.json', 
  #  help='output pickle file containing an array of segments');
  #parser.add_argument('--geojson',
  #  help='output geojson file to export constrained geometry');

  args = parser.parse_args()
  print 'features:', args.features
  print 'mapmatch:', args.mapmatch
  print 'facility:', args.facility

  np.set_printoptions(linewidth = 500)

  #print 'output file:', args.ofile

  with open(args.facility, 'r') as f:
    graph = pickle.load(f)
  graph.build_spatial_node_index()

  with open(args.mapmatch, 'r') as f:
    mm = pickle.load(f)

  with open(args.features, 'r') as f:
    features = json.load(f)

  end_of_facility = match_intersections(
    load_discontinuity("data/discontinuity/end_of_facility"), graph)
  change_of_facility_type = match_intersections(
    load_discontinuity("data/discontinuity/change_of_facility_type"), graph)
  intersections_disc = match_intersections(
    load_discontinuity("data/intersections/intersections_on_bike_network_with_change_in_road_type"), graph)
  traffic_lights = match_intersections(
    load_traffic_lights("data/traffic_lights/All_lights"), graph)


  def best_path(weights, trajectory):
    def extract_feature(segment, length):
      fcns = [
        length_feature(lambda link: True),
        length_feature(link_type_predicate(graph, any_cycling_link)),
        length_feature(link_type_predicate(graph, designated_roadway)),
        length_feature(link_type_predicate(graph, bike_lane)),
        length_feature(link_type_predicate(graph, seperate_cycling_link)),
        length_feature(link_type_predicate(graph, offroad_link)),
        length_feature(link_type_predicate(graph, other_road_type)),
        length_feature(link_type_predicate(graph, arterial_link)),
        length_feature(link_type_predicate(graph, collector_link)),
        length_feature(link_type_predicate(graph, highway_link)),
        length_feature(link_type_predicate(graph, local_link)),
        turn_feature(graph, left_turn),
        turn_feature(graph, right_turn),
        intersection_feature(lambda u: True),
        intersection_feature(intersection_collection(end_of_facility)),
        intersection_feature(intersection_collection(change_of_facility_type)),
        intersection_feature(intersection_collection(intersections_disc)),
        intersection_feature(intersection_collection(traffic_lights)),
      ]
      return np.array(map(lambda fn:fn(segment, length), fcns))

    goal = trajectory['segment'][-1]
    goal_point = sg.LineString(
        graph.way(goal['link'])).interpolate(goal['bounds'][1][1])

    def heuristic_fn((u, v)):
      return graph.intersection(v).distance(goal_point) * 0.5

    return a_star(graph, trajectory['segment'][0], trajectory['segment'][-1], weights, extract_feature, heuristic_fn)

  def feature_prediction(weights, trajectory):
    (path, features) = best_path(weights, trajectory)
    return features

  def gradient_fn(param, example):
    #weights = 1.0 - 2.0 / (1.0 + np.exp(param))
    feature = feature_prediction(param, example[1])
    print 'features', feature
    print 'exemple', example[0]
    return np.divide(example[0] - feature, example[0] + 1.0)

  data = []
  for trajectory in mm:
    if trajectory['id'] not in features:
      continue
    feature = features[trajectory['id']]
    example = (np.array([
        feature['length'],
        feature['length_cycling'],
        feature['length_designated_roadway'],
        feature['length_bike_lane'],
        feature['length_seperate_cycling_link'],
        feature['length_offroad'],
        feature['length_other_road'],
        feature['length_arterial'],
        feature['length_collector'],
        feature['length_highway'],
        feature['length_local'],
        feature['left_turn'],
        feature['right_turn'],
        feature['intersections'],
        feature['end_of_facility'],
        feature['change_of_facility_type'],
        feature['intersections_disc'],
        feature['traffic_lights'],
      ]), trajectory)
    data.append(example)

  weights = np.ones(18)

  weights = inverse_optimal_control(data, gradient_fn, weights, 
    0.01, 0.1, 200)



  #with open(args.ofile, 'w+') as f:
  #  pickle.dump(matched, f)


if __name__ == "__main__":
  main(sys.argv[1:])