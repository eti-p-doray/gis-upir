import sys, getopt, argparse, fnmatch
import pickle, csv, geojson, json, math
import numpy as np
import shapely.geometry as sg
from scipy import spatial
import shapefile

from spat.spatial_graph import SpatialGraph
from spat.utility import *

def extract_length(trajectory, graph, predicate):
  length = 0
  for segment in trajectory:
    if (predicate(segment['link'])):
      if segment['link'] == None:
        length += sg.LineString(segment['geometry']).length
      else:
        length += segment['bounds'][1][1] - segment['bounds'][0][1]
  return length

def load_discontinuity(file):
  sf = shapefile.Reader(file)
  points = []

  for s, r in zip(sf.shapes(), sf.records()):
    points.append({'geometry': sg.Point(s.points[0])})
  return points

def match_discontinuity(points, graph):
  discontinuity_index = {}
  for i, p in enumerate(points):
    geom = p['geometry']
    nearby = graph.node_intersection(bb_bounds(geom.x, geom.y, 2, 2));

    min_distance = np.inf
    best_node = None
    for node in nearby:
      distance = graph.intersection(node).distance(geom)
      if distance < min_distance:
        min_distance = distance
        best_node = node
    if best_node not in discontinuity_index:
      discontinuity_index[best_node] = []
    discontinuity_index[best_node].append(i)
  return discontinuity_index

def extract_discontinuity(trajectory, discontinuity_index):
  count = 0

  for segment in trajectory:
    if segment['link'] != None and segment['bounds'][1][0] == True:
      node = segment['link'][1]
      if node in discontinuity_index:
        count += len(discontinuity_index[node])
  return count


def extract_features(trajectories, graph):
  end_of_facility = match_discontinuity(
    load_discontinuity("data/discontinuity/end_of_facility"), graph)
  change_of_facility_type = match_discontinuity(
    load_discontinuity("data/discontinuity/change_of_facility_type"), graph)
  intersections_disc = match_discontinuity(
    load_discontinuity("data/intersections/intersections_on_bike_network_with_change_in_road_type"), graph)

  def any_link(link):
    return True
  def any_cycling_link(link):
    if link == None:
      return False
    return graph.edge(link)['type'] > 10
  def designated_roadway(link):
    if link == None:
      return False
    return graph.edge(link)['type'] == 11
  def bike_lane(link):
    if link == None:
      return False
    return graph.edge(link)['type'] == 13
  def seperate_cycling_link(link):
    if link == None:
      return False
    return graph.edge(link)['type'] in {14, 15, 16}
  def offroad(link):
    if link == None:
      return False
    return graph.edge(link)['type'] == 17


  features = []
  for i, trajectory in enumerate(trajectories):
    features.append({})
    features[i]['length'] = extract_length(trajectory, graph, 
      any_link)
    features[i]['length_cycling'] = extract_length(trajectory, graph, 
      any_cycling_link)
    features[i]['length_designated_roadway'] = extract_length(trajectory, graph, 
      designated_roadway)
    features[i]['length_bike_lane'] = extract_length(trajectory, graph, 
      bike_lane)
    features[i]['length_seperate_cycling_link'] = extract_length(trajectory, graph, 
      seperate_cycling_link)
    features[i]['length_offroad'] = extract_length(trajectory, graph, 
      offroad)

    features[i]['end_of_facility'] = extract_discontinuity(
      trajectory, end_of_facility)
    features[i]['change_of_facility_type'] = extract_discontinuity(
      trajectory, change_of_facility_type)
    features[i]['intersections_disc'] = extract_discontinuity(
      trajectory, intersections_disc)

    features[i]['left_turn'] = extract_turn(
      trajectory, left_turn)

    print features[i].values()
  return features

def main(argv):
  parser = argparse.ArgumentParser(description="""
    Extract several features from mapmatched bike trajectories.
    Features are:
    length
    length_cycling
    length_designated_roadway
    length_bike_lane
    length_seperate_cycling_link
    length_offroad

    end_of_facility
    change_of_facility_type
    intersections_disc
    """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('-i', '--ifile', default = 'data/bike_path/mm.pickle', 
    help='input pickle file of mapmatched (with spat.trajectory.mapmatch) data.');
  parser.add_argument('--facility', default = 'data/mtl_geobase/mtl.pickle', 
    help="""input pickle file containing the facility graph 
    (with spat.geobase.preprocess) representing the road network""");
  parser.add_argument('-o', '--ofile', 
    default = ['data/bike_path/features.json'], nargs='+',
    help="""output file containing an array of segments. 
    Supported formats include *.json, *.csv""");

  args = parser.parse_args()
  print 'input file:', args.ifile
  print 'facility:', args.facility
  print 'output file:', args.ofile

  with open(args.facility, 'r') as f:
    graph = pickle.load(f)
  graph.build_spatial_node_index()

  with open(args.ifile, 'r') as f:
    data = pickle.load(f)

  features = extract_features(data, graph)

  for output in args.ofile:
    with open(output, 'w+') as f:
      if fnmatch.fnmatch(output, '*.csv'):
        writer = csv.writer(f)
        for row in features:
          print row.values()
          writer.writerow(row.values())
      elif fnmatch.fnmatch(output, '*.json'):
        json.dump(features, f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])
