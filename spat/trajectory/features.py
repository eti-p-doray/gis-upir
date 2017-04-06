import sys, getopt, argparse, fnmatch
import pickle, csv, geojson, json, math
import numpy as np
import itertools
import shapely.geometry as sg
from scipy import spatial
import shapefile
import rtree

from spat.spatial_graph import SpatialGraph
from spat.utility import *

class RegionPartition:
  def __init__(self, filename):
    sf = shapefile.Reader(filename);

    self.regions = {}
    self.spatial_idx = rtree.index.Index()
    for s, r in zip(sf.iterShapes(), sf.iterRecords()):
      ring = sg.Polygon(s.points)
      self.spatial_idx.insert(r[0], ring.bounds)
      self.regions[r[0]] = ring

  def fit(self, point):
    print point.bounds
    neighbors = self.spatial_idx.intersection(point.bounds)
    for i in neighbors:
      print '  ', i, self.regions[i]
      if (self.regions[i].contains(point)):
        return i
    return None

def extract_length(trajectory, graph, predicate):
  length = 0
  for segment in trajectory['segment']:
    if (predicate(segment['link'])):
      if segment['link'] == None:
        length += sg.LineString(segment['geometry']).length
      else:
        length += segment['bounds'][1][1] - segment['bounds'][0][1]
  return length

def load_point_list(file):
  sf = shapefile.Reader(file)

  for s, r in zip(sf.iterShapes(), sf.iterRecords()):
    yield {'geometry': sg.Point(s.points[0])}

def load_discontinuity(file):
  return load_point_list(file)

def load_traffic_lights(file):
  return load_point_list(file)

def match_intersections(points, graph):
  intersection_index = {}
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
    if best_node not in intersection_index:
      intersection_index[best_node] = []
    intersection_index[best_node].append(i)
  return intersection_index

def extract_intersection(trajectory, predicate):
  count = 0

  for segment in trajectory['segment']:
    if (segment['link'] != None and segment['bounds'][1][0] == True):
      if predicate(segment['link'][1]):
        count += 1
  return count

def extract_turn(trajectory, graph, predicate):
  count = 0

  for segment0, segment1 in pairwise(trajectory['segment']):
    if (segment0['link'] != None and segment0['bounds'][1][0] == True and
        segment1['link'] != None and segment1['bounds'][0][0] == True):

      angle = graph.turn_angle((segment0['link'][0], 
                                segment0['link'][1], 
                                segment1['link'][1]))
      if (predicate(math.degrees(angle))):
        count += 1
  return count

def link_type_predicate(graph, predicate):
  def fn(link):
    if link == None:
      return False
    return predicate(graph.edge(link)['type'])
  return fn

def any_cycling_link(link_type):
  return link_type > 10
def designated_roadway(link_type):
  return link_type == 11
def bike_lane(link_type):
  return link_type == 13
def seperate_cycling_link(link_type):
  return link_type in {14, 15, 16}
def offroad_link(link_type):
  return link_type == 17
def other_road_type(link_type):
  return link_type in {1, 2, 3, 4, 9}
def arterial_link(link_type):
  return link_type in {6, 7}
def collector_link(link_type):
  return link_type == 5
def highway_link(link_type):
  return link_type == 8
def local_link(link_type):
  return link_type == 0

def left_turn(angle):
  return angle > 45.0
def right_turn(angle):
  return angle < -45.0

def intersection_collection(collection):
  def fn(node):
    return node in collection
  return fn


def extract_features(trajectories, graph):
  end_of_facility = match_intersections(
    load_discontinuity("data/discontinuity/end_of_facility"), graph)
  change_of_facility_type = match_intersections(
    load_discontinuity("data/discontinuity/change_of_facility_type"), graph)
  intersections_disc = match_intersections(
    load_discontinuity("data/intersections/intersections_on_bike_network_with_change_in_road_type"), graph)
  traffic_lights = match_intersections(
    load_traffic_lights("data/traffic_lights/All_lights"), graph)

  partition = RegionPartition("data/partition/ZT2008_1631_v2b_region")

  features = {}
  for trajectory in trajectories:
    i = trajectory['id']
    features[i] = {}
    features[i]['length'] = extract_length(trajectory, graph, 
      lambda link: True)
    features[i]['length_cycling'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, any_cycling_link))
    features[i]['length_designated_roadway'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, designated_roadway))
    features[i]['length_bike_lane'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, bike_lane))
    features[i]['length_seperate_cycling_link'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, seperate_cycling_link))
    features[i]['length_offroad'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, offroad_link))
    features[i]['length_other_road'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, other_road_type))
    features[i]['length_arterial'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, arterial_link))
    features[i]['length_collector'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, collector_link))
    features[i]['length_highway'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, highway_link))
    features[i]['length_local'] = extract_length(trajectory, graph, 
      link_type_predicate(graph, local_link))

    features[i]['left_turn'] = extract_turn(trajectory, graph, left_turn)
    features[i]['right_turn'] = extract_turn(trajectory, graph, right_turn)

    features[i]['intersections'] = extract_intersection(trajectory, lambda link: True)
    features[i]['end_of_facility'] = extract_intersection(trajectory, 
      intersection_collection(end_of_facility))
    features[i]['change_of_facility_type'] = extract_intersection(trajectory, 
      intersection_collection(change_of_facility_type))
    features[i]['intersections_disc'] = extract_intersection(trajectory, 
      intersection_collection(intersections_disc))
    features[i]['traffic_lights'] = extract_intersection(trajectory, 
      intersection_collection(traffic_lights))


    partition_begin = partition.fit(sg.Point(trajectory['segment'][0]['geometry'][0]))
    partition_end = partition.fit(sg.Point(trajectory['segment'][-1]['geometry'][-1]))
    features[i]['partition'] = (partition_begin, partition_end)

    #print len(trajectory['segment'])
    features[i]['duration'] = (trajectory['segment'][-1]['idx'][1] - 
                               trajectory['segment'][0]['idx'][0])
    features[i]['avg_speed'] = features[i]['length'] / features[i]['duration']

    #print features[i].values()
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
