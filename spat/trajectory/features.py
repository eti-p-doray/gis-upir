import pickle, geojson, json
import sys, getopt, math
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
  points = {}

  id_field = next(i for i,v in enumerate(sf.fields) if v[0] == "id_link")-1
  for s, r in zip(sf.shapes(), sf.records()):
    points[r[id_field]] = {
      'geometry': sg.Point(s.points[0]), 
      'properties': {'id': r[id_field]}}
  return points

def load_discontinuity_inter(file):
  sf = shapefile.Reader(file)
  points = {}

  id_field = next(i for i,v in enumerate(sf.fields) if v[0] == "nb_classes")-1
  for s, r in zip(sf.shapes(), sf.records()):
    points[r[id_field]] = {
      'geometry': sg.Point(s.points[0])}
  return points

def match_discontinuity(points, graph):
  discontinuity_index = {}
  for i, p in points.iteritems():
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
    load_discontinuity_inter("data/intersections/intersections_on_bike_network_with_change_in_road_type"), graph)

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

  features = {}
  for i, trajectory in enumerate(trajectories):
    features[i] = {}
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

    print features[i].values()
  return features

def main(argv):
  inputfile = 'data/bike_path/mm.pickle'
  facilityfile = 'data/mtl_geobase/mtl.pickle'
  outputfile = 'data/bike_path/features.json'
  try:
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile=","facility="])
  except getopt.GetoptError:
    print 'features [-i <inputfile>] [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'features [-i <inputfile>] [-o <outputfile>]'
      sys.exit()
    if opt in ("-i", "--ifile"):
      inputfile = arg
    if opt in ("--facility"):
      facilityfile = arg
    elif opt in ("-o", "--ofile"):
      outputfile = arg

  print 'input file:', inputfile
  print 'facility:', facilityfile
  print 'output file:', outputfile

  with open(facilityfile, 'r') as f:
    graph = pickle.load(f)
  graph.build_spatial_node_index()

  with open(inputfile, 'r') as f:
    data = pickle.load(f)

  features = extract_features(data, graph)
  print features

  with open(outputfile, 'w+') as f:
    json.dump(features, f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])
