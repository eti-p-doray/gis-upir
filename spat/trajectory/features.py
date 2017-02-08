import pickle, geojson, json
import sys, getopt, math
import numpy as np
import shapely.geometry as sg
from scipy import spatial
import shapefile

from spat.spatial_graph import SpatialGraph
from spat.utility import *

def extract_length(trajectory, graph):
  for segment in trajectory:
    if segment['link'] == 0:
      length += sg.LineString(segment['geometry']).length
    else:
      length += segment['bounds'][1] - segment['bounds'][0]
  return length

def load_discontinuity(file):
  sf = shapefile.Reader(file)
  points = {}

  id_field = next(i for i,v in enumerate(sf.fields) if v[0] == "id_link")-1
  frequence_field = next(i for i,v in enumerate(sf.fields) if v[0] == "frequency")-1
  for s, r in zip(sf.shapes(), sf.records()):
    points[r[id_field]] = {
      'geometry': sg.Point(s.points[0]), 
      'properties': {'id': r[id_field], 'frequency': r[frequence_field]}}
  return points

def load_inter_discontinuity(file):
  sf = shapefile.Reader(file)
  points = {}

  for i, (s, r) in enumerate(zip(sf.shapes(), sf.records())):
    points[i] = {
      'geometry': sg.Point(s.points[0]), 
      'properties': {'id': i}}
  return points

def match_discontinuity(points, graph):
  discontinuity_index = {}
  for i, p in points.iteritems():
    geom = p['geometry']
    nearby = graph.edge_intersection(bb_bounds(geom.x, geom.y, 2, 2));

    for edge in nearby:
      distance = sg.LineString(graph.way(edge.object)).distance(geom)
      if distance < 2:
        if edge.object not in discontinuity_index:
          discontinuity_index[edge.object] = []
        discontinuity_index[edge.object].append(i)
  return discontinuity_index

def extract_discontinuity(trajectory, discontinuity_index):
  count = 0
  edge = 0
  nodes = trajectory['node']
  for node in nodes:
    if node[0] != edge and node[0] != 0:
      if node[0] in discontinuity_index:
        count += len(discontinuity_index[node[0]])
    edge = node[0]

  return count


def extract_features(trajectories, graph):
  end_of_facility = load_discontinuity("data/discontinuity/end_of_facility");
  change_of_facility_type = load_discontinuity("data/discontinuity/change_of_facility_type");
  intersections_disc = load_inter_discontinuity("data/intersections/intersections_on_bike_network_with_change_in_road_type");

  end_of_facility_idx = match_discontinuity(end_of_facility, graph)
  change_of_facility_type_idx = match_discontinuity(change_of_facility_type, graph)
  intersections_disc_idx = match_discontinuity(intersections_disc, graph)

  features = {}
  for i, trajectory in enumerate(trajectories):
    features[i] = {}
    features[i]['length'] = extract_length(trajectory, graph)

    features[i]['end_of_facility'] = extract_discontinuity(trajectory, end_of_facility_idx)
    features[i]['change_of_facility_type'] = extract_discontinuity(trajectory, change_of_facility_type_idx)
    features[i]['intersections_disc'] = extract_discontinuity(trajectory, intersections_disc_idx)
  return features

def main(argv):
  inputfile = 'data/bike_path/mm.pickle'
  facilityfile = 'data/osm/mtl.pickle'
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
    facility = pickle.load(f)
  graph = SpatialGraph()
  graph.import_osm(facility)
  graph.compress()
  graph.build_spatial_edge_index()

  with open(inputfile, 'r') as f:
    data = pickle.load(f)

  features = extract_features(data, graph)
  print features

  with open(outputfile, 'w+') as f:
    json.dump(features, f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])
