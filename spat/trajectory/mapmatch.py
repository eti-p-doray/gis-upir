import pickle, geojson, json
import sys, getopt
import shapely.geometry as sg

from spat.spatial_graph import SpatialGraph
from spat.path_inference import infer_path

def map_match(trajectories, graph, heuristic_factor):
  for trajectory in trajectories:
    if trajectory['id'] == '8014':
      trajectory['edges'] = list(infer_path(trajectory['state'], trajectory['transition'], graph, heuristic_factor))
      yield trajectory
      break

def make_geojson(trajectories, graph):
  features = []
  for i, trajectory in enumerate(trajectories):
    mm = []
    for u,v in trajectory['edges']:
      if graph.has_edge(u, v):
        mm.extend(graph.way((u,v)))
    features.append(geojson.Feature(
      geometry = sg.mapping(sg.LineString(mm)), 
      properties = {'id':i, 'oid':trajectory['id'], 'type':'mm'}))

  fc = geojson.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc

def main(argv):
  inputfile = 'data/bike_path/smoothed.pickle'
  facilityfile = 'data/osm/mtl.pickle'
  outputfile = 'data/bike_path/mm.json'
  heuristic_factor = 30.0
  try:
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile=","facility="])
  except getopt.GetoptError:
    print 'mapmatch [-i <inputfile>] [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'mapmatch [-i <filteredfile>] [-o <classfile>]'
      sys.exit()
    if opt in ("-i", "--ifile"):
       inputfile = arg
    if opt in ("-i", "--facility"):
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

  data = map_match(data, graph, heuristic_factor)

  with open(outputfile, 'w+') as f:
    json.dump(make_geojson(data, graph), f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])
