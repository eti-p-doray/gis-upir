import pickle, geojson, json
import sys, getopt, math
import shapely.geometry as sg
from scipy import spatial
from scipy import linalg

from spat.spatial_graph import SpatialGraph
from spat.path_inference import PathInference
from spat.utility import *

def map_match(trajectories, graph, heuristic_factor):

  def coords_fn(state):
    return state.x[0:2]

  def statemap_fn(v):
    return np.asmatrix([
      np.hstack((v, np.zeros(2))),
      np.hstack((np.zeros(2), v))])

  def nearby_fn(state):
    quantile = 10.0
    ell = quantile * linalg.sqrtm(state.P[0:2,0:2])
    height = math.sqrt(ell[0][0]**2 + ell[1][0]**2)
    width = math.sqrt(ell[0][1]**2 + ell[1][1]**2)
    return graph.edge_intersection(
      bb_bounds(state.x[0], state.x[1], width, height))

  path = PathInference(graph, statemap_fn, coords_fn, nearby_fn, 5.0, 80.0)
  for trajectory in trajectories:
    trajectory['mm'] = path.solve(trajectory['state'], trajectory['transition'])
    yield trajectory

def make_geojson(trajectories, graph):
  features = []
  for i, trajectory in enumerate(trajectories):
    if not trajectory['mm']:
      continue
    mm = []
    #print trajectory['mm']
    for state in trajectory['mm']:

      mm.append(state['state'].x[0:2])
      #features.append(geojson.Feature(
      #  geometry = sg.mapping(sg.Point(state['state'].x[0:2])), 
      #  properties = {'type':state['type'], 'cost':state['cost'], 'priority':state['priority'],'index':state['index']}))
    #print mm
      #if graph.has_edge(u, v):
        #mm.extend(graph.way((u,v)))
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
  heuristic_factor = 2.0
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
