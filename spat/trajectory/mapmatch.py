import pickle, geojson, json
import sys, getopt, math
import shapely.geometry as sg
from scipy import spatial
from scipy import linalg

from spat.spatial_graph import SpatialGraph
from spat.path_inference import PathInference
from spat.utility import *

def coords_fn(state):
    return state.x[0:2]

def statemap_fn(v):
  return np.asmatrix([
    np.hstack((v, np.zeros(2))),
    np.hstack((np.zeros(2), v))])

def parse_nodes(nodes, states, graph):
  previous_edge = 0
  current_way = None

  segment = { 'geometry': [] }
  for node in nodes:
    current_edge = node[0]

    if current_edge != previous_edge: # end of segment      
      previous_way = current_way
      if current_edge != 0:
        current_way = sg.LineString(graph.way(current_edge))
      else:
        current_way = None

      if segment['geometry']: # segment not empty
        segment['geometry'].pop() # last coord is not part of segment

        if previous_edge == 0: # current segment is floating
          coord = coords_fn(states[node])
          segment['geometry'].append(coord)
          projection = current_way.project(sg.Point(coord))
          segment['bounds'][1] = (False, projection)
        elif current_edge == 0: # next segment is floating
          projection = previous_way.project(sg.Point(coords_fn(states[previous_node])))
          segment['bounds'][1] = (False, projection)
        else:
          segment['bounds'][1] = ((True, current_way.length))

        yield segment

      segment = {
        'geometry': [],
        'bounds': [(True, 0.0), None],
        'link': current_edge
      }
      print current_edge
      if current_edge == 0: # current segment is floating
        coord = coords_fn(states[previous_node])
        segment['geometry'].append(coord)
        projection = previous_way.project(sg.Point(coord))
        segment['bounds'][0] = (False, projection)
      elif previous_edge == 0: # previous segment is floating
        projection = current_way.project(sg.Point(coords_fn(states[node])))
        segment['bounds'][0] = (False, projection)

    segment['geometry'].append(coords_fn(states[node]))

    previous_node = node
    previous_edge = current_edge

  if segment['geometry']:
    if previous_edge == 0: # last segment is floating
      segment['bounds'][1] = (False, 0.0)
      yield segment
    else:
      projection = previous_way.project(sg.Point(coords_fn(states[previous_node])))
      segment['bounds'][1] = (False, projection)
      yield segment


def map_match(trajectories, graph, heuristic_factor):

  def nearby_fn(state):
    quantile = 10.0
    ell = quantile * linalg.sqrtm(state.P[0:2,0:2])
    height = math.sqrt(ell[0][0]**2 + ell[1][0]**2)
    width = math.sqrt(ell[0][1]**2 + ell[1][1]**2)
    return graph.edge_intersection(
      bb_bounds(state.x[0], state.x[1], width, height))

  path = PathInference(graph, statemap_fn, coords_fn, nearby_fn, heuristic_factor, 100.0)
  for trajectory in trajectories:
    nodes, states = path.solve(trajectory['state'], trajectory['transition'])

    yield list(parse_nodes(nodes, states, graph))

def make_geojson(trajectories, graph):
  features = []
  for i, trajectory in enumerate(trajectories):
    mm = []
    for segment in trajectory:
      for point in segment['geometry']:
        mm.append(point)
      #features.append(geojson.Feature(
      #  geometry = sg.mapping(sg.Point(state['state'].x[0:2])), 
      #  properties = {'type':state['type'], 'cost':state['cost'], 'priority':state['priority'],'index':state['index']}))
    #print mm
      #if graph.has_edge(u, v):
        #mm.extend(graph.way((u,v)))
    features.append(geojson.Feature(
      geometry = sg.mapping(sg.LineString(mm)), 
      properties = {'id':i, 'type':'mm'}))

  fc = geojson.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc

def main(argv):
  inputfile = 'data/bike_path/smoothed.pickle'
  facilityfile = 'data/mtl_geobase/mtl.pickle'
  outputfile = 'data/bike_path/mm.pickle'
  heuristic_factor = 5.0
  try:
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile=","facility="])
  except getopt.GetoptError:
    print 'mapmatch [-i <inputfile>] [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'mapmatch [-i <inputfile>] [-o <outputfile>]'
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
  graph.build_spatial_edge_index()
  """with open(facilityfile, 'r') as f:
    facility = pickle.load(f)
  graph = SpatialGraph()
  graph.import_osm(facility)
  graph.compress()
  graph.build_spatial_edge_index()"""

  with open(inputfile, 'r') as f:
    data = pickle.load(f)

  matched = map_match(data, graph, heuristic_factor)
  result = list(matched)

  #print result

  with open(outputfile, 'w+') as f:
    pickle.dump(result, f)
  print 'done' 

  with open("data/bike_path/mm.json", 'w+') as f:
    json.dump(make_geojson(result, graph), f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])
