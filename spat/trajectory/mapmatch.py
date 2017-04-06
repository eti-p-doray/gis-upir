import sys, argparse
import pickle, geojson, json, math
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
    np.hstack((v, np.zeros(4))),
    np.hstack((np.zeros(3), v, np.zeros(1)))])

def parse_nodes(nodes, states, graph):
  previous_edge = None
  current_way = None

  segment = { 'geometry': [] }
  for node in nodes:
    current_edge = node[0]

    if current_edge != previous_edge: # end of segment      
      previous_way = current_way
      if current_edge != None:
        current_way = sg.LineString(graph.way(current_edge))
      else:
        current_way = None

      if segment['geometry']: # segment not empty
        segment['geometry'].pop() # last coord is not part of segment
        segment['idx'].append(node[1])

        if previous_edge == None: # current segment is floating
          coord = coords_fn(states[node])
          segment['geometry'].append(coord)
          projection = current_way.project(sg.Point(coord))
          segment['bounds'][1] = (False, projection)
        elif current_edge == None: # next segment is floating
          projection = previous_way.project(sg.Point(coords_fn(states[previous_node])))
          segment['bounds'][1] = (False, projection)
        else:
          segment['bounds'][1] = (True, previous_way.length)

        print ' ', segment['link'], segment['idx']
        yield segment

      segment = {
        'geometry': [],
        'bounds': [(True, 0.0), None],
        'link': current_edge,
        'idx': [node[1]]
      }
      if current_edge == None: # current segment is floating
        coord = coords_fn(states[previous_node])
        segment['geometry'].append(coord)
        projection = previous_way.project(sg.Point(coord))
        segment['bounds'][0] = (False, projection)
      elif previous_edge == None: # previous segment is floating
        projection = current_way.project(sg.Point(coords_fn(states[node])))
        segment['bounds'][0] = (False, projection)

    segment['geometry'].append(coords_fn(states[node]))

    previous_node = node
    previous_edge = current_edge

  if segment['geometry']:
    segment['idx'].append(previous_node[1])
    assert previous_edge != None
    projection = current_way.project(sg.Point(coords_fn(states[previous_node])))
    segment['bounds'][1] = (False, projection)
    yield segment


def map_match(trajectories, graph, heuristic_factor):

  def nearby_fn(state, quantile):
    ell = quantile * linalg.sqrtm(state.P[0:2,0:2])
    height = math.sqrt(ell[0][0]**2 + ell[1][0]**2)
    width = math.sqrt(ell[0][1]**2 + ell[1][1]**2)
    #print height, width

    return graph.edge_intersection(
      bb_bounds(state.x[0], state.x[1], width, height))

  path = PathInference(graph, statemap_fn, coords_fn, nearby_fn, heuristic_factor, 200.0)
  for trajectory in trajectories:
    print trajectory['id']
    nodes, states = path.solve(trajectory['state'], trajectory['transition'])
    if nodes == None:
      print 'trashing ', trajectory['id'], ' due to incomplete mapmatch'
      continue
    segments = list(parse_nodes(nodes, states, graph))
    if (len(segments) > 0):
      yield {
        'segment':segments,
        'id': trajectory['id'],
        'count': len(trajectory['state'])}

def make_geojson(trajectories, graph):
  features = []
  for trajectory in trajectories:
    mm = []
    for segment in trajectory['segment']:
      for point in segment['geometry']:
        mm.append(point)
    if len(mm) > 1:
      features.append(geojson.Feature(
        geometry = sg.mapping(sg.LineString(mm)), 
        properties = {'id':trajectory['id'], 'type':'mm'}))

  fc = geojson.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc

def main(argv):
  parser = argparse.ArgumentParser(description="""
    Mapmatch preprocessed bike trajectories based on a facility graph.
    The outcome is a pickle file describing every step of the trajectory in 
    terms of segments that may be linked to an edge in the facility graph.
    Each segment also have its own geometry, which is the result of constraining 
    the state on the current link.
    """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('-i', '--ifile', default = 'data/bike_path/smoothed.pickle', 
    help='input pickle file of preprocessed (with spat.trajectory.preprocess) data.');
  parser.add_argument('--facility', default = 'data/mtl_geobase/mtl.pickle', 
    help="""input pickle file containing the facility graph 
    (with spat.geobase.preprocess) representing the road network""");
  parser.add_argument('-o', '--ofile', default = 'data/bike_path/mm.pickle', 
    help='output pickle file containing an array of segments');
  parser.add_argument('--geojson',
    help='output geojson file to export constrained geometry');
  parser.add_argument('--factor', default = 10.0, type=float,
    help='heuristic factor. Higher is more greedy');

  args = parser.parse_args()
  print 'input file:', args.ifile
  print 'facility:', args.facility
  print 'output file:', args.ofile

  with open(args.facility, 'r') as f:
    graph = pickle.load(f)
  graph.build_spatial_edge_index()

  with open(args.ifile, 'r') as f:
    preprocessed = pickle.load(f)

  matched = list(map_match(preprocessed, graph, args.factor))

  with open(args.ofile, 'w+') as f:
    pickle.dump(matched, f)
  if args.geojson != None:
    with open(args.geojson, 'w+') as f:
      json.dump(make_geojson(matched, graph), f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])
