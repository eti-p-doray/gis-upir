import math
import numpy
from scipy import spatial, linalg
import shapely.geometry as sg

from spat.trajectory import features
from spat.path_inference import PathInference
from spat import utility

def coords_fn(state):
  return state.x[0:2]


def statemap_fn(v):
  return numpy.asmatrix([
    numpy.hstack((v, numpy.zeros(2))),
    numpy.hstack((numpy.zeros(2), v))])


def parse_nodes(nodes, states, graph):
  previous_edge = None
  index = 0
  current_way = None

  segment = { 'geometry': [], 'idx': [0] }
  for node in nodes:
    current_edge = node[0]

    #print node, coords_fn(states[node])

    if current_edge != previous_edge: # end of segment
      previous_way = current_way
      if current_edge != None:
        current_way = sg.LineString(graph.way(current_edge))
      else:
        current_way = None

      if segment['geometry']: # segment not empty
        #segment['geometry'].pop() # last coord is not part of segment
        segment['idx'].append(node[1])

        if previous_edge is None: # current segment is floating
          coord = coords_fn(states[node])
          segment['geometry'].append(coord)
          projection = current_way.project(sg.Point(coord))
          segment['bounds'][1] = (False, projection)
        elif current_edge is None: # next segment is floating
          projection = previous_way.project(sg.Point(coords_fn(states[previous_node])))
          segment['bounds'][1] = (False, projection)
        else:
          segment['bounds'][1] = (True, previous_way.length)

        print(' ', segment['link'], segment['idx'])

        yield segment

      segment = {
        'geometry': [],
        'bounds': [(True, 0.0), None],
        'link': current_edge,
        'idx': [index]
      }

      if current_edge is None: # current segment is floating
        coord = coords_fn(states[previous_node])
        segment['geometry'].append(coord)
        projection = previous_way.project(sg.Point(coord))
        segment['bounds'][0] = (False, projection)
      elif previous_edge is None: # previous segment is floating
        projection = current_way.project(sg.Point(coords_fn(states[node])))
        segment['bounds'][0] = (False, projection)
        segment['idx']

    segment['geometry'].append(coords_fn(states[node]))

    index = node[1]
    previous_node = node
    previous_edge = current_edge

  if segment['geometry']:
    segment['idx'].append(node[1])
    assert previous_edge is not None
    projection = current_way.project(sg.Point(coords_fn(states[previous_node])))
    segment['bounds'][1] = (False, projection)
    yield segment


def solve(trajectories, graph, heuristic_factor):

  def nearby_fn(state, quantile):
    ell = quantile * linalg.sqrtm(state.P[0:2,0:2])
    height = math.sqrt(ell[0][0]**2 + ell[1][0]**2)
    width = math.sqrt(ell[0][1]**2 + ell[1][1]**2)
    #print height, width

    return graph.search_edges(
      utility.bb_bounds(state.x[0], state.x[1], width, height))

  distance_costs = numpy.array([
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
  intersection_costs = numpy.array([
    1.0, 0.0])

  end_of_facility = features.match_intersections(
    features.load_discontinuity("data/discontinuity/end_of_facility"), graph)

  def distance_cost_fn(distance, link):
    if link is None:
      return 300.0 * distance
    predicates = [
      lambda link: True,
      features.link_type_predicate(graph, features.any_cycling_link),
      features.link_type_predicate(graph, features.designated_roadway),
      features.link_type_predicate(graph, features.bike_lane),
      features.link_type_predicate(graph, features.seperate_cycling_link),
      features.link_type_predicate(graph, features.offroad_link),
      features.link_type_predicate(graph, features.other_road_type),
    ]
    return numpy.dot(
      numpy.array(map(lambda pred:pred(link), predicates)),
      distance_costs)

  def intersection_cost_fn(u, v, k):
    node_predicates = [
      lambda link: True,
      features.intersection_collection(end_of_facility),
    ]
    return numpy.dot(
      numpy.array(map(lambda pred:pred(v), node_predicates)),
      intersection_costs)

  path = PathInference(graph, statemap_fn, coords_fn,
                       distance_cost_fn, intersection_cost_fn,
                       nearby_fn, heuristic_factor)
  for trajectory in trajectories:
    print(trajectory['id'])
    nodes, states = path.solve(trajectory['state'], trajectory['transition'])
    if nodes is None:
      print('trashing ', trajectory['id'], ' due to incomplete mapmatch')
      continue
    segments = list(parse_nodes(nodes, states, graph))
    if len(segments) > 0:
      yield {
        'segment':segments,
        'id': trajectory['id'],
        'count': len(trajectory['state'])}