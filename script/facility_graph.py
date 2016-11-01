import itertools
import shapefile
import rtree
import networkx as nx
import shapely.geometry as sg
import geojson as gj
import pickle
import collections

def pairwise(iterable):
  a, b = itertools.tee(iterable)
  next(b, None)
  return itertools.izip(a, b)

def peek(iterable):
  try:
    first = next(iterable)
  except StopIteration:
    return None
  return first, itertools.chain([first], iterable)

def xor(a, b):
  return bool(a) ^ bool(b)

def empty(iterable):
  return peek(iterable) is None

def merge_dicts(*dict_args):
  result = {}
  for dictionary in dict_args:
    result.update(dictionary)
  return result

def node_collapse(g, n, i, j):
  g.add_edge(i, j, 
    type = 'way',
    edge_id = g[n][i]['edge_id'],
    way_id = g[n][i]['way_id'],
    order = i < j,
    geometry = g[n][i]['geometry'] + [g.node[n]['geometry']] + g[n][j]['geometry'])
  g.remove_node(n)

def compress_graph(g):
  for n in g.nodes():
    adj = g.neighbors(n)
    if (len(adj) == 2):
      i, j = adj[0], adj[1]     
      if g[n][i]['edge_id'] > g[n][j]['edge_id']:
        i, j = j, i
      if (g[n][i]['type'] == 'way' and
          g[n][j]['type'] == 'way' and
          g[n][i]['way_id'] == g[n][j]['way_id']):
        node_collapse(g, n, i, j)

def make_geojson(g, epsg):
  features = []
  k = 0
  for i,j,p in g.edges_iter(data=True):
    if p['type'] == 'way':
      if xor(i > j, g[i][j]['order'] == False):
        i, j = j, i
      line = sg.mapping(sg.LineString([g.node[i]['geometry']] + p['geometry'] + [g.node[j]['geometry']]))
      feature = gj.Feature(
        geometry = line,
        properties = {'first': i, 'last': j, 'edge_id': p['edge_id'], 'way_id':p['way_id']})
      features.append(feature)
  fc = gj.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': epsg}}
  return fc

def make_shp(g):
  sf = shapefile.Writer(shapefile.POLYLINE)
  sf.autoBalance = 1
  sf.field('first')
  sf.field('last')
  sf.field('way_id')
  sf.field('edge_id')
  for i,j,p in g.edges(data=True):
    if p['type'] == 'way':
      if xor(i > j, g[i][j]['order'] == False):
        i, j = j, i
      line = sg.mapping(sg.LineString([g.node[i]['geometry']] + p['geometry'] + [g.node[j]['geometry']]))
      sf.line(parts=[line['coordinates']])
      sf.record(first= i, last= j, way_id= p['way_id'], edge_id= p['edge_id'])
  return sf

class FacilityGraph:
  def __init__(self):
    self.spatial_idx = rtree.index.Index()
    self.graph = nx.Graph()
    self.metadata = {}

  def import_geobase(self, data, distance_threshold = 1.0):
    idx = 0
    for d in data:
      previous_idx = -1
      first_idx = self.graph.order()
      for p in d['geometry'].coords:
        current_idx = self.graph.order()
        g = sg.Point(p)
        self.graph.add_node(current_idx, geometry = g)
        if (previous_idx != -1):
          self.graph.add_edge(previous_idx, current_idx, 
            way_id = d['id'],
            edge_id = idx,
            type = 'way',
            geometry = [],
            order = True)
          idx += 1
        inter = self.spatial_idx.intersection(g.buffer(distance_threshold).bounds)
        for k in inter:
          if (g.distance(self.graph.node[k]['geometry']) <= distance_threshold and
              k != previous_idx):
            self.graph.add_edge(current_idx, k, 
              type = 'intersection',
              edge_id = idx)
            idx += 1

        self.spatial_idx.insert(current_idx, g.bounds)
        previous_idx = current_idx

  def import_osm(self, data):
    for id, n in data['nodes'].iteritems():
      self.graph.add_node(id, geometry = sg.Point(n['geometry']))
    idx = 0
    for id, w in data['ways'].iteritems():
      self.metadata[id] = w['tags']
      for i,j in pairwise(w['nodes']):
        self.graph.add_edge(i, j,
          way_id = id,
          edge_id = idx,
          type = 'way',
          order = i < j, 
          geometry = [])
        idx += 1

  def build_spatial_index(self):
    self.spatial_idx = rtree.index.Index()
    for n,p in self.graph.nodes(data=True):
      self.spatial_idx.insert(n, p['geometry'].bounds)

  def compress(self):
    compress_graph(self.graph)

  def export_index(self):
    index = {'neighbors':{}, 'way':{}, 'metadata':{}}
    for n in self.graph.node_iter():
      index['intersection'][n] = []
      index['way'][n] = []
    for i,j,p in self.graph.edges_iter(data=True):
      if p['type'] == 'intersection':
        index['intersection'][i].append(j)
        index['intersection'][j].append(i)
      if p['type'] == 'way':
        index['way'][i].append(p['edge_id'])
        index['way'][j].append(p['edge_id'])
      index['metadata'][p['way_id']] = self.metadata[p['way_id']]
    return index

  def neighbors(n):
    return self.graph.neighbors(n)

  def way(i, j):
    if xor(i > j, self.graph[i][j]['order'] == False):
      i, j = j, i
    return sg.mapping(sg.LineString([self.graph.node[i]['geometry']] + p['geometry'] + [self.graph.node[j]['geometry']]))

  def export_shp(self):
    return make_shp(self.graph)
  def export_geojson(self, epsg):
    return make_geojson(self.graph, epsg)

  def __getstate__(self):
    odict = self.__dict__.copy()
    del odict['spatial_idx']
    return odict

  def __setstate__(self, dict):
    self.__dict__.update(dict)
    build_spatial_index()
