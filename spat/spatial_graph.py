import shapefile
import rtree
import networkx as nx
import shapely.geometry as sg
import geojson as gj
import collections

from utility import *

class SpatialGraph:
  def __init__(self):
    self.spatial_idx = rtree.index.Index()
    self.graph = nx.Graph()
    self.metadata = {}

  def has_node(self, item):
    return self.graph.has_node(item)

  def has_edge(self, u, v):
    return self.graph.has_edge(u, v)

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
        nodes = self.spatial_node_idx.intersection(g.buffer(distance_threshold).bounds)
        for k in nodes:
          if (g.distance(self.graph.node[k]['geometry']) <= distance_threshold and
              k != previous_idx):
            self.graph.add_edge(current_idx, k, 
              type = 'intersection',
              edge_id = idx)
            idx += 1

        self.spatial_node_idx.insert(current_idx, g.bounds)
        previous_idx = current_idx

  def import_osm(self, data):
    for i, n in data['nodes'].iteritems():
      self.graph.add_node(i, geometry = sg.Point(n['geometry']))
    idx = 0
    for k, w in data['ways'].iteritems():
      self.metadata[i] = w['tags']
      for i,j in pairwise(w['nodes']):
        self.graph.add_edge(i, j,
          way_id = k,
          edge_id = idx,
          type = 'way',
          order = i < j, 
          geometry = [])
        idx += 1

  def build_spatial_node_index(self):
    self.spatial_node_idx = rtree.index.Index()
    for n,p in self.graph.nodes_iter(data=True):
      self.spatial_node_idx.insert(n, p['geometry'].bounds)

  def build_spatial_edge_index(self):
    self.spatial_edge_idx = rtree.index.Index()
    for k,(i,j) in enumerate(self.graph.edges_iter()):
      self.spatial_edge_idx.insert(k, sg.LineString(self.way((i, j))).bounds, obj=(i,j))

  def node_collapse(self, u, n, v):
    self.graph.add_edge(u, v, 
      type = 'way',
      edge_id = self.graph[n][u]['edge_id'],
      way_id = self.graph[n][u]['way_id'],
      order = u < v,
      geometry = 
        list(self.inner_way((u,n))) + 
        [self.graph.node[n]['geometry']] + 
        list(self.inner_way((n,v))))
    self.graph.remove_node(n)

  def compress(self):
    for n in self.graph.nodes():
      adj = self.graph.neighbors(n)
      if len(adj) == 2:
        u, v = adj[0], adj[1]
        if self.graph.has_edge(u,v):
          continue
        if self.graph[n][u]['edge_id'] > self.graph[n][v]['edge_id']:
          u, v = v, u
        if (self.graph[n][u]['type'] == 'way' and
            self.graph[n][v]['type'] == 'way' and
            self.graph[n][u]['way_id'] == self.graph[n][v]['way_id']):
          self.node_collapse(u, n, v)

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

  def neighbors(self, n):
    return self.graph.neighbors(n)

  def node_intersection(self, bounds):
    return self.spatial_node_idx.intersection(bounds)

  def edge_intersection(self, bounds):
    return self.spatial_edge_idx.intersection(bounds, objects=True)

  def direction(self, u, v):
    return xor(u > v, self.graph[u][v]['order'])

  def orientate(self, u, v):
    if self.direction(u,v) == False:
      return u, v
    return u, v

  def intersection(self, i):
    return self.graph.node[i]['geometry']

  def way(self, (u, v)):
    yield self.graph.node[u]['geometry']
    if self.direction(u, v) == False:
      inner = reversed(self.graph[u][v]['geometry'])
    else:
      inner = self.graph[u][v]['geometry']
    for point in inner:
      yield point
    yield self.graph.node[v]['geometry']

  def inner_way(self, (u, v)):
    if self.direction(u, v) == False:
      inner = reversed(self.graph[u][v]['geometry'])
    else:
      inner = self.graph[u][v]['geometry']
    for point in inner:
      yield point

  def make_shp(self):
    sf = shapefile.Writer(shapefile.POLYLINE)
    sf.autoBalance = 1
    sf.field('first')
    sf.field('last')
    sf.field('way_id')
    sf.field('id')
    idx = 0
    for u, v, p in self.graph.edges(data=True):
      if p['type'] == 'way':
        line = sg.mapping(sg.LineString([self.way((u,v))]))
        sf.line(parts=[line['coordinates']])
        sf.record(first= u, last= v, way_id= p['way_id'], id= p['edge_id'])
        idx += 1
    return sf

  def make_geojson(self, epsg):
    features = []
    idx = 0
    for u, v, p in self.graph.edges_iter(data=True):
      if p['type'] == 'way':
        line = sg.mapping(sg.LineString(list(self.way((u,v)))))
        feature = gj.Feature(
          geometry = line,
          properties = {'first': u, 'last': v, 'way_id':p['way_id'], 'id': p['edge_id']})
        features.append(feature)
        idx += 1
    fc = gj.FeatureCollection(features)
    fc['crs'] = {'type': 'EPSG', 'properties': {'code': epsg}}
    return fc

