import shapefile
import rtree
import networkx as nx
import shapely.geometry as sg
import geojson as gj
import collections

from utility import *

class SpatialGraph:
  def __init__(self):
    self.graph = nx.Graph()
    self.metadata = {}

  def has_node(self, item):
    return self.graph.has_node(item)

  def has_edge(self, u, v):
    return self.graph.has_edge(u, v)

  def neighbors(self, u):
    for v in self.graph.neighbors(u):
      if (self.graph[u][v]['way'] == True):
        yield (u, v)
      else:
        n = self.graph.neighbors(v)
        if n[0] != u:
          yield (v, n[0])
        elif n[1] != u:
          yield (v, n[1])
        else:
          raise Exception('Invalid intersection')

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

  def is_way(self, (u, v)):
    return self.graph[u][v]['way']

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

  def node_collapse(self, u, n, v):
    self.graph.add_edge(u, v, 
      way = True,
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
        if (self.graph[n][u]['way'] == True and
            self.graph[n][v]['way'] == True and
            self.graph[n][u]['way_id'] == self.graph[n][v]['way_id']):
          self.node_collapse(u, n, v)

  def build_spatial_node_index(self):
    self.spatial_node_idx = rtree.index.Index()
    for n,p in self.graph.nodes_iter(data=True):
      self.spatial_node_idx.insert(n, p['geometry'].bounds)

  def build_spatial_edge_index(self):
    self.spatial_edge_idx = rtree.index.Index()
    for k,(i,j) in enumerate(self.graph.edges_iter()):
      self.spatial_edge_idx.insert(k, sg.LineString(self.way((i, j))).bounds, obj=(i,j))

  def import_geobase(self, data, first_idx, distance_threshold = 1.0):
    edge_idx = self.graph.size()
    print first_idx
    for segment in data:
      properties = segment['properties']

      last_idx = first_idx + len(segment['geometry'].coords)
      first_geom = sg.Point(segment['geometry'].coords[0])
      last_geom = sg.Point(segment['geometry'].coords[-1])
      """if segment['geometry'].length > 5000.0:
        print 'boubou', segment['geometry'].length
        for p in segment['geometry'].coords:
          print p.x, p.y
        print"""
      if first_geom.distance(last_geom) > 5000:
        print first_geom.distance(last_geom)
        #print segment['geometry'].coords

      self.graph.add_node(first_idx, geometry = first_geom)
      self.graph.add_node(last_idx,  geometry = last_geom)
      self.graph.add_edge(first_idx, last_idx, 
          edge_id = edge_idx,
          way = True,
          geometry = [sg.Point(x) for x in segment['geometry'].coords[1:-1]],
          order = True,
          **properties)

      def add_intersections(point, idx, before):
        nodes = self.spatial_node_idx.intersection(
          bb_buffer(point, distance_threshold))
        for k in nodes:
          if (point.distance(self.graph.node[k]['geometry']) <= 
                distance_threshold and
              k < before):
            self.graph.add_edge(idx, k, 
              way = False,
              edge_id = edge_idx,
              order = True, 
              geometry = [])
            """d = self.graph.node[idx]['geometry'].distance(
              self.graph.node[k]['geometry'])
            if d > 1000:
              print d"""
            #print self.graph.node[idx]['geometry'].distance(
            #  self.graph.node[k]['geometry'])
      add_intersections(first_geom, first_idx, first_idx)
      add_intersections(last_geom, last_idx, first_idx)

      self.spatial_node_idx.insert(first_idx, first_geom.bounds)
      self.spatial_node_idx.insert(last_idx, last_geom.bounds)
      edge_idx += 1
      first_idx = last_idx + 1

    return first_idx

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
          way = True,
          order = i < j, 
          geometry = [])
        idx += 1

  def make_shp(self):
    sf = shapefile.Writer(shapefile.POLYLINE)
    sf.autoBalance = 1
    sf.field('first')
    sf.field('last')
    sf.field('way_id')
    sf.field('id')
    idx = 0
    for u, v, p in self.graph.edges(data=True):
      if p['way'] == True:
        line = sg.mapping(sg.LineString([self.way((u,v))]))
        sf.line(parts=[line['coordinates']])
        sf.record(first= u, last= v, way_id= p['way_id'], id= p['edge_id'])
        idx += 1
    return sf

  def make_geojson(self, epsg):
    features = []
    idx = 0
    for u, v, p in self.graph.edges_iter(data=True):
      if p['way'] == True:
        line = sg.mapping(sg.LineString(list(self.way((u,v)))))
        feature = gj.Feature(
          geometry = line,
          properties = {'first': u, 'last': v, 'way_id':p['way_id'], 'id': p['edge_id']})
        features.append(feature)
        idx += 1
    fc = gj.FeatureCollection(features)
    fc['crs'] = {'type': 'EPSG', 'properties': {'code': epsg}}
    return fc

  def export_index(self):
    index = {'neighbors':{}, 'way':{}, 'metadata':{}}
    for n in self.graph.node_iter():
      index['intersection'][n] = []
      index['way'][n] = []
    for i,j,p in self.graph.edges_iter(data=True):
      if p['way'] == False:
        index['intersection'][i].append(j)
        index['intersection'][j].append(i)
      if p['way'] == True:
        index['way'][i].append(p['edge_id'])
        index['way'][j].append(p['edge_id'])
      index['metadata'][p['way_id']] = self.metadata[p['way_id']]
    return index

  def __getstate__(self):
    odict = self.__dict__.copy()
    if 'spatial_node_idx' in odict:
      del odict['spatial_node_idx']
    if 'spatial_edge_idx' in odict:
      del odict['spatial_edge_idx']
    return odict

  def __setstate__(self, dict):
    self.__dict__.update(dict)

