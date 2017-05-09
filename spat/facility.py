import shapefile
import rtree
import networkx as nx
import shapely.geometry as sg
import geojson
import numpy

from spat import utility


class SpatialGraph:
    def __init__(self):
        self.graph = nx.Graph()
        self.metadata = {}

    def has_node(self, item):
        return self.graph.has_node(item)

    def has_edge(self, u, v):
        return self.graph.has_edge(u, v)

    def adjacent(self, u):
        for v in self.graph.neighbors(u):
            yield (u, v)

    def search_nodes(self, bounds):
        return self.spatial_node_idx.intersection(bounds)

    def search_edges(self, bounds):
        return self.spatial_edge_idx.intersection(bounds, objects=True)

    def direction(self, u, v):
        return utility.xor(u > v, self.graph[u][v]['order'])

    def orient(self, u, v):
        if not self.direction(u, v):
            return u, v
        return u, v

    def edge(self, u, v):
        return self.graph[u][v]

    def node(self, u):
        return self.graph.node[u]

    def edge_geometry_size(self, u, v):
        return len(self.graph[u][v]['geometry']) + 1

    def edge_geometry(self, u, v):
        yield self.graph.node[u]['geometry']
        if not self.direction(u, v):
            inner = reversed(self.graph[u][v]['geometry'])
        else:
            inner = self.graph[u][v]['geometry']
        for point in inner:
            yield point
        yield self.graph.node[v]['geometry']

    def inner_edge_geometry(self, u, v):
        if not self.direction(u, v):
            inner = reversed(self.graph[u][v]['geometry'])
        else:
            inner = self.graph[u][v]['geometry']
        for point in inner:
            yield point

    def node_geometry(self, i):
        return self.graph.node[i]['geometry']

    # turn angle in radians. 0 is staight, negative is right.
    def turn_angle(self, u, v, k):
        link0 = self.graph[u][v]['geometry']
        if link0:
            p0 = utility.point_to_vec(link0[-1])
        else:
            p0 = utility.point_to_vec(self.graph.node[u]['geometry'])

        p1 = utility.point_to_vec(self.graph.node[v]['geometry'])

        link1 = self.graph[v][k]['geometry']
        if link1:
            p2 = utility.point_to_vec(link1[0])
        else:
            p2 = utility.point_to_vec(self.graph.node[k]['geometry'])

        v0 = numpy.array(p1) - numpy.array(p0)
        v1 = numpy.array(p2) - numpy.array(p1)
        return numpy.math.atan2(numpy.linalg.det([v0,v1]), numpy.dot(v0,v1))

    def node_collapse(self, u, n, v):
        self.graph.add_edge(u, v,
                            edge_id = self.graph[n][u]['edge_id'],
                            way_id = self.graph[n][u]['way_id'],
                            order = u < v,
                            geometry =
                                list(self.inner_edge_geometry(u,n)) +
                                [self.graph.node[n]['geometry']] +
                                list(self.inner_edge_geometry(n,v)))
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
                if self.graph[n][u]['way_id'] == self.graph[n][v]['way_id']:
                    self.node_collapse(u, n, v)

    def build_spatial_node_index(self):
        self.spatial_node_idx = rtree.index.Index()
        for n,p in self.graph.nodes_iter(data=True):
            self.spatial_node_idx.insert(n, p['geometry'].bounds)

    def build_spatial_edge_index(self):
        self.spatial_edge_idx = rtree.index.Index()
        for k,(i,j) in enumerate(self.graph.edges_iter()):
            self.spatial_edge_idx.insert(k, sg.LineString(self.edge_geometry(i, j)).bounds, obj=(i,j))

    def import_geobase(self, data, distance_threshold = 1.0):
        for segment in data:
            properties = segment['properties']

            first_geom = sg.Point(segment['geometry'].coords[0])
            last_geom = sg.Point(segment['geometry'].coords[-1])

            def find_node(point, ignore):
                nodes = self.spatial_node_idx.intersection(
                    utility.bb_buffer(point, distance_threshold))
                best = None
                min_distance = numpy.inf
                for k in nodes:
                    if k == ignore:
                        continue
                    distance = point.distance(self.graph.node[k]['geometry'])
                    if distance <= min_distance:
                        min_distance = distance
                        best = k
                if best is None:
                    best = self.graph.order()
                    self.graph.add_node(best, geometry = point)
                    self.spatial_node_idx.insert(best, point.bounds)
                return best

            first_node = find_node(first_geom, None)
            last_node = find_node(last_geom, first_node)

            self.graph.add_edge(first_node, last_node,
                                geometry = [sg.Point(x) for x in segment['geometry'].coords[1:-1]],
                                order = first_node < last_node,
                                **properties)

    def import_osm(self, data):
        for i, n in data['nodes'].iteritems():
            self.graph.add_node(i, geometry = sg.Point(n['geometry']))
        idx = 0
        for k, w in data['ways'].iteritems():
            self.metadata[i] = w['tags']
            for i,j in utility.pairwise(w['nodes']):
                self.graph.add_edge(i, j,
                                    way_id = k,
                                    edge_id = idx,
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
            line = sg.mapping(sg.LineString([self.edge_geometry(u,v)]))
            sf.line(parts=[line['coordinates']])
            sf.record(first= u, last= v, way_id= p['way_id'], id= p['edge_id'])
            idx += 1
        return sf

    def make_geojson(self, epsg):
        features = []
        idx = 0
        for u, v, p in self.graph.edges_iter(data=True):
            line = sg.mapping(sg.LineString(list(self.edge_geometry(u,v))))
            feature = geojson.Feature(
                geometry = line,
                properties = {'first': u, 'last': v})
            features.append(feature)
            idx += 1
        fc = geojson.FeatureCollection(features)
        fc['crs'] = {'type': 'EPSG', 'properties': {'code': epsg}}
        return fc

    def export_index(self):
        index = {'neighbors':{}, 'way':{}, 'metadata':{}}
        for n in self.graph.node_iter():
            index['way'][n] = []
        for i,j,p in self.graph.edges_iter(data=True):
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

