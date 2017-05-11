import shapefile
import rtree
import networkx as nx
import shapely.geometry as sg
import geojson
import numpy

from spat import utility


class SpatialGraph:
    def __init__(self):
        self.graph = nx.MultiGraph()
        self.geometry = {}

    def has_node(self, item):
        return self.graph.has_node(item)

    def has_edge(self, u, v):
        return self.graph.has_edge(u, v)

    def adjacent(self, u):
        for v in self.graph[u]:
            for k in range(0, self.graph.number_of_edges(u, v)):
                yield (u, v, k)

    def search_node_intersection(self, bounds):
        return self.spatial_node_idx.intersection(bounds)

    def search_node_nearest(self, bounds, count):
        return self.spatial_node_idx.nearest(bounds, count)

    def search_edge_intersection(self, bounds):
        return self.spatial_edge_idx.intersection(bounds, objects=True)

    def search_edge_nearest(self, bounds, count):
        return self.spatial_edge_idx.nearest(bounds, count, objects=True)

    def valid_circulation(self, edge):
        u, v, k = edge
        if self.graph[u][v][k]['sens'] == 0:
            return True
        return not utility.xor(self.graph[u][v][k]['sens'] > 0, self.ordered(edge))

    def ordered(self, edge):
        u, v, k = edge
        return utility.xor(u > v, self.graph[u][v][k]['order'])

    def orient(self, edge):
        u, v, k = edge
        if not self.ordered(edge):
            return u, v, k
        return u, v, k

    def edge(self, edge):
        u, v, k = edge
        return self.graph[u][v][k]

    def node(self, u):
        return self.graph.node[u]

    def edge_geometry(self, edge):
        return self.geometry[edge]
        """yield self.graph.node[u]['geometry']
        if not self.ordered(u, v):
            inner = reversed(self.graph[u][v]['geometry'])
        else:
            inner = self.graph[u][v]['geometry']
        for point in inner:
            yield point
        yield self.graph.node[v]['geometry']"""

    def node_geometry(self, i):
        return self.graph.node[i]['geometry']

    # turn angle in radians. 0 is staight, negative is right.
    def turn_angle(self, e1, e2):
        _, v, _ = e1

        link0 = self.edge_geometry(e1)
        p0 = link0.coords[-1]

        p1 = utility.point_to_vec(self.graph.node[v]['geometry'])

        link1 = self.edge_geometry(e2)
        p2 = link1.coords[0]

        v0 = numpy.array(p1) - numpy.array(p0)
        v1 = numpy.array(p2) - numpy.array(p1)
        return numpy.math.atan2(numpy.linalg.det([v0,v1]), numpy.dot(v0,v1))

    def build_spatial_node_index(self):
        self.spatial_node_idx = rtree.index.Index()
        for n,p in self.graph.nodes_iter(data=True):
            self.spatial_node_idx.insert(n, p['geometry'].bounds)

    def build_spatial_edge_index(self):
        self.spatial_edge_idx = rtree.index.Index()
        for i,edge in enumerate(self.graph.edges_iter(keys=True)):
            self.spatial_edge_idx.insert(i, self.edge_geometry(edge).bounds, obj=edge)

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
                                order = first_node < last_node,
                                **properties)
            k = self.graph.number_of_edges(first_node, last_node)-1
            self.geometry[first_node, last_node, k] = sg.LineString(segment['geometry'].coords)
            self.geometry[last_node, first_node, k] = sg.LineString(reversed(segment['geometry'].coords))

    def make_shp(self):
        sf = shapefile.Writer(shapefile.POLYLINE)
        sf.autoBalance = 1
        sf.field('first')
        sf.field('last')
        sf.field('way_id')
        sf.field('id')
        idx = 0
        for u, v, k, p in self.graph.edges(data=True,keys=True):
            line = sg.mapping(self.edge_geometry((u,v,k)))
            sf.line(parts=[line['coordinates']])
            sf.record(first= u, last= v, way_id= p['way_id'], id= p['edge_id'])
            idx += 1
        return sf

    def make_geojson(self, epsg):
        features = []
        idx = 0
        for u, v, k, p in self.graph.edges_iter(data=True, keys=True):
            line = sg.mapping(self.edge_geometry((u,v,k)))
            feature = geojson.Feature(
                geometry = line,
                properties = {'first': u, 'last': v})
            features.append(feature)
            idx += 1
        fc = geojson.FeatureCollection(features)
        fc['crs'] = {'type': 'EPSG', 'properties': {'code': epsg}}
        return fc

    def __getstate__(self):
        odict = self.__dict__.copy()
        if 'spatial_node_idx' in odict:
            del odict['spatial_node_idx']
        if 'spatial_edge_idx' in odict:
            del odict['spatial_edge_idx']
        return odict

    def __setstate__(self, odict):
        self.__dict__.update(odict)

