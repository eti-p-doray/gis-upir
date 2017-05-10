import logging
import math
import numpy
import shapefile
import rtree
import pyproj
import shapely.geometry as sg

from spat import utility, raster


class RegionPartition:
    def __init__(self, filename):
        sf = shapefile.Reader(filename)

        self.regions = {}
        self.spatial_idx = rtree.index.Index()
        for s, r in zip(sf.iterShapes(), sf.iterRecords()):
            ring = sg.Polygon(s.points)
            self.spatial_idx.insert(r[0], ring.bounds)
            self.regions[r[0]] = ring

    def fit(self, point):
        neighbors = self.spatial_idx.intersection(point.bounds)
        for i in neighbors:
            if self.regions[i].contains(point):
                return i
        return None


def extract_length(trajectory, predicate):
    length = 0
    for segment in trajectory['segment']:
        if predicate(segment.edge):
            if segment.edge is None:
                length += sg.LineString(segment.geometry).length
            else:
                length += segment.end.projection - segment.begin.projection
    return length


def load_point_list(file):
    sf = shapefile.Reader(file)

    for s, r in zip(sf.iterShapes(), sf.iterRecords()):
        yield {'geometry': sg.Point(s.points[0])}


def load_discontinuity(file):
    return load_point_list(file)


def load_traffic_lights(file):
    return load_point_list(file)


def match_intersections(points, graph):
    intersection_index = {}
    for i, p in enumerate(points):
        geom = p['geometry']
        nearby = graph.search_node_nearest(utility.bb_bounds(geom.x, geom.y, 2, 2), 3)

        min_distance = math.inf
        best_node = None
        for node in nearby:
            distance = graph.node_geometry(node).distance(geom)
            if distance < min_distance:
                min_distance = distance
                best_node = node
        if best_node not in intersection_index:
            intersection_index[best_node] = []
        intersection_index[best_node].append(i)
    return intersection_index


def extract_intersection(trajectory, predicate):
    count = 0

    for segment in trajectory['segment']:
        if segment.edge is not None and segment.end.attached:
            if predicate(segment.edge[1]):
                count += 1
    return count


def extract_turn(trajectory, graph, predicate):
    count = 0

    for segment0, segment1 in utility.pairwise(trajectory['segment']):
        if (segment0.edge is not None and segment0.end.attached and
            segment1.edge is not None and segment1.begin.attached):

            assert segment0.edge[1] == segment1.edge[0]
            angle = graph.turn_angle(segment0.edge[0], segment0.edge[1], segment1.edge[1])
            if predicate(math.degrees(angle)):
                count += 1
    return count


def extract_nodes(trajectory):
    segments = trajectory['segment']
    for segment in segments:
        yield segment.geometry[0]
    yield segments[-1].geometry[-1]


def extract_elevation_stats(trajectory, graph, elevation):
    dst_proj = pyproj.Proj(init='epsg:4326')
    for n1, n2 in utility.pairwise(extract_nodes(trajectory)):
        e1 = elevation.at(n1, dst_proj)
        e2 = elevation.at(n2, dst_proj)
        d = sg.Point(n1).distance(sg.Point(n2))
        if d > 0.0:
          slope = (e2 - e1) / d
          M2 += slope ** 2
          M3 += slope ** 3
    return M2, M3


def link_type_predicate(graph, predicate):
    def fn(link):
        if link is None:
            return False
        return predicate(graph.edge(*link)['type'])
    return fn


def link_circulation(graph):
    def fn(link):
        if link is None:
            return False
        return not graph.valid_circulation(*link)
    return fn


def any_cycling_link(link_type):
    return link_type > 10


def designated_roadway(link_type):
    return link_type == 11


def bike_lane(link_type):
    return link_type == 13


def seperate_cycling_link(link_type):
    return link_type in {14, 15, 16}


def offroad_link(link_type):
    return link_type == 17


def other_road_type(link_type):
    return link_type in {1, 2, 3, 4, 9}


def arterial_link(link_type):
    return link_type in {6, 7}


def collector_link(link_type):
    return link_type == 5


def highway_link(link_type):
    return link_type == 8


def local_link(link_type):
    return link_type == 0


def left_turn(angle):
    return angle > 45.0


def right_turn(angle):
    return angle < -45.0


def intersection_collection(collection):
    def fn(node):
        return node in collection
    return fn


def link_features(length, start_elevation, end_elevation, link, graph):
    type = graph.edge(*link)['type']
    edge_predicates = [
        lambda link: True,
        any_cycling_link,
        designated_roadway,
        bike_lane,
        seperate_cycling_link,
        offroad_link,
        other_road_type,
        arterial_link,
        collector_link,
        highway_link,
        local_link,
    ]
    slope = 0.0
    if length > 0.0:
      slope = (end_elevation - start_elevation) / length

    return numpy.array(list(map(lambda pred: pred(type), edge_predicates)) + 
                       [not graph.valid_circulation(*link), slope**2, slope**3]) * length


def intersection_features(a, b, graph, collections):
    assert a[1] == b[0]
    u, v = a
    v, k = b
    angle = graph.turn_angle(u, v, k)
    node_predicates = [
        lambda node: True,
        intersection_collection(collections['end_of_facility']),
        intersection_collection(collections['change_of_facility_type']),
        intersection_collection(collections['intersections_disc']),
        intersection_collection(collections['traffic_lights']),
    ]
    turn_predicates = [
        left_turn,
        right_turn
    ]
    return numpy.array(list(map(lambda pred: pred(math.degrees(angle)), turn_predicates)) +
                       list(map(lambda pred: pred(v), node_predicates)))


def extract_features(trajectories, graph):
    end_of_facility = match_intersections(
        load_discontinuity("data/discontinuity/end_of_facility"), graph)
    change_of_facility_type = match_intersections(
        load_discontinuity("data/discontinuity/change_of_facility_type"), graph)
    intersections_disc = match_intersections(
        load_discontinuity("data/intersections/intersections_on_bike_network_with_change_in_road_type"), graph)
    traffic_lights = match_intersections(
        load_traffic_lights("data/traffic_lights/All_lights"), graph)
    elevation = raster.RasterImage("data/elevation/30n090w_20101117_gmted_min075.tif")

    partition = RegionPartition("data/partition/ZT2008_1631_v2b_region")

    features = {}
    for trajectory in trajectories:
        logging.info("extracting features for %s", trajectory['id'])

        i = trajectory['id']
        features[i] = {}
        features[i]['length'] = extract_length(trajectory,
                                               lambda link: True)
        features[i]['length_cycling'] = extract_length(trajectory,
                                                       link_type_predicate(graph, any_cycling_link))
        features[i]['length_designated_roadway'] = extract_length(trajectory,
                                                                  link_type_predicate(graph, designated_roadway))
        features[i]['length_bike_lane'] = extract_length(trajectory,
                                                         link_type_predicate(graph, bike_lane))
        features[i]['length_seperate_cycling_link'] = extract_length(trajectory,
                                                                     link_type_predicate(graph, seperate_cycling_link))
        features[i]['length_offroad'] = extract_length(trajectory,
                                                       link_type_predicate(graph, offroad_link))
        features[i]['length_other_road'] = extract_length(trajectory,
                                                          link_type_predicate(graph, other_road_type))
        features[i]['length_arterial'] = extract_length(trajectory,
                                                        link_type_predicate(graph, arterial_link))
        features[i]['length_collector'] = extract_length(trajectory,
                                                         link_type_predicate(graph, collector_link))
        features[i]['length_highway'] = extract_length(trajectory,
                                                       link_type_predicate(graph, highway_link))
        features[i]['length_local'] = extract_length(trajectory,
                                                     link_type_predicate(graph, local_link))
        features[i]['length_inverse'] = extract_length(trajectory, link_circulation(graph))

        features[i]['left_turn'] = extract_turn(trajectory, graph, left_turn)
        features[i]['right_turn'] = extract_turn(trajectory, graph, right_turn)

        features[i]['intersections'] = extract_intersection(trajectory, lambda link: True)
        features[i]['end_of_facility'] = extract_intersection(trajectory,
                                                              intersection_collection(end_of_facility))
        features[i]['change_of_facility_type'] = extract_intersection(trajectory,
                                                                      intersection_collection(change_of_facility_type))
        features[i]['intersections_disc'] = extract_intersection(trajectory,
                                                                 intersection_collection(intersections_disc))
        features[i]['traffic_lights'] = extract_intersection(trajectory,
                                                             intersection_collection(traffic_lights))

        partition_begin = partition.fit(sg.Point(trajectory['segment'][0].geometry[0]))
        partition_end = partition.fit(sg.Point(trajectory['segment'][-1].geometry[-1]))
        features[i]['partition'] = (partition_begin, partition_end)

        features[i]['duration'] = (trajectory['segment'][-1].end.idx -
                                   trajectory['segment'][0].begin.idx)
        features[i]['avg_speed'] = features[i]['length'] / features[i]['duration']

        elev_M2, elev_M3 = extract_elevation_stats(trajectory, graph, elevation)
        features[i]['elev_m2'] = elev_M2
        features[i]['elev_m3'] = elev_M2

    return features
