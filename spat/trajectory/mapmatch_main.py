import sys, argparse, logging
import pickle, json, geojson, csv
import shapely.geometry as sg
import numpy
import pyproj

from spat.trajectory import mapmatch, smooth, load, features
from spat import raster, utility


def make_geojson(trajectories, graph):
    features = []
    for trajectory in trajectories:
        segments = trajectory['segment']
        mm = []
        for segment in segments:
            for point in segment.geometry:
                mm.append(point)
        if len(mm) > 1:
            features.append(geojson.Feature(
                geometry = sg.mapping(sg.LineString(mm)),
                properties = {'type':'mm'}))
        for node in trajectory['node']:
            if (isinstance(node, mapmatch.LinkedNode) or isinstance(node, mapmatch.ForwardingNode) or
                isinstance(node, mapmatch.FloatingNode) or isinstance(node, mapmatch.JumpingNode)):
                features.append(geojson.Feature(
                    geometry= sg.mapping(sg.Point(node.coordinates())),
                    properties= {'type': node.__class__.__name__}
                ))

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
    parser.add_argument('-i', '--ifile', default = 'data/bike_path/Chunk_1_mm.csv',
                        help='input pickle file of preprocessed (with spat.trajectory.preprocess) data.')
    parser.add_argument('--facility', default = 'data/mtl_geobase/mtl.pickle',
                        help="""input pickle file containing the facility graph 
    (with spat.geobase.preprocess) representing the road network""")
    parser.add_argument('-o', '--ofile', default = 'data/bike_path/mm_1.pickle',
                        help='output pickle file containing an array of segments')
    parser.add_argument('--geojson',
                        help='output geojson file to export constrained geometry')
    parser.add_argument('--factor', default=15.0, type=float,
                        help='heuristic factor. Higher is more greedy')
    parser.add_argument('--max', type=int, default=None,
                        help='maximum number of trajectory that will be processed')

    args = parser.parse_args()
    print('input file:', args.ifile)
    print('facility:', args.facility)
    print('output file:', args.ofile)

    logging.basicConfig(level=logging.INFO)

    with open(args.facility, 'rb') as f:
        graph = pickle.load(f)
    graph.build_spatial_edge_index()
    graph.build_spatial_node_index()

    link_weights = numpy.array([
        1.0,   0.0,   0.0,    0.0,   0.0,
        0.0,  0.0,   0.0,   0.0,   0.0,
        0.0, 0.0, 0.0, 0.0])
    intersection_weights = numpy.array([
        1.0,   0.0,   0.0,  0.0,
        0.0,  0.0,  0.0])

    intersection_collections = {
        'end_of_facility': features.match_intersections(
            features.load_discontinuity("data/discontinuity/end_of_facility"), graph),
        'change_of_facility_type': features.match_intersections(
            features.load_discontinuity("data/discontinuity/change_of_facility_type"), graph),
        'intersections_disc': features.match_intersections(
            features.load_discontinuity("data/intersections/intersections_on_bike_network_with_change_in_road_type"),
            graph),
        'traffic_lights': features.match_intersections(
            features.load_traffic_lights("data/traffic_lights/All_lights"), graph),
    }

    elevation = raster.RasterImage("data/elevation/30n090w_20101117_gmted_min075.tif")
    dst_proj = pyproj.Proj(init='epsg:4326')

    def distance_cost(length, start, end, link):
        start_elevation = elevation.at(start, dst_proj)
        end_elevation = elevation.at(end, dst_proj)
        cost = numpy.dot(features.link_features(length, start_elevation, end_elevation, link, graph), link_weights)
        if link is None:
            cost += 100.0 * length
        return cost

    def intersection_cost(a, b):
        return numpy.dot(features.intersection_features(a, b, graph, intersection_collections), intersection_weights)

    matched = []
    with open(args.ifile, 'r') as f:
        input_data = csv.reader(f)
        for trajectory in utility.take(load.load_csv(input_data), args.max):
            smoothed_trajectory = smooth.smooth_state(trajectory)
            if smoothed_trajectory is None:
                continue
            matched_trajectory = mapmatch.solve(smoothed_trajectory, graph, distance_cost, intersection_cost, args.factor)
            if matched_trajectory is None:
                continue
            matched.append(matched_trajectory)

    with open(args.ofile, 'wb+') as f:
        pickle.dump(matched, f)
    if args.geojson is not None:
        with open(args.geojson, 'w+') as f:
            json.dump(make_geojson(matched, graph), f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])