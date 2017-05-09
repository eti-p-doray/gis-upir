import sys, argparse, logging
import pickle, json, geojson
import shapely.geometry as sg
import numpy

from spat.trajectory import ioc, features


def make_geojson(trajectories, graph):
    feature = []
    for trajectory in trajectories:
        bp = []
        for node in trajectory['ioc']:
            if isinstance(node, ioc.Node):
                bp.extend(graph.edge_geometry(*node.edge))
        if len(bp) > 1:
            feature.append(geojson.Feature(
                geometry = sg.mapping(sg.LineString(bp)),
                properties = {'id': trajectory['id'],
                              #'feature': trajectory['feature'],
                              'type': 'ioc'}))

    fc = geojson.FeatureCollection(feature)
    fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
    return fc


def main(argv):
    parser = argparse.ArgumentParser(description="""
    Learns weiths associated with a vector of features.
    """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--features', default = 'data/bike_path/features.json',
                        help='input pickle file of preprocessed (with spat.trajectory.preprocess) data.')
    parser.add_argument('--mapmatch', default = 'data/bike_path/mm.pickle',
                        help='input pickle file of preprocessed (with spat.trajectory.preprocess) data.')
    parser.add_argument('--facility', default = 'data/mtl_geobase/mtl.pickle',
                        help="""input pickle file containing the facility graph 
    (with spat.geobase.preprocess) representing the road network""")

    args = parser.parse_args()
    print('features:', args.features)
    print('mapmatch:', args.mapmatch)
    print('facility:', args.facility)

    logging.basicConfig(level=logging.INFO)

    with open(args.facility, 'rb') as f:
        graph = pickle.load(f)
    graph.build_spatial_node_index()
    graph.build_spatial_edge_index()

    with open(args.mapmatch, 'rb') as f:
        mm = pickle.load(f)

    with open(args.features, 'r') as f:
        feature_table = json.load(f)

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

    """weights = numpy.ones(18)
    for i, trajectory in enumerate(mm):
        path, feature = ioc.best_path(weights, trajectory, graph, intersection_collections)
        mm[i]['ioc'] = path

    with open("data/bike_path/ioc.json", 'w+') as f:
        json.dump(make_geojson(mm, graph), f, indent=2)"""

    data = []
    for trajectory in mm:
        if trajectory['id'] not in feature_table:
            continue
        feature = feature_table[trajectory['id']]
        example = (numpy.array([
            feature['length'],
            feature['length_cycling'],
            feature['length_designated_roadway'],
            feature['length_bike_lane'],
            feature['length_seperate_cycling_link'],
            feature['length_offroad'],
            feature['length_other_road'],
            feature['length_arterial'],
            feature['length_collector'],
            feature['length_highway'],
            feature['length_local'],
            feature['left_turn'],
            feature['right_turn'],
            feature['intersections'],
            feature['end_of_facility'],
            feature['change_of_facility_type'],
            feature['intersections_disc'],
            feature['traffic_lights'],
        ]), trajectory)
        data.append(example)

    weights = numpy.array([-1.44504282,  1.01295696,  1.92465963,  1.67661907,  1.30585154,  3.80870991,
                            5.10398271,  1.70687101,  1.90445324,  2.01541104,  2.08286045,  1.71281308,
                            1.26563477, -0.56916418,  4.29999485,  0.53619568, -0.88838111,  1.56403229])

    weights = ioc.inverse_optimal_control(
        data, 1,
        lambda param, examples: ioc.estimate_gradient(param, examples, graph, intersection_collections), weights,
        0.01, 0.1, 200)


if __name__ == "__main__":
    main(sys.argv[1:])