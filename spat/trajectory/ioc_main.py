import sys, argparse, logging
import pickle, json, geojson
import shapely.geometry as sg
import numpy
import pyproj

from spat.trajectory import ioc, features
from spat import raster


def make_geojson(trajectories, graph):
    feature = []
    for trajectory in trajectories:
        bp = []
        for node in trajectory['ioc']:
            if isinstance(node, ioc.Node):
                bp.extend(graph.edge_geometry(node.edge).coords)
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
    parser.add_argument('--features', default = ['data/bike_path/features.json'], nargs='*',
                        help='input pickle file of preprocessed (with spat.trajectory.preprocess) data.')
    parser.add_argument('--mapmatch', default = 'data/bike_path/mm.pickle', nargs='*',
                        help='input pickle file of preprocessed (with spat.trajectory.preprocess) data.')
    parser.add_argument('--facility', default = 'data/mtl_geobase/mtl.pickle',
                        help="""input pickle file containing the facility graph 
    (with spat.geobase.preprocess) representing the road network""")

    args = parser.parse_args()
    print('features:', args.features)
    print('mapmatch:', args.mapmatch)
    print('facility:', args.facility)

    logging.basicConfig(level=logging.INFO)

    mm = []
    for filename in args.mapmatch:
        with open(filename, 'rb') as f:
            mm.extend(pickle.load(f))

    feature_dict = {}
    for filename in args.features:
        with open(filename, 'r') as f:
            feature_dict.update(json.load(f))

    elevation = raster.RasterImage("data/elevation/30n090w_20101117_gmted_min075.tif")
    dst_proj = pyproj.Proj(init='epsg:4326')

    observation = []
    for trajectory in mm:
        if trajectory['id'] not in feature_dict:
            continue
        feature = feature_dict[trajectory['id']]
        example = numpy.array([
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
              feature['length_inverse'],
              feature['elev_m2'],
              feature['elev_m3'],
              feature['left_turn'],
              feature['right_turn'],
              feature['intersections'],
              feature['end_of_facility'],
              feature['change_of_facility_type'],
              feature['intersections_disc'],
              feature['traffic_lights'],
          ])
        observation.append(example)

    #print(observation)
    observation = numpy.array(observation).T
    covariance = numpy.cov(observation)
    #print(observation.shape)
    #print(covariance)
    eigen_values, eigen_vectors = numpy.linalg.eig(covariance)
  

    logging.info("eigen_values: %s", str(eigen_values))
    #logging.info("eigen_vectors: %s", str(eigen_vectors))

    eigen_values = eigen_values[0:19]
    eigen_vectors = eigen_vectors[:,0:19]

    observation = numpy.dot(eigen_vectors.T, observation)

    examples = []
    for i, trajectory in enumerate(mm):
        if trajectory['id'] not in feature_dict:
            continue
        examples.append((observation[:,i], trajectory))
        #print(observation[:, i])

    with open(args.facility, 'rb') as f:
        graph = pickle.load(f)
    graph.build_spatial_node_index()
    graph.build_spatial_edge_index()

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

    params = numpy.dot(numpy.ones(21), eigen_vectors)
    print(params)

    params = ioc.inverse_optimal_control(
        examples,
        lambda param, examples: ioc.estimate_gradient(param, eigen_values, eigen_vectors, examples, graph,
                                                      intersection_collections, elevation, dst_proj), params,
        0.01, 0.1, 10)

    logging.info("params: %s", str(params))

    """score = numpy.zeros(weights.shape)
    for i, trajectory in enumerate(mm):
        path, feature = ioc.best_path(weights, trajectory, graph, intersection_collections, elevation, dst_proj)
        logging.info("feature: %s", str(feature))
        score += numpy.dot(weights, feature)

    weights = (weights / score) * len(mm)
    print(weights)"""


if __name__ == "__main__":
    main(sys.argv[1:])