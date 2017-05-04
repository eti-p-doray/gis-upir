import sys, argparse
import pickle, json, geojson
import shapely.geometry as sg

from spat.trajectory import mapmatch


def make_geojson(trajectories, graph):
    features = []
    for path, states in trajectories:
        mm = []
        for node in path:
            if isinstance(node, mapmatch.LinkedNode) or isinstance(node, mapmatch.FloatingNode):
                mm.append(states[node][0].coordinates())
        if len(mm) > 1:
            features.append(geojson.Feature(
                geometry = sg.mapping(sg.LineString(mm)),
                properties = {'type':'mm'}))

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
    parser.add_argument('-i', '--ifile', default = 'data/bike_path/smoothed.pickle',
                        help='input pickle file of preprocessed (with spat.trajectory.preprocess) data.')
    parser.add_argument('--facility', default = 'data/mtl_geobase/mtl.pickle',
                        help="""input pickle file containing the facility graph 
    (with spat.geobase.preprocess) representing the road network""")
    parser.add_argument('-o', '--ofile', default = 'data/bike_path/mm.pickle',
                        help='output pickle file containing an array of segments')
    parser.add_argument('--geojson',
                        help='output geojson file to export constrained geometry')
    parser.add_argument('--factor', default=150.0, type=float,
                        help='heuristic factor. Higher is more greedy')

    args = parser.parse_args()
    print('input file:', args.ifile)
    print('facility:', args.facility)
    print('output file:', args.ofile)

    with open(args.facility, 'rb') as f:
        graph = pickle.load(f)
    graph.build_spatial_edge_index()
    graph.build_spatial_node_index()

    with open(args.ifile, 'rb') as f:
        preprocessed = pickle.load(f)

    matched = list(mapmatch.solve(preprocessed, graph, args.factor))

    with open(args.ofile, 'wb+') as f:
        pickle.dump(matched, f)
    if args.geojson is not None:
        with open(args.geojson, 'w+') as f:
            json.dump(make_geojson(matched, graph), f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])