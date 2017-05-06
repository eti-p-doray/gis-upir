import sys, argparse, fnmatch, logging
import pickle, csv, json

from spat.trajectory import features


def main(argv):
    parser = argparse.ArgumentParser(description="""
    Extract several features from mapmatched bike trajectories.
    Features are:
    length
    length_cycling
    length_designated_roadway
    length_bike_lane
    length_seperate_cycling_link
    length_offroad

    end_of_facility
    change_of_facility_type
    intersections_disc
    """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--ifile', default = 'data/bike_path/mm.pickle',
                        help='input pickle file of mapmatched (with spat.trajectory.mapmatch) data.')
    parser.add_argument('--facility', default = 'data/mtl_geobase/mtl.pickle',
                        help="""input pickle file containing the facility graph 
    (with spat.geobase.preprocess) representing the road network""")
    parser.add_argument('-o', '--ofile',
                        default = ['data/bike_path/features.json'], nargs='+',
                        help="""output file containing an array of segments. 
    Supported formats include *.json, *.csv""")

    args = parser.parse_args()
    print('input file:', args.ifile)
    print('facility:', args.facility)
    print('output file:', args.ofile)

    logging.basicConfig(level=logging.DEBUG)

    with open(args.facility, 'rb') as f:
        graph = pickle.load(f)
    graph.build_spatial_node_index()

    with open(args.ifile, 'rb') as f:
        data = pickle.load(f)

    observed_features = features.extract_features(data, graph)

    for output in args.ofile:
        with open(output, 'w+') as f:
            if fnmatch.fnmatch(output, '*.csv'):
                writer = csv.writer(f)
                for row in observed_features:
                    print(row.values())
                    writer.writerow(row.values())
            elif fnmatch.fnmatch(output, '*.json'):
                json.dump(observed_features, f, indent=2)

if __name__ == "__main__":
    main(sys.argv[1:])
