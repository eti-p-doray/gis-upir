import sys, os, fnmatch, argparse, logging
import pickle, geojson, json, csv
import math
import shapely.geometry as sg

from spat.trajectory import smooth, load
from spat import utility


def make_geojson(trajectories):
    features = []
    for trajectory in trajectories:

        state = [[s.x[0], s.x[1]] for s in trajectory['state']]
        features.append(geojson.Feature(
            geometry = sg.mapping(sg.LineString(state)),
            properties = {
                'id':trajectory['id'],
                'type':'state'}))

    fc = geojson.FeatureCollection(features)
    fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
    return fc


def main(argv):
    parser = argparse.ArgumentParser(description="""
    Read bike trajectory in csv format and apply a preprocess step
    that includes optimal smoothing with an autoregressive model,
    as well as filtering out broken trajectories.
    """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--ifile', default = 'data/bike_path/Chunk_1_mm.csv',
                        help='input data file.')
    parser.add_argument('-o', '--ofile', default = 'data/bike_path/smoothed.pickle',
                        help='output pickle file to export serialized result')
    parser.add_argument('--geojson',
                        help='output geojson file to export smoothed geometry')
    parser.add_argument('--max', type=int, default=None,
                        help='maximum number of trajectory that will be processed')

    args = parser.parse_args()
    print('input file:', args.ifile)
    print('output file:', args.ofile)

    logging.basicConfig(level=logging.DEBUG)

    smoothed_trajectories = []
    with open(args.ifile, 'r') as f:
      input_data = csv.reader(f)
      for trajectory in utility.take(load.load_csv(input_data), args.max):
        smoothed_trajectory = smooth.smooth_state(trajectory)
        if smoothed_trajectory is not None:
          smoothed_trajectories.append(smoothed_trajectory)


    with open(args.ofile, 'wb+') as f:
        pickle.dump(smoothed_trajectories, f)
    if args.geojson:
        with open(args.geojson, 'w+') as f:
            json.dump(make_geojson(smoothed_trajectories), f, indent=2)
    print('done')

if __name__ == "__main__":
    main(sys.argv[1:])
