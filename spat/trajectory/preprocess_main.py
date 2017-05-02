import sys, os, fnmatch, argparse, math
import pickle, geojson, json
import shapely.geometry as sg

import spat.trajectory.smooth as smooth
import spat.trajectory.load as load


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
    parser.add_argument('-i', '--idir', default = 'data/bike_path',
                        help="""directory containing input data files. 
            All *.csv files will be imported""");
    parser.add_argument('-o', '--ofile', default = 'data/bike_path/smoothed.pickle',
                        help='output pickle file to export serialized result');
    parser.add_argument('--geojson',
                        help='output geojson file to export smoothed geometry');
    parser.add_argument('--max', default = math.inf, type=int,
                        help='maximum number of trajectory that will be processed');

    args = parser.parse_args()
    print('input directory:', args.idir)
    print('found input files:')
    for file in os.listdir(args.idir):
        if fnmatch.fnmatch(file, '*.csv'):
            print(' ', args.idir + file)
    print()

    print('output file:', args.ofile)

    files = (os.path.join(args.idir, file)
             for file in os.listdir(args.idir)
             if fnmatch.fnmatch(file, '*.csv'))

    trajectories = load.load_all(files, args.max)
    smoothed_trajectories = list(smooth.smooth_state(trajectories))

    with open(args.ofile, 'w+') as f:
        pickle.dump(smoothed_trajectories, f)
    if args.geojson:
        with open(args.geojson, 'w+') as f:
            json.dump(make_geojson(smoothed_trajectories), f, indent=2)
    print('done')

if __name__ == "__main__":
    main(sys.argv[1:])
