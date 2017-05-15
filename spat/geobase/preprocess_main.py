import sys, argparse
import pickle, json
import pyproj
import shapely.geometry as sg

from spat import facility


def preprocess_road(data):
    src_proj = pyproj.Proj(init='epsg:4326')
    dst_proj = pyproj.Proj(init='epsg:2950')

    for segment in data['features']:
        geometry = []
        for point in segment['geometry']['coordinates']:
            coord = pyproj.transform(src_proj, dst_proj,
                                   float(point[0]),
                                   float(point[1]))
            geometry.append(coord)
        properties = segment['properties']

        yield {
            'geometry': sg.LineString(geometry),
            'properties': {
                'way_id': properties['ID_TRC'],
                'type': properties['CLASSE'],
                'sens': properties['SENS_CIR'],
            }
        }


def preprocess_cycling(data):
    for segment in data['features']:
        geometry = sg.asShape(segment['geometry'])
        p = segment['properties']
        properties = {
            'way_id': p['ID'],
            'link_id': p['ID_TRC_GEOBASE'],
            'type': p['TYPE_VOIE'] + 10,
            'sens': 0,
        }
        if (geometry.type == 'LineString'):
            yield {
                'geometry': sg.asShape(segment['geometry']),
                'properties': properties
            }
        elif (geometry.type == 'MultiLineString'):
            for line in geometry:
                yield {
                    'geometry': line,
                    'properties': properties
                }


def main(argv):
    parser = argparse.ArgumentParser(description="""
      Preprocess montreal geobase into a facility graph.
      """, formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('-o', '--ofile',
                        default='data/mtl_geobase/mtl.pickle',
                        help="""output file containing a serialization of the graph. 
      Supported formats include *.json, *.csv""")
    parser.add_argument('--geojson',
                        help='output geojson file to export constrained geometry')

    args = parser.parse_args()
    print('output file:', args.ofile)

    distance_threshold = 6.0

    graph = facility.SpatialGraph()
    graph.build_spatial_node_index()
    with open("data/mtl_geobase/road.json", 'r') as f:
        graph.import_geobase(preprocess_road(json.load(f)), distance_threshold)
    with open("data/mtl_geobase/cycling.json", 'r') as f:
        graph.import_geobase(preprocess_cycling(json.load(f)), distance_threshold)

    with open(args.ofile, 'wb+') as f:
        pickle.dump(graph, f)

    if args.geojson is not None:
        with open(args.geojson, 'w+') as f:
            json.dump(graph.make_geojson(2150), f, indent=2)
    print('done')

if __name__ == "__main__":
    main(sys.argv[1:])
