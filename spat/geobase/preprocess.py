import sys, json, getopt, pickle
import pyproj as proj
import shapely.geometry as sg

from spat.spatial_graph import SpatialGraph

def preprocess_road(data):
  srcProj = proj.Proj(init='epsg:4326')
  dstProj = proj.Proj(init='epsg:2950')

  for segment in data['features']:
    geometry = []
    for point in segment['geometry']['coordinates']:
      coord = proj.transform(srcProj, dstProj, 
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
      'type': p['TYPE_VOIE'],
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
  outputfile = 'data/mtl_geobase/mtl.pickle'
  distance_threshold = 2.0
  try:
    opts, args = getopt.getopt(argv,"i:o:",["ofile="])
  except getopt.GetoptError:
    print 'preprocess [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'preprocess [-o <outputfile>]'
      sys.exit()
    elif opt in ("-o", "--ofile"):
       outputfile = arg

  print 'output file:', outputfile

  graph = SpatialGraph()
  graph.build_spatial_node_index()
  with open("data/mtl_geobase/road.json", 'r') as f:
    idx = graph.import_geobase(preprocess_road(json.load(f)), 0, 2.0)
  with open("data/mtl_geobase/cycling.json", 'r') as f:
    graph.import_geobase(preprocess_cycling(json.load(f)), idx, 2.0)

  for edge in graph.graph.edges_iter():
    length = sg.LineString(graph.way(edge)).length
    if length > 5000:
      print edge, length
      for p in graph.way(edge):
        print p.x, p.y
      print

  graph.compress()

  """for edge in graph.graph.edges_iter():
    length = sg.LineString(graph.way(edge)).length
    if length > 5000:
      print edge, length"""
  with open(outputfile, 'w+') as f:
    pickle.dump(graph, f)
  print 'done' 

if __name__ == "__main__":
  main(sys.argv[1:])
