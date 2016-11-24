from spat.spatial_graph import SpatialGraph
from spat.path_inference import infer_path

def map_match(trajectories, graph):
  for trajectory in trajectories:
    trajectory['edges'] = list(infer_path(trajectory['state'], graph))
    yield trajectory

def make_geojson(trajectories, graph):
  features = []
  for i, trajectory in enumerate(trajectories):
    mm = []
    for u,v in trajectory['edges']:
      if graph.has_edge(u, v):
        mm.extend(graph.way((u,v)))
    features.append(geojson.Feature(
      geometry = sg.mapping(sg.LineString(mm)), 
      properties = {'id':i, 'oid':trajectory['id'], 'type':'mm'}))

  fc = geojson.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc

def main(argv):
  inputfile = 'data/bike_path/smoothed.pickle'
  facilityfile = 'data/bike_path/mtl.pickle'
  outputfile = 'data/bike_path/mm.json'
  try:
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile=","facility="])
  except getopt.GetoptError:
    print 'mapmatch [-i <inputfile>] [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'mapmatch [-i <filteredfile>] [-o <classfile>]'
      sys.exit()
    if opt in ("-i", "--ifile"):
       inputfile = arg
    if opt in ("-i", "--facility"):
       facilityfile = arg
    elif opt in ("-o", "--ofile"):
       outputfile = arg

  print 'input file:', inputfile
  print 'facility:', facilityfile
  print 'output file:', outputfile

  with open(facilityfile, 'r') as f:
    facility = pickle.load(f)
  graph = SpatialGraph()
  graph.import_osm(facility)

  with open(inputfile, 'r') as f:
    data = pickle.load(f)

  data = map_match(data, graph)

  with open(outputfile, 'w+') as f:
    json.dump(make_geojson(data), f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])
