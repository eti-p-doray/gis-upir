import pickle, json
import sys, getopt, os

from spat.spatial_graph import SpatialGraph

def main(argv):
  inputfile = ''
  outputfile = ''
  try:
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
  except getopt.GetoptError:
    print 'export [-i <inputfile>] [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'export [-i <filteredfile>] [-o <classfile>]'
      sys.exit()
    if opt in ("-i", "--ifile"):
       inputfile = arg
    elif opt in ("-o", "--ofile"):
       outputfile = arg
  if inputfile == '' or outputfile == '':
    print 'export [-i <inputfile>] [-o <outputfile>]'
    sys.exit()

  print 'input file:', inputfile
  print 'output file:', outputfile

  with open(inputfile, 'r') as f:
    data = pickle.load(f)
  g = SpatialGraph()
  g.import_osm(data)
  g.compress()
  with open(outputfile, 'w+') as f:
    name, ext = os.path.splitext(outputfile)
    if ext == '.json':
      json.dump(g.make_geojson(2150), f, indent=2)
    elif ext == '.shp':
      g.make_shp().save(name)

if __name__ == "__main__":
  main(sys.argv[1:])
