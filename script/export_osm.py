import facility_graph as fg
import pickle
import sys, getopt
import json

def main(argv):
  opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
  for opt, arg in opts:
    if opt in ("-i", "--ifile"):
       inputfile = arg
    elif opt in ("-o", "--ofile"):
       outputfile = arg
  with open(inputfile, 'r') as f:
    data = pickle.load(f)

  g = fg.FacilityGraph()
  g.import_osm(data)
  g.compress()
  #with open(outputfile, 'w+') as f:
  #  json.dump(g.export_geojson(4326), f)
  g.export_shp().save(outputfile)

if __name__ == "__main__":
  main(sys.argv[1:])
