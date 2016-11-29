import pickle, geojson, json, math
import shapely.geometry as sg
import sys, getopt

from spat.spatial_graph import SpatialGraph
from spat.kalman import KalmanFilter
from spat.utility import *

def make_geojson(trajectories):
  features = []
  for idx, trajectory in enumerate(trajectories):
    state = [[s.x[0], s.x[1]] for s in trajectory['state']]
    features.append(geojson.Feature(
      geometry = sg.mapping(sg.LineString(state)), 
      properties = {'id':idx, 'oid':trajectory['id'], 'type':'state'}))

    """observations = [o for o in trajectory['observations']]
    features.append(geojson.Feature(
      geometry = sg.mapping(sg.LineString(observations)), 
      properties = {'id':idx, 'oid':trajectory['id'], 'type':'raw'}))"""

  fc = geojson.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc

def main(argv):
  inputfile = 'data/bike_path/smoothed.pickle'
  outputfile = 'data/bike_path/smoothed.json'
  try:
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
  except getopt.GetoptError:
    print 'export [-i <inputfile>] [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'export [-i <inputfile>] [-o <outputfile>]'
      sys.exit()
    if opt in ("-i", "--ifile"):
       inputfile = arg
    elif opt in ("-o", "--ofile"):
       outputfile = arg

  print 'input file:', inputfile
  print 'output file:', outputfile

  with open(inputfile, 'r') as f:
    data = pickle.load(f)
  with open(outputfile, 'w+') as f:
    json.dump(make_geojson(data), f, indent=2)

if __name__ == "__main__":
  main(sys.argv[1:])
