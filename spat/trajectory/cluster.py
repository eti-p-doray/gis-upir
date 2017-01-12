import pickle, geojson, json, math
import shapely.geometry as sg
import sys, getopt

from point_of_interest import *
from spat.utility import *

def make_geojson(clusters):
  features = []

  for i in clusters['center']:
    coord, label = clusters['coord'][i], clusters['label'][i]
  #for coord, label in zip(clusters['coord'], clusters['label']):
    features.append(geojson.Feature(
      geometry = sg.mapping(sg.Point(coord)), 
      properties = {'id': i, 'type':'poi', 'label':label}))

  fc = geojson.FeatureCollection(features)
  fc['crs'] = {'type': 'EPSG', 'properties': {'code': 2150}}
  return fc

class handler:
  def __init__(self, clusterfile):
    with open(clusterfile, 'r') as f:
      self.cluster = data = pickle.load(f)

  def do_GET(self, request):
    print request.path
    if request.path == 'center.json':
      return json.dumps(make_geojson(self.cluster))
    elif request.path.startswith('selection'):
      print request.data
      if request.data:
        return json.dumps(self.intersection(request.data['labels']))
      return json.dumps([])

  def intersection(self, selection):
    print selection
    if selection:
      return list(set.intersection(*[self.cluster['trajectories'][int(i)] for i in selection]))
    return []

def main(argv):
  inputfile = 'data/bike_path/smoothed.pickle'
  outputfile = 'data/bike_path/cluster.pickle'
  try:
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
  except getopt.GetoptError:
    print 'cluster [-i <inputfile>] [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'cluster [-i <inputfile>] [-o <outputfile>]'
      sys.exit()
    if opt in ("-i", "--ifile"):
       inputfile = arg
    elif opt in ("-o", "--ofile"):
       outputfile = arg

  print 'input file:', inputfile
  print 'output file:', outputfile

  with open(inputfile, 'r') as f:
    data = pickle.load(f)

  cluster = cluster_trajectories(data)

  with open(outputfile, 'w+') as f:
    pickle.dump(cluster, f)
  print 'done' 

if __name__ == "__main__":
  main(sys.argv[1:])
