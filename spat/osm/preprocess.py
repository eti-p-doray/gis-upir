import json, pickle, xml.etree.ElementTree as et, pyproj as proj
import sets
import itertools
import sys, getopt

from spat.utility import *

def load_osm(tree):
  root = tree.getroot()
  data = {'nodes':{}, 'ways':{}}
  used_nodes = sets.Set()
  srcProj = proj.Proj(init='epsg:4326')
  dstProj = proj.Proj(init='epsg:2145')
  for e in root:
    nodes = e.findall('nd')
    tags = e.findall('tag')
    if 'id' not in e.attrib:
      continue
    id = int(e.attrib['id'])
    if e.tag == 'node':
      coord = proj.transform(srcProj, dstProj, float(e.attrib['lon']), float(e.attrib['lat']))
      data['nodes'][id] = {'geometry':coord}

    if e.tag == 'way':
      highway = peek(x for x in tags if x.attrib['k'] == 'highway')
      if highway != None:
        data['ways'][id] = {'nodes':[], 'type':'way', 'tags':{}}
        for i in nodes:
          n = int(i.attrib['ref'])
          data['ways'][id]['nodes'].append(n)
          used_nodes.add(n)
        for p in tags:
          keys = ['highway', 'cycleway', 'oneway', 
            'foot', 'bicycle', 'lanes', 'lit', 
            'width', 'maxspeed', 'name']
          for k in keys:
            if p.attrib['k'] == k:
              data['ways'][id]['tags'][k] = p.attrib['v']
  unused_nodes = list((n for n in data['nodes'] if n not in used_nodes))
  for n in unused_nodes:
    del data['nodes'][n]
  return data

def main(argv):
  inputfile = ''
  outputfile = ''
  try:
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
  except getopt.GetoptError:
    print 'preprocess [-i <inputfile>] [-o <outputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'preprocess [-i <filteredfile>] [-o <classfile>]'
      sys.exit()
    if opt in ("-i", "--ifile"):
       inputfile = arg
    elif opt in ("-o", "--ofile"):
       outputfile = arg
  if inputfile == '' or outputfile == '':
    print 'preprocess [-i <inputfile>] [-o <outputfile>]'
    sys.exit()

  print 'input file:', inputfile
  print 'output file:', outputfile 

  tree = et.parse(inputfile)
  data = load_osm(tree)
  with open(outputfile, 'w+') as f:
    pickle.dump(data, f)

if __name__ == "__main__":
  main(sys.argv[1:])

