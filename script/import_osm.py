import xml.etree.ElementTree as et
import sets
import itertools
import json
import pickle
import sys, getopt

def peek(iterable):
  try:
    first = next(iterable)
  except StopIteration:
    return None
  return first, itertools.chain([first], iterable)

def empty(iterable):
  return peek(iterable) is None

def load_osm(tree):
  root = tree.getroot()
  data = {'nodes':{}, 'ways':{}}
  used_nodes = sets.Set()
  for e in root:
    nodes = e.findall('nd')
    tags = e.findall('tag')
    if 'id' not in e.attrib:
      continue
    id = int(e.attrib['id'])
    if e.tag == 'node':
      coord = [float(e.attrib['lon']), float(e.attrib['lat'])]
      data['nodes'][id] = {'geometry':coord}

    if e.tag == 'way':
      highway = (x for x in tags if x.attrib['k'] == 'highway')
      if not empty(highway):
        data['ways'][id] = {'nodes':[], 'type':'way', 'tags':{}}
        for i in nodes:
          n = int(i.attrib['ref'])
          data['ways'][id]['nodes'].append(n)
          used_nodes.add(n)
        for p in tags:
          keys = ['highway', 'cycleway', 'oneway', 'foot', 'bicycle', 'lanes', 'lit', 'width', 'maxspeed', 'name']
          for k in keys:
            if p.attrib['k'] == k:
              data['ways'][id]['tags'][k] = p.attrib['v']
  unused_nodes = list((n for n in data['nodes'] if n not in used_nodes))
  for n in unused_nodes:
    del data['nodes'][n]
  return data

def main(argv):
  opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
  for opt, arg in opts:
    if opt in ("-i", "--ifile"):
       inputfile = arg
    elif opt in ("-o", "--ofile"):
       outputfile = arg
  tree = et.parse(inputfile)
  data = load_osm(tree)
  with open(outputfile, 'w+') as f:
    pickle.dump(data, f)

if __name__ == "__main__":
  main(sys.argv[1:])

