import sys
sys.path.append('.')

import facility_graph as fg
import json
import shapefile

def load_geobase_road():
  sf = shapefile.Reader("data/mtl_geobase/GEOBASE_MTL")
  data = []
  id_field = next(i for i,v in enumerate(sf.fields) if v[0] == "ID_TRC")-1
  for s, r in zip(sf.shapes(), sf.records()):
    print s.points
    data.append({'geometry': sg.LineString(s.points), 'properties': {'oid': r[id_field]}})
  return data

def load_geobase_cycling():
  sf = shapefile.Reader("data/mtl_geobase/reseaucyclable2016sept2016")
  data = []
  id_field = next(i for i,v in enumerate(sf.fields) if v[0] == "ID")-1
  for s, r in zip(sf.shapes(), sf.records()):
    data.append({'geometry': sg.LineString(s.points), 'properties': {'oid': int(r[id_field])}})
  return data

def create_graph():
  n = fg.FacilityGraph()
  n.import_geobase(load_geobase_road(), 5.0)
  n.import_geobase(load_geobase_cycling(), 5.0)
  with open('data/facilities/montreal.pickle', 'w+') as f:
    pickle.dump(n, f)
  return n

#n = create_graph()
#with open('data/facilities/montreal.pickle', 'r') as f:
#  n = pickle.load(f)

#g = n.graph.copy()
#compress(g)

load_geobase_road()

#with open('data/facilities/mtl_geobase.json', 'w+') as f:
#  json.dump(make_geojson(g), f, indent=2)

#with open('data/facilities/montreal_index.json', 'w+') as f:
#  json.dump(make_index(g), f, indent=2)

#make_shp(g).save('data/facilities/montreal_geo')
print "done"
