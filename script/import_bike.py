import shapefile
import csv


def get_trajectories(data):
	path = []
	previous_id = -1;
	for row in data:
		current_id = row[0]
		if current_id != previous_id:
			if path:
				yield {'geometry': path, 'id': previous_id}
				path = []
			previous_id = current_id
		path.append([float(row[3]), float(row[2])]);

sf = shapefile.Reader("data/intersections/intersections_on_bike_network_with_change_in_road_type")
shapes = sf.shapes()

w = shapefile.Writer(shapefile.POLYLINE)
w.autoBalance = 1
w.field('id')

w2 = shapefile.Writer(shapefile.POINT)
w2.autoBalance = 1
w2.field('id')

csvr = csv.reader(open("data/bike_path/Chunk_1_mm.csv"))
rows = iter(csvr)
print "header: ", rows.next()

for t in get_trajectories(rows):
	w.line([t['geometry']])
	w.record(t['id'])

w.save('data/bike_path/Chunk_1')
prj = open("%s.prj" % 'data/bike_path/Chunk_1', "w") 
epsg = 'GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]]'
prj.close()

csvr = csv.reader(open("data/bike_path/Chunk_1_mm.csv"))
rows = iter(csvr)
print rows.next()
for i in range(1, 100000):
	row = rows.next()
	w2.point(float(row[3]), float(row[2]))
	w2.record(row[0])

w2.save('data/bike_path/_Chunk_1')
prj = open("%s.prj" % 'data/bike_path/_Chunk_1', "w") 
epsg = 'GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]]'
prj.close()