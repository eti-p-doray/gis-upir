import shapefile
import csv

sf = shapefile.Reader("data/intersections/intersections_on_bike_network_with_change_in_road_type")
shapes = sf.shapes()

for i in shapes:
	i
	#print(i.points);


w = shapefile.Writer(shapefile.POLYLINE)
w.autoBalance = 1
w.field('id')

spamreader = csv.reader(open("data/bike_path/Chunk_1_mm.csv"))
rows = iter(spamreader)
print "header: ", rows.next()

current_id = -1;
path = []
#for i in range(0,100000):
#	row = rows.next()
for row in rows:
	id = row[0];
	if (id != current_id):
		if path:
			w.line(parts=[path])
			#print path
		path = []
		current_id = id
	path.append([float(row[3]), float(row[2])])
	w.record(row[0])

w.save('data/bike_path/Chunk_1')
prj = open("%s.prj" % 'data/bike_path/Chunk_1', "w") 
epsg = 'GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]]'
prj.close()