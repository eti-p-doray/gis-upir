import shapefile
import csv

def read_trajectories(data):
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

csvr = csv.reader(open("data/bike_path/Chunk_1_mm.csv"))
rows = iter(csvr)
print "header: ", rows.next()
print rows.next()