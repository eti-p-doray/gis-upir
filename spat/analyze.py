


def infer_paths(trajectories, graph):
  for t in trajectories:
    t['path'] = list(infer_path(t['state'], graph))
    yield t



def main(argv):
  #studentsfile = 'studentsByAvailability.json'
  #classfile = 'classes.json'
  #outputfile = 'schedule.json'
  #tidy = False
  smooth = False
  match = False
  try:
    opts, args = getopt.getopt(argv,"hf::",["smooth","match","cluster"])
  except getopt.GetoptError:
    print 'make_schedule [--filtered=<filteredfile>] [-c <classfile>] [-s <studentsfile>] [-o <outputfile>]'
    sys.exit(2)
  for opt, arg in opts:
    if opt == '-h':
      print 'make_schedule [--tidy] [-c <classfile>] [-s <studentsfile>] [-o <outputfile>]'
      sys.exit()
    elif opt in ("-s", "--students"):
      studentsfile = arg
    elif opt in ("-c", "--classes"):
      classfile = arg
    elif opt in ("-o", "--ofile"):
      outputfile = arg
    elif opt == "--tidy":
      tidy = True

  F = np.identity(4)
  F[0][2] = 1.0
  F[1][3] = 1.0
  Q = np.diag([0.2, 0.2, 0.5, 0.5])

  with open(studentsfile) as data_file:
    students = load_students(json.loads(data_file.read()))
  with open(classfile) as data_file:
    classes = load_classes(json.loads(data_file.read()))
  students, classes = assign_schedules(students, classes)
  print_score(students, classes)
  if tidy:
    result = export_tidy_schedule(students, classes)
  else:
    result = export_schedule(students, classes)
  with open (outputfile, 'w+') as data_file:
    json.dump(result, data_file, indent = 2)

csvr = csv.reader(open('data/bike_path/Chunk_1_mm.csv'))
traj = import_trajectories(csvr)
filtered_traj = estimate_state(traj, F, Q)

with open('data/bike_path/filtered.pickle', 'w+') as f:
  pickle.dump(list(x for i,x in zip(range(200),filtered_traj)), f)

#trajectories, points_of_interest = cluster_trajectories(filtered_traj, F, Q)

#with open('data/osm/jean-drapeau.pickle', 'r') as f:
#  mtl = pickle.load(f)
#graph = SpatialGraph()
#graph.import_osm(mtl)
#graph.compress()
#graph.build_spatial_edge_index()
#mapped_traj = infer_paths(filtered_traj, graph)
#pois = extract_poi(filtered_traj)
#for p in pois:
#  print p[0]

#with open('data/bike_path/clustered.json', 'w+') as f:
#  json.dump(make_geojson(trajectories, points_of_interest), f, indent=2)
