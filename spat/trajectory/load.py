import datetime
import csv
import pyproj

def load_csv(data):
    rows = iter(data)
    header = next(rows)
    latitude_idx = next(i for i,v in enumerate(header) if v == "latitude")
    longitude_idx = next(i for i,v in enumerate(header) if v == "longitude")
    speed_idx = next(i for i,v in enumerate(header) if v == "speed")
    altitude_idx = next(i for i,v in enumerate(header) if v == "altitude")
    time_idx = next(i for i,v in enumerate(header) if v == "recorded_at")
    hort_acc_idx = next(i for i,v in enumerate(header) if v == "hort_accuracy")
    vert_acc_idx = next(i for i,v in enumerate(header) if v == "vert_accuracy")
    src_node_idx = next(i for i,v in enumerate(header) if v == "src")
    dst_node_idx = next(i for i,v in enumerate(header) if v == "dst")

    src_proj = pyproj.Proj(init='epsg:4326')
    dst_proj = pyproj.Proj(init='epsg:2950')

    observations = []
    accuracy = []
    link = []
    previous_id = -1
    for row in data:
        current_id = row[0]
        if current_id != previous_id:
            if observations:
                print(previous_id)
                yield {
                    'observations': observations,
                    'accuracy': accuracy,
                    'id': previous_id,
                    'link': link
                }
                observations = []
                accuracy = []
                link = []
            previous_id = current_id
            previous_time = None

        current_time = datetime.datetime.strptime(row[time_idx], '%Y-%m-%d %H:%M:%S')
        while (previous_time != None and
                           previous_time + datetime.timedelta(seconds=1) < current_time):
            observations.append(None)
            accuracy.append(None)
            link.append(None)
            previous_time += datetime.timedelta(seconds=1)

        previous_time = current_time
        try:
            coord = pyproj.transform(src_proj, dst_proj,
                                     float(row[longitude_idx]),
                                     float(row[latitude_idx]))
        except RuntimeError:
            previous_id = -1
            continue
        obs = [coord[0], coord[1], float(row[speed_idx])]
        quantile = 1.96
        acc = [float(row[hort_acc_idx])/quantile, float(row[vert_acc_idx])/quantile]
        observations.append(obs)
        accuracy.append(acc)
        link.append((int(row[src_node_idx]), int(row[dst_node_idx])))


def load_all(files, max_count):
    for filepath in files:
        with open(filepath) as csvfile:
            data = csv.reader(csvfile)
            for trajectory in load_csv(data):
                yield trajectory
                max_count -= 1
                if max_count == 0:
                    return