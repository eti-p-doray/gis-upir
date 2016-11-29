import socket, sys, getopt, threading, json
from jquery_unparam import jquery_unparam
sys.path.append(".")

import spat.trajectory.cluster

class async_task:
  def __init__(self, f, stop):
    self.f = f
    self.stop = stop

  def __call__(self):
    while self.stop.is_set() == False:
      self.f()

class http_request:
  def __init__(self, text):
    if text:
      request_line = text.splitlines()[0].rstrip('\r\n')
      (self.method,
       self.path,
       self.version) = request_line.split()
      data_idx = self.path.find('?')
      self.data = []
      if data_idx != -1:
        self.data = jquery_unparam(self.path[data_idx+1:])
      self.ok = True
    else:
      self.ok = False

class http_server:
  def __init__(self, host, port, handler):
    self.hostname = host
    self.port = port
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #self.sock.setblocking(False)
    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.sock.bind((host, port))
    self.sock.listen(5)
    self.handler = handler

  def serve(self):
    connection, address = self.sock.accept()
    request = http_request(connection.recv(1024))
    if request.ok:
      connection.sendall(self.handler(request))
    connection.close()

  def serve_forever(self):
    self.done = threading.Event()
    self.th = threading.Thread(target = async_task(self.serve, self.done))
    self.th.start()

  def stop(self):
    self.done.set()
    socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect( (self.hostname, self.port))
    self.sock.close()

class request_handler:
  def __init__(self, cluster):
    self.cluster = cluster

  def __call__(self, request):
    if request.path.startswith('/cluster/'):
      request.path = request.path[len('/cluster/'):]
      return self.cluster(request)
    if request.data:
      print request.path, request.data
      return json.dumps(request.data)
    else:
      path = request.path
      if path == '/':
        path = 'visualization/index.html'
      elif path.startswith('/resources/'):
        path = 'visualization/' + path
      else:
        path = path[1:]
      print path
      try:
        with open(path, 'r') as f:
          return f.read()
      except IOError:
        return 'dummy'


def main(argv):
  clusterfile = 'data/bike_path/cluster.pickle'
  try:
    opts, args = getopt.getopt(argv,"h",["cluster="])
  except getopt.GetoptError:
    print 'server [--cluster = <inputfile>]'
  for opt, arg in opts:
    if opt == '-h':
      print 'server [-- cluster = <inputfile>]'
      sys.exit()
    if opt in ("--cluster"):
       clusterfile = arg

  print 'cluster file:', clusterfile

  cluster = spat.trajectory.cluster.handler(clusterfile)
  s = http_server('localhost', 8000, request_handler(cluster))
  s.serve_forever()
  try:
    raw_input()
  except KeyboardInterrupt:
    pass
  s.stop()

if __name__ == "__main__":
  main(sys.argv[1:])

