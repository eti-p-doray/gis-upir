import socket, sys, getopt, threading, json
from jquery_unparam import jquery_unparam
sys.path.append(".")
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer

import spat.trajectory.cluster

class async_task:
  def __init__(self, f, stop):
    self.f = f
    self.stop = stop

  def __call__(self):
    while self.stop.is_set() == False:
      self.f()

class RequestHandler(BaseHTTPRequestHandler):
  def __init__(self, cluster, *args):
    self.cluster = cluster
    BaseHTTPRequestHandler.__init__(self, *args)

  #Handler for the GET requests
  def do_GET(self):
    data_idx = self.path.find('?')
    self.data = []
    if data_idx != -1:
      self.data = jquery_unparam(self.path[data_idx+1:])

    self.send_response(200)
    self.send_header('Content-type','text/html')
    self.end_headers()

    if self.path.startswith('/cluster/'):
      self.path = self.path[len('/cluster/'):]
      self.wfile.write(self.cluster.do_GET(self))
    else:
      path = self.path
      if path == '/':
        path = 'visualization/index.html'
      elif path.startswith('/resources/'):
        path = 'visualization/' + path
      else:
        path = path[1:]
      print path
      try:
        with open(path, 'r') as f:
          self.wfile.write(f.read())
      except IOError:
        self.send_error(404,'File Not Found: %s' % self.path)

    return

def handle_requests_using(cluster):
  return lambda *args: RequestHandler(cluster, *args)


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

  #cluster = spat.trajectory.cluster.handler(clusterfile)
  #s = http_server('localhost', 8000, request_handler(cluster))
  try:
    s = HTTPServer(('localhost', 8000), handle_requests_using(None))
    s.serve_forever()
  except KeyboardInterrupt:
    print '^C received, shutting down the web server'
    s.socket.close()


if __name__ == "__main__":
  main(sys.argv[1:])

