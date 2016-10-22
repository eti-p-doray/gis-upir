import socket
import threading
import sys

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
  def __call__(self, request):
    path = request.path
    if path == '/':
      path = 'index.html'
    elif path.startswith('/data/'):
      path = '..' + path
    else:
      path = path[1:]
    print path
    try:
      with open(path, 'r') as f:
        return f.read()
    except IOError:
      return ''
    

s = http_server('localhost', 8000, request_handler())
s.serve_forever()
raw_input('Press any key to quit')
s.stop()

