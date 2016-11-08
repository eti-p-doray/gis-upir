import itertools

def pairwise(iterable):
  a, b = itertools.tee(iterable)
  next(b, None)
  return itertools.izip(a, b)

def drop(iterable, n):
  return itertools.islice(iterable, n, None)

def enumerate_reversed(L):
  for i in reversed(xrange(len(L))):
      yield i, L[i]

def peek(iterable):
  it = iter(iterable)
  try:
    first = next(it)
  except StopIteration:
    return None
  return first, itertools.chain([first], it)

def first(iterable):
  return peek(iterable)[0]

def empty(iterable):
  return peek(iterable) is None

def xor(a, b):
  return bool(a) ^ bool(b)

def merge_dicts(*dict_args):
  result = {}
  for dictionary in dict_args:
    result.update(dictionary)
  return result

