import itertools
import numpy
import math


def pairwise(iterable):
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


def drop(iterable, n):
    return itertools.islice(iterable, n, None)


def enumerate_reversed(L):
    for i in reversed(range(len(L))):
        yield i, L[i]


def peek(iterable):
    it = iter(iterable)
    try:
        first = next(it)
    except StopIteration:
        return None
    return itertools.chain([first], it)


def first(iterable):
    return peek(iterable)[0]


def xor(a, b):
    return bool(a) ^ bool(b)


def merge_dicts(*dict_args):
    result = {}
    for dictionary in dict_args:
        result.update(dictionary)
    return result


def point_to_vec(p):
    return p.x, p.y


def normal2d(v):
    return numpy.array([v[1], -v[0]])


def sqnorm2d(v):
    return v[0]**2 + v[1]**2


def norm2d(v):
    return math.sqrt(sqnorm2d(v))


def distance2d(x1, x2):
    return norm2d(x1[0:2] - x2[0:2])


def bb_bounds(x, y, width, height):
    return x-width, y-width, x+width, y+height


def bb_buffer(point, width):
    return point.x-width, point.y-width, point.x+width, point.y+width

def intersect(a, b):
    return a[0] < b[2] and a[2] > b[0]  and a[1] < b[3] and a[3] > b[1]
