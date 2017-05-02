import math, operator
import scipy.stats, numpy


class RunningStats:
  def __init__(self):
    self.totalWeight = 0.0
    self.m = 0.0
    self.M2 = 0.0
    self.M3 = 0.0

  def push(self, x, w = 1.0): 

    m1 = self.m
    delta1 = x - m1
    self.totalWeight += w
    self.m += w * delta1 / self.totalWeight
    m2 = self.m

    delta2 = x - m2
    delta12 = w * delta1 * delta2

    self.M3 += delta12 * (m1 - 2.0 * m2 + x) + 3.0 * self.M2 * (m1 - m2)
    self.M2 += delta12

  def mean(self):
    return self.m

  def variance(self):
    return self.M2 / self.totalWeight

  def skewness(self):
    return math.sqrt(self.totalWeight) * self.M3 / (self.M2 ** (3.0/2.0))


"""data = [1.0, 1.0, 2.0, 3.0]
wdata = [(2.0, 1.0), (1.0, 2.0), (1.0, 3.0)]
stats = RunningStats()
for (w,x) in wdata:
  stats.push(x, w)

print math.sqrt(stats.variance()), stats.skewness()
print numpy.std(data)

def accumulate(iterable, func=operator.add):
  'Return running totals'
  # accumulate([1,2,3,4,5]) --> 1 3 6 10 15
  # accumulate([1,2,3,4,5], operator.mul) --> 1 2 6 24 120
  total = 0.0
  for element in iterable:
    total = func(total, element)
  return total

sum = accumulate(data)
mean = sum / len(data)

u2 = accumulate(((x - mean)**2 for x in data))
variance = u2 / len(data)
std = math.sqrt(variance)


s3 = accumulate((x**3.0 for x in data))
M3 = s3 / len(data) - mean ** 3.0 - 3.0 * mean * variance

u3 = accumulate(( ((x - mean) / std) ** 3.0 for x in data))

print sum, mean, std, M3 * len(data), u3 / len(data)"""
    