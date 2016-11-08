from scipy import linalg
import numpy as np

def xcov(X, Y):
  """P = np.zeros([X.shape[0], Y.shape[0]])
  meanx = np.mean(X, 1)
  meany = np.mean(Y, 1)
  n = X.shape[1]
  for i in range(n):
    P += np.outer(X[:,i] - meanx, Y[:,i] - meany)
  P /= n
  return P"""
  n = X.shape[0]
  m = Y.shape[0]
  return np.cov(X, Y, bias=True)[0:n,-m:]

def LDL(A):
  A = array(A)
  n = A.shape[1]
  L = array(eye(n))
  D = zeros((n, 1))
  for i in range(n):
    D[i] = A[i, i] - dot(L[i, 0:i] ** 2, D[0:i])
    for j in range(i + 1, n):
      L[j, i] = (A[j, i] - dot(L[j, 0:i] * L[i, 0:i], D[0:i])) / D[i]
  return L, D

class KalmanFilter:
  def __init__(self, initial_state, initial_state_covariance):
    self.x = np.asarray(initial_state)
    self.P = np.asmatrix(initial_state_covariance)

  def time_update(self, F, Q):
    self.x = np.dot(np.asarray(F), self.x)
    self.P = F * self.P * F.T + Q
    return (self.x, self.P)

  def unscented_time_update(self, f, Q):
    n = self.x.shape[0]
    noise = linalg.sqrtm(n * self.P).T
    sigmax = np.empty([2*n, n])
    for i in range(n):
      sigmax[i] = f(self.x + noise[i])
      sigmax[n+i] = f(self.x - noise[i])
    
    self.x = np.mean(sigmax, 0)
    self.P = np.cov(sigmax.T, bias=True) + Q
    return (self.x, self.P)

  def measurment_update(self, y, H, R):
    U = np.dot(self.P, H.T)
    S = np.linalg.inv(np.dot(H, U) + R)
    z = y - np.dot(H, self.x)
    l = np.dot(np.dot(S, z), z)
    K = np.dot(U, S)
    self.x += np.dot(K, z)
    self.P -= np.dot(np.dot(K, H), self.P)
    return self.x, self.P, l

  def unscented_measurment_update(self, y, h, R):
    y = np.asarray(y)
    n = self.x.shape[0]
    m = y.shape[0]

    noise = linalg.sqrtm(n * np.asarray(self.P)).T
    sigmax = np.empty([2*n, n])
    sigmay = np.empty([2*n, m])
    for i in range(n):
      sigmax[i] = self.x + noise[i]
      sigmax[n+i] = self.x - noise[i]
      sigmay[i] = h(sigmax[i])
      sigmay[n+i] = h(sigmax[n+i])
    Py = np.asmatrix(np.cov(sigmay.T, bias=True) + R)
    S = np.linalg.inv(Py)
    Pxy = xcov(sigmax.T, sigmay.T)
    z = y - np.mean(sigmay, 0)
    K = Pxy * S
    l = np.dot(np.dot(np.asarray(S), z), z)
    self.x += np.dot(np.asarray(K), z)
    self.P = self.P - K * Py * K.T
    return self.x, self.P, l

  def smooth_update(self, (x1, P1), F, Q):
    x2 = np.dot(F, x1)
    P2 = F * P1 * F.T + Q
    K = P1 * F.T * np.linalg.inv(P2)
    self.x = x1 + np.dot(np.asarray(K), self.x - x2)
    self.P = P1 - K * (P2 - self.P) * K.T
    return (self.x, self.P)

  def unscented_smooth_update(self, (x1, P1), f, Q):
    noise = linalg.sqrtm(n * P1).T
    sigma1 = np.empty([2*n, n])
    sigma2 = np.empty([2*n, n])
    for i in range(n):
      sigma1[i] = x1 + noise[i]
      sigma1[n+i] = x1 - noise[i]
      sigma2[i] = f(sigma1[i])
      sigma2[n+i] = f(sigma1[n+i])
    x2 = np.mean(sigma2, 0)
    P2 = np.cov(sigma2.T, bias=True) + Q
    P12 = xcov(sigma1.T, sigma2.T)
    K = P12 * np.linalg.inv(P2)
    self.x = x + np.dot(np.asarray(K), self.x - x2)
    self.P = P1 - K * (P2 - self.P) * K.T
    return self.x, self.P

  def project_constraint(self, D, d):
    U = self.P * D.T
    z = d - np.dot(np.asarray(D), self.x)
    S = np.linalg.inv(D * U)
    l = np.dot(z, np.dot(np.asarray(S), z))
    K = U * S
    self.x += np.dot(np.asarray(K), z)
    self.P = self.P - K * D * self.P
    return self.x, self.P, l



