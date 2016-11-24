from scipy import linalg
import numpy as np

def xcov(X, Y):
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
    self.x = np.array(initial_state)
    self.P = np.matrix(initial_state_covariance)

  def copy(self):
    return KalmanFilter(self.x, self.P)

  def time_update(self, F, Q):
    self.x = np.dot(np.asarray(F), self.x)
    self.P = F * self.P * F.T + Q

  def unscented_time_update(self, f, Q):
    n = self.x.shape[0]
    noise = linalg.sqrtm(n * self.P).T
    sigmax = np.empty([2*n, n])
    for i in range(n):
      sigmax[i] = f(self.x + noise[i])
      sigmax[n+i] = f(self.x - noise[i])
    
    self.x = np.mean(sigmax, 0)
    self.P = np.cov(sigmax.T, bias=True) + Q

  def measurment_update(self, y, H, R):
    H = np.asmatrix(H)
    U = self.P * H.T
    S = np.linalg.inv(H * U + R)
    z = y - np.dot(np.asarray(H), self.x)
    l = np.dot(np.dot(np.asarray(S), z), z)
    K = np.dot(U, S)
    self.x += np.dot(np.asarray(K), z)
    self.P -= K * H * self.P
    return l

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
    return l

  def smooth_update(self, next, F, Q):
    x1 = np.dot(F, self.x)
    P1 = F * self.P * F.T + Q
    K = self.P * F.T * np.linalg.inv(P1)
    self.x = self.x + np.dot(np.asarray(K), next.x - x1)
    self.P = P1 - K * (P1 - next.P) * K.T

  def unscented_smooth_update(self, next, f, Q):
    noise = linalg.sqrtm(n * self.P).T
    sigma1 = np.empty([2*n, n])
    sigma2 = np.empty([2*n, n])
    for i in range(n):
      sigma0[i] = self.x + noise[i]
      sigma0[n+i] = self.x - noise[i]
      sigma1[i] = f(sigma1[i])
      sigma1[n+i] = f(sigma1[n+i])
    x1 = np.mean(sigma1, 0)
    P1 = np.cov(sigma1.T, bias=True) + Q
    P01 = xcov(sigma0.T, sigma1.T)
    K = P12 * np.linalg.inv(P1)
    self.x = x + np.dot(np.asarray(K), next.x - x1)
    self.P = P1 - K * (P1 - next.P) * K.T

  def project_constraint(self, d, D):
    D = np.asmatrix(D)
    U = self.P * D.T
    z = d - np.dot(np.asarray(D), self.x)
    S = np.linalg.inv(D * U)
    l = np.dot(z, np.dot(np.asarray(S), z))
    K = U * S
    self.x += np.dot(np.asarray(K), z)
    self.P = self.P - K * D * self.P
    return l

  def transform(self, D):
    D = np.asmatrix(D)
    return KalmanFilter(np.dot(np.asarray(D), self.x), D * self.P * D.T)

  def measurment_distance(self, y, H, R):
    H = np.asmatrix(H)
    U = self.P * H.T
    S = np.linalg.inv(H * U + R)
    z = y - np.dot(np.asarray(H), self.x)
    l = np.dot(np.dot(np.asarray(S), z), z)
    return l

  def constraint_distance(self, d, D):
    D = np.asmatrix(D)
    S = np.linalg.inv(D * self.P * D.T)
    z = d - np.dot(np.asarray(D), self.x)
    l = np.dot(z, np.dot(np.asarray(S), z))
    return l



