import logging
import math
import numpy

from spat import kalman
from spat import utility


def obs_transition_speed(x):
    return [x[0], x[1], math.sqrt(x[2]**2 + x[3]**2)]


def obs_transition(x):
    return [x[0], x[1]]


def filter_state(trajectory):
    P = numpy.identity(4) * 10.0
    F = numpy.identity(4)
    F[0][2] = 1.0
    F[1][3] = 1.0
    Q = numpy.diag([2.0, 2.0, 2.0, 2.0])

    y = trajectory['observations'][0]
    state = kalman.KalmanFilter(y[0:2] + [0.0, 0.0], P)
    trajectory['state'] = []
    trajectory['transition'] = (F, Q)

    for observation, accuracy, link in zip(
            trajectory['observations'],
            trajectory['accuracy'],
            trajectory['link']):
        error = 0.0
        state.time_update(F, Q)
        if observation != None:
            if observation[2] >= 0.0:
                R = numpy.diag([accuracy[0]**2, accuracy[1]**2, 1.0])
                error = state.unscented_measurment_update(observation,
                                                          obs_transition_speed, R)
            else:
                R = numpy.diag([accuracy[0]**2, accuracy[1]**2])
                error = state.unscented_measurment_update(observation[0:2],
                                                          obs_transition, R)

            if error > 50.0**2: # trajectory is broken
                logging.warning("trashing %s due to broken path", trajectory['id'])
                return None
        trajectory['state'].append(state.copy())

    if len(trajectory['state']) >= 2:
        return trajectory
    return None


def smooth_state(trajectory):
    trajectory = filter_state(trajectory)
    if trajectory is None:
        return None
    (F, Q) = trajectory['transition']
    next_state = trajectory['state'][-1]

    for i, _ in utility.drop(utility.enumerate_reversed(trajectory['state']), 1):
        trajectory['state'][i].smooth_update(next_state, F, Q)
        next_state = trajectory['state'][i]
        determinant = numpy.linalg.det(next_state.P)
        if determinant > 0.0 and math.log10(determinant) > 4*next_state.P.shape[0]:
            logging.warning("trashing %s due to missing data", trajectory['id'])
            return None

    R = Q
    R[2:4,2:4] = 0
    H = numpy.matrix([[1, 0, 0, 0], [0, 1, 0, 0]])

    start_state = None
    for i, state in enumerate(trajectory['state']):
        if start_state is None:
            start_state = state
            x = numpy.concatenate((state.x[0:2], numpy.zeros(2)))
            P = Q
            P[0:2,0:2] += state.P[0:2,0:2]
            start_state = kalman.KalmanFilter(x, P)
        else:
            start_state.time_update(numpy.identity(4), R)
            l = start_state.measurment_distance(state.x, numpy.identity(4), state.P)
            if l > 1.04:
              break
            start_state.measurment_update(state.x[0:2], H, state.P[0:2,0:2])

    logging.warning("starting trajectory at %d", i)
    if i > 1:
      trajectory['state'] = [start_state] + trajectory['state'][i:]

    end_state = None
    for i, state in enumerate(reversed(trajectory['state'])):
        if end_state is None:
            end_state = state
            x = numpy.concatenate((state.x[0:2], numpy.zeros(2)))
            P = Q
            P[0:2,0:2] += state.P[0:2,0:2]
            end_state = kalman.KalmanFilter(x, P)
        else:
            end_state.time_update(numpy.identity(4), R)
            l = end_state.measurment_distance(state.x, numpy.identity(4), state.P)
            if l > 1.04:
              break
            end_state.measurment_update(state.x[0:2], H, state.P[0:2,0:2])

    logging.warning("ending trajectory at %d", i)
    if i > 1:
      trajectory['state'] = trajectory['state'][:-i] + [end_state]

    if len(trajectory['state']) >= 2:
        return trajectory
    return None