import math
import numpy

from spat import kalman
from spat import utility


def obs_transition_speed(x):
    return [x[0], x[1], math.sqrt(x[2]**2 + x[3]**2)]


def obs_transition(x):
    return [x[0], x[1]]


def filter_state(trajectories):
    P = numpy.identity(4) * 10.0
    F = numpy.identity(4)
    F[0][2] = 1.0
    F[1][3] = 1.0
    Q = numpy.diag([2.0, 2.0, 2.0, 2.0])

    def init_trajectory(t):
        return {
            'id': t['id'],
            'observations':[],
            'link': [],
            'state':[],
            'accuracy':[],
            'transition': (F, Q)
        }

    for trajectory in trajectories:
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
                    print('trashing ', trajectory['id'], ' due to broken path')
                    break
            trajectory['state'].append(state.copy())

        if len(trajectory['state']) >= 2:
            yield trajectory


def smooth_state(trajectories):
    filtered_trajectories = filter_state(trajectories)
    for t in filtered_trajectories:
        (F, Q) = t['transition']
        next_state = t['state'][-1]

        broken = False
        for i, _ in utility.drop(utility.enumerate_reversed(t['state']), 1):
            t['state'][i].smooth_update(next_state, F, Q)
            next_state = t['state'][i]
            if math.log10(numpy.linalg.det(next_state.P)) > 4*next_state.P.shape[0]:
                print('trashing ', t['id'], ' due to broken path')
                broken = True
                break

        if not broken:
            yield t