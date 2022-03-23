#!/usr/bin/env python3
from enum import Enum


class LongTunes(Enum):
  PEDAL = 0
  TSS2 = 1
  TSS = 2

class LatTunes(Enum):
  INDI_PRIUS = 0
  LQR_RAV4 = 1
  PID_A = 2
  PID_B = 3
  PID_C = 4
  PID_D = 5
  PID_E = 6
  PID_F = 7
  PID_G = 8
  PID_I = 9
  PID_H = 10
  PID_J = 11
  PID_K = 12
  PID_L = 13
  PID_M = 14
  PID_N = 15
  INDI_PRIUS_TSS2 = 16
  INDI_RAV4_TSS2 = 17

###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.TSS2 or name == LongTunes.PEDAL:
    tune.deadzoneBP = [0., 8.05]
    tune.deadzoneV = [.0, .14]
    tune.kpBP = [0., 5., 20.]
    tune.kpV = [1.3, 1.0, 0.7]
    tune.kiBP = [0., 3., 5., 12., 20., 27.]
    tune.kiV = [.34, .32, .3, .23, .17, .02]
  # Default longitudinal tune
  elif name == LongTunes.TSS:
    tune.deadzoneBP = [0., 9.]
    tune.deadzoneV = [0., .15]
    tune.kpBP = [0., 5., 35.]
    tune.kiBP = [0., 35.]
    tune.kpV = [3.6, 2.4, 1.5]
    tune.kiV = [0.54, 0.36]
  else:
    raise NotImplementedError('This longitudinal tune does not exist')


###### LAT ######
def set_lat_tune(tune, name):
  if name == LatTunes.INDI_PRIUS:
    tune.init('indi')
    tune.indi.innerLoopGainBP = [0.]
    tune.indi.innerLoopGainV = [4.0]
    tune.indi.outerLoopGainBP = [0.]
    tune.indi.outerLoopGainV = [3.0]
    tune.indi.timeConstantBP = [0.]
    tune.indi.timeConstantV = [1.0]
    tune.indi.actuatorEffectivenessBP = [0.]
    tune.indi.actuatorEffectivenessV = [1.0]

  elif name == LatTunes.INDI_PRIUS_TSS2:
    tune.init('indi')
    tune.indi.innerLoopGainBP = [20, 24, 30]
    tune.indi.innerLoopGainV = [7.25, 7.5, 9]
    tune.indi.outerLoopGainBP = [20, 24, 30]
    tune.indi.outerLoopGainV = [6, 7.25, 6]
    tune.indi.timeConstantBP = [20, 24]
    tune.indi.timeConstantV = [2.0, 2.2]
    tune.indi.actuatorEffectivenessBP = [20, 24]
    tune.indi.actuatorEffectivenessV = [2, 3]

  elif name == LatTunes.INDI_RAV4_TSS2:
    tune.init('indi')
    tune.indi.innerLoopGainBP = [8.3,   12.10,  13.9,  16.7,  20,    30]
    tune.indi.innerLoopGainV = [5.2,    7.8,    8.8,   10,    12.8,  15]
    tune.indi.outerLoopGainBP = [8.3,   12.10,  13.9,  16.7,  20,    30]
    tune.indi.outerLoopGainV = [4.98,  7.688, 8.6, 9.864, 12.785, 14.99]
    tune.indi.timeConstantBP = [6, 8.33, 13.9, 18,  20,  23.88, 23.89,  30,  40]
    tune.indi.timeConstantV = [0.2, 0.2424, 0.4, 0,51, 0.83, 1.3, 1.5, 1.9,  2.4]
    #tune.indi.timeConstantV = [0.15, 0.2425, 0.4, 0,57, 0.9, 1.3, 1.5, 1.9,  2.4]
    tune.indi.actuatorEffectivenessBP = [10, 11, 11.1, 25, 33]
    tune.indi.actuatorEffectivenessV = [15, 15, 15, 14, 12.5]

  elif name == LatTunes.LQR_RAV4:
    tune.init('lqr')
    tune.lqr.scale = 1500.0
    tune.lqr.ki = 0.05
    tune.lqr.a = [0., 1., -0.22619643, 1.21822268]
    tune.lqr.b = [-1.92006585e-04, 3.95603032e-05]
    tune.lqr.c = [1., 0.]
    tune.lqr.k = [-110.73572306, 451.22718255]
    tune.lqr.l = [0.3233671, 0.3185757]
    tune.lqr.dcGain = 0.002237852961363602

  elif 'PID' in str(name):
    tune.init('pid')
    tune.pid.kiBP = [0.0]
    tune.pid.kpBP = [0.0]
    if name == LatTunes.PID_A:
      tune.pid.kpV = [0.2]
      tune.pid.kiV = [0.05]
      tune.pid.kf = 0.00003
    elif name == LatTunes.PID_B:
      tune.pid.kpV = [0.6]
      tune.pid.kiV = [0.05]
      tune.pid.kf = 0.00006
    elif name == LatTunes.PID_C:
      tune.pid.kpV = [0.6]
      tune.pid.kiV = [0.1]
      tune.pid.kf = 0.00006
    elif name == LatTunes.PID_D:
      tune.pid.kpV = [0.6]
      tune.pid.kiV = [0.1]
      tune.pid.kf = 0.00007818594
    elif name == LatTunes.PID_E:
      tune.pid.kpV = [0.6]
      tune.pid.kiV = [0.15]
      tune.pid.kf = 0.00007818594
    elif name == LatTunes.PID_F:
      tune.pid.kpV = [0.723]
      tune.pid.kiV = [0.0428]
      tune.pid.kf = 0.00006
    elif name == LatTunes.PID_G:
      tune.pid.kpV = [0.18]
      tune.pid.kiV = [0.015]
      tune.pid.kf = 0.00012
    elif name == LatTunes.PID_H:
      tune.pid.kpV = [0.17]
      tune.pid.kiV = [0.03]
      tune.pid.kf = 0.00006
    elif name == LatTunes.PID_I:
      tune.pid.kpV = [0.15]
      tune.pid.kiV = [0.05]
      tune.pid.kf = 0.00004
    elif name == LatTunes.PID_J:
      tune.pid.kpV = [0.19]
      tune.pid.kiV = [0.02]
      tune.pid.kf = 0.00007818594
    elif name == LatTunes.PID_L:
      tune.pid.kpV = [0.3]
      tune.pid.kiV = [0.05]
      tune.pid.kf = 0.00006
    elif name == LatTunes.PID_M:
      tune.pid.kpV = [0.3]
      tune.pid.kiV = [0.05]
      tune.pid.kf = 0.00007
    elif name == LatTunes.PID_N:
      tune.pid.kpV = [0.35]
      tune.pid.kiV = [0.15]
      tune.pid.kf = 0.00007818594
    else:
      raise NotImplementedError('This PID tune does not exist')
  else:
    raise NotImplementedError('This lateral tune does not exist')
