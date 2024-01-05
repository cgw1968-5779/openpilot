#!/usr/bin/env python3
# The MIT License
#
# Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other of contributors.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from openpilot.common.numpy_fast import interp

DP_ACCEL_STOCK = 0
DP_ACCEL_ECO = 1
DP_ACCEL_NORMAL = 2
DP_ACCEL_SPORT = 3

# accel profile by @arne182 modified by cgw
_DP_CRUISE_MIN_V =       [-0.011, -0.034, -0.20, -0.36, -0.45, -0.40,  -0.35]
_DP_CRUISE_MIN_V_ECO =   [-0.010, -0.032, -0.18, -0.32, -0.40, -0.35,  -0.30]
_DP_CRUISE_MIN_V_SPORT = [-0.012, -0.036, -0.22, -0.40, -0.50, -0.45,  -0.40]
_DP_CRUISE_MIN_BP =      [0.,     4.0,    11.,   18.,   28.,   33.,    55.]

_DP_CRUISE_MAX_V =       [3.5, 3.5, 2.3, 1.8, 1.0,  .80,  .54,  .44, .34, .14]
_DP_CRUISE_MAX_V_ECO =   [3.5, 3.5, 1.8, 1.3, .90,  .68,  .50,  .38, .30, .092]
_DP_CRUISE_MAX_V_SPORT = [3.5, 3.5, 3.0, 2.6, 1.4,  1.0,  .7 ,  .6,  .38, .2]
_DP_CRUISE_MAX_BP =      [0.,  3,   6.,  8.,  11.,  15.,  20.,  25., 30., 55.]

class AccelController:

  def __init__(self):
    # self._params = Params()
    self._profile = DP_ACCEL_STOCK

  def set_profile(self, profile):
    try:
      self._profile = int(profile) if int(profile) in [DP_ACCEL_STOCK, DP_ACCEL_ECO, DP_ACCEL_NORMAL, DP_ACCEL_SPORT] else DP_ACCEL_STOCK
    except:
      self._profile = DP_ACCEL_STOCK

  def _dp_calc_cruise_accel_limits(self, v_ego):
    if self._profile == DP_ACCEL_ECO:
      min_v = _DP_CRUISE_MIN_V_ECO
      max_v = _DP_CRUISE_MAX_V_ECO
    elif self._profile == DP_ACCEL_SPORT:
      min_v = _DP_CRUISE_MIN_V_SPORT
      max_v = _DP_CRUISE_MAX_V_SPORT
    else:
      min_v = _DP_CRUISE_MIN_V
      max_v = _DP_CRUISE_MAX_V

    a_cruise_min = interp(v_ego, _DP_CRUISE_MIN_BP, min_v)
    a_cruise_max = interp(v_ego, _DP_CRUISE_MAX_BP, max_v)
    return a_cruise_min, a_cruise_max

  def get_accel_limits(self, v_ego, accel_limits):
    return accel_limits if self._profile == DP_ACCEL_STOCK else self._dp_calc_cruise_accel_limits(v_ego)

  def is_enabled(self):
    return self._profile != DP_ACCEL_STOCK
