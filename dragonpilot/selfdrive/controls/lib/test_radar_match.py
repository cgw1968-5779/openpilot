from types import SimpleNamespace as N

from dragonpilot.selfdrive.controls.lib.radar_match import plausible_tracks, LeadJumpGuard

V_EGO = 123 / 3.6


def test_other_lane_slower_track_is_not_offered():
  # route 1a7 21:25:40: camera lead in our lane ~110 m at 116 km/h; radar track 93 m, 3.8 m left, 81 km/h
  lead = N(y=[0.2], v=[116 / 3.6])
  tracks = {9780: N(dRel=93.0, yRel=-3.8, vRel=(81 - 123) / 3.6), 1: N(dRel=105.0, yRel=-0.3, vRel=(115 - 123) / 3.6)}
  assert set(plausible_tracks(V_EGO, lead, tracks)) == {1}


def test_any_one_condition_alone_keeps_the_track():
  lead = N(y=[0.0], v=[100 / 3.6])
  near = {1: N(dRel=35.0, yRel=-3.8, vRel=-40 / 3.6)}                   # within 40 m: stock
  same_lane = {1: N(dRel=90.0, yRel=-1.0, vRel=-40 / 3.6)}              # not to the side
  same_speed = {1: N(dRel=90.0, yRel=-3.8, vRel=(100 - 123) / 3.6)}     # speed agrees
  for t in (near, same_lane, same_speed):
    assert plausible_tracks(V_EGO, lead, t) == t


DT_MDL = 0.05


class Tr:
  def __init__(self, tid, d, vl):
    self.tid, self.dRel, self.vLead = tid, d, vl

  def get_RadarState(self, prob=0.0):
    return {'status': True, 'radar': True, 'radarTrackId': self.tid, 'dRel': self.dRel, 'vLead': self.vLead}


def vision(d, vl):
  return {'status': True, 'radar': False, 'radarTrackId': -1, 'dRel': d, 'vLead': vl}


def test_one_frame_jump_to_a_stationary_return_behind_the_lead_is_not_used():
  # route 1ba 13:36:19.69: lead track 424 28.2 m 46 km/h; one frame matched to stationary track 447 at 34.9 m
  g = LeadJumpGuard(DT_MDL)
  car, ghost = Tr(424, 28.2, 46 / 3.6), Tr(447, 34.9, 2.8 / 3.6)
  tracks = {424: car, 447: ghost}
  v = 46 / 3.6
  assert g.update(v, car.get_RadarState(), tracks, 0.9)['radarTrackId'] == 424
  assert g.update(v, ghost.get_RadarState(), tracks, 0.9)['radarTrackId'] == 424
  assert g.update(v, car.get_RadarState(), tracks, 0.9)['radarTrackId'] == 424


def test_a_real_one_is_used_after_two_frames():
  g = LeadJumpGuard(DT_MDL)
  car, stopped = Tr(1, 20.0, 30 / 3.6), Tr(2, 25.0, 0.0)
  tracks = {1: car, 2: stopped}
  g.update(8.0, car.get_RadarState(), tracks, 0.9)
  out = [g.update(8.0, stopped.get_RadarState(), tracks, 0.9)['radarTrackId'] for _ in range(4)]
  assert out == [1, 1, 2, 2]


def test_stock_everywhere_else():
  car, stopped = Tr(1, 20.0, 30 / 3.6), Tr(2, 25.0, 0.0)
  tracks = {1: car, 2: stopped}
  cases = [
    (8.0, vision(30.0, 30 / 3.6), stopped.get_RadarState()),    # camera lead -> radar track: real stops, stock
    (8.0, car.get_RadarState(), Tr(2, 18.0, 0.0).get_RadarState()),   # nearer than the lead: stock
    (8.0, car.get_RadarState(), Tr(2, 25.0, 6.0).get_RadarState()),   # not 5 m/s slower: stock
    (4.0, car.get_RadarState(), stopped.get_RadarState()),       # below 5 m/s: stock
    (8.0, car.get_RadarState(), vision(25.0, 0.0)),              # to the camera's lead: stock
  ]
  for v, first, new in cases:
    g = LeadJumpGuard(DT_MDL)
    g.update(v, first, tracks, 0.9)
    assert g.update(v, new, tracks, 0.9) == new
  g = LeadJumpGuard(DT_MDL)                                      # the lead's track is gone: stock
  g.update(8.0, car.get_RadarState(), tracks, 0.9)
  assert g.update(8.0, stopped.get_RadarState(), {2: stopped}, 0.9)['radarTrackId'] == 2
