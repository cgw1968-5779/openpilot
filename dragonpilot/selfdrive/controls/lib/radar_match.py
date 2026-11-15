"""
dragonpilot (dppb, Corolla Altis; owner 2026-11-11): do not pair the camera's lead with a radar track in another lane.

Route 000001a7 21:25:40, 123 km/h: the camera saw the lead in our lane (y ~0.1-0.25 m, ~110 m, ~116 km/h, prob 0.5-0.6)
and radard's match_vision_to_track paired it with radar track 9780 in the LEFT lane (yRel -3.8 m, 93 m, 81 km/h): the
stock match has no lateral check and its speed check passes any moving target (v_ego + vRel > 3). The planner then
saw a car 93 m ahead closing at 42 km/h and sent -1.74 (actual -2.62, 123 -> 116.5 km/h) for ~0.5 s - a phantom brake.

A track is not offered to the match when it is clearly ANOTHER object than the camera's lead - all three at once:
farther than FAR_D, more than LAT_D to the side of the camera's lead, and its speed more than DV from the camera's.
Then the best of the remaining tracks is matched, or - none left - the camera's own lead is used (the lead is never
dropped). Within FAR_D everything is as stock, and the low-speed override still sees every track.
11-10/11-11 rlogs (101 min of radar leads): 152 frames (7.6 s) qualify, all slower tracks in an adjacent lane; of the
36 tracks 3 later cut into our lane and were matched again as soon as they were in it (closest 56 / 21 / 15 m).
"""
FAR_D = 40.0          # m
LAT_D = 2.5           # m
DV = 15.0 / 3.6       # m/s


def plausible_tracks(v_ego: float, lead, tracks: dict) -> dict:
  """the tracks the camera's lead may be matched with (radard RADAR_TO_CAMERA offset is within the 40 m margin)"""
  y_cam, v_cam = -float(lead.y[0]), float(lead.v[0])
  return {k: c for k, c in tracks.items()
          if not (c.dRel > FAR_D and abs(c.yRel - y_cam) > LAT_D and abs(c.vRel + v_ego - v_cam) > DV)}



"""
dragonpilot (dppb; owner 2026-11-16): a sudden switch to a much SLOWER radar track BEHIND the radar lead we have must be
seen for JUMP_T first.

Route 000001ba 13:36:19.69, 46 km/h: the lead was radar track 424 (28.2 m, 46 km/h). For ONE radarState frame the
camera's lead was matched with stationary track 447 (34.9 m, 2.8 km/h, y -0.4 - a return 6.7 m BEYOND the car we were
following; the stock vel_sane passed it at 9.5 < 10 m/s), the next frame was back on track 424. That one frame made the
planner ask -1.60 and the car slowed at -0.83 for ~1 s - a phantom brake. Route 000001c5 20:45:39.54, 64 km/h: the
same, track 1394 (58.6 m, 78 km/h) -> stationary track 1521 at 77.4 m for one frame, aTarget -2.64.

Stationary tracks cannot simply be ignored: ~half of the 11-15 "radar stopped / camera moving" switches were real
stopped cars the camera's speed caught up with only ~1 s later - but those are switches from the CAMERA's own lead (no
radar track yet) to a radar track, and they are left exactly as stock. Only when the lead we have is a radar track
that is still there, and the new one is another radar track more than JUMP_DD farther and more than JUMP_DV slower,
the radar track we had is kept until the new one has been chosen JUMP_T in a row (3 model frames: at most 0.10 s held).
A real slower object beyond the lead (the lead changing lanes in front of it) is used 0.10 s later.
11-15 replay (all 280 segments, 321682 frames): leadOne held in 23 places, none longer than 0.10 s.
"""
JUMP_DD = 1.0         # m
JUMP_DV = 5.0         # m/s
JUMP_T = 0.15         # s
JUMP_V_EGO = 5.0      # m/s


class LeadJumpGuard:
  def __init__(self, dt: float):
    self.n_need = max(int(round(JUMP_T / dt)), 1)
    self.out = None
    self.cnt = 0

  def update(self, v_ego: float, lead: dict, tracks: dict, lead_prob: float) -> dict:
    prev, tid = self.out, (self.out or {}).get('radarTrackId', -1)
    if (prev is not None and prev['status'] and prev.get('radar', False) and tid in tracks and v_ego > JUMP_V_EGO and
        lead['status'] and lead.get('radar', False) and lead['radarTrackId'] != tid and
        lead['dRel'] > tracks[tid].dRel + JUMP_DD and lead['vLead'] < tracks[tid].vLead - JUMP_DV):
      self.cnt += 1
      if self.cnt < self.n_need:
        self.out = tracks[tid].get_RadarState(lead_prob)
        return self.out
    else:
      self.cnt = 0
    self.out = lead
    return lead
