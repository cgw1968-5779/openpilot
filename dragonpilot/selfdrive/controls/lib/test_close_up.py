# dppb (owner 2026-11-20): standing more than 4 m behind the lead -> creep forward and soft-stop at 3.5 m
from types import SimpleNamespace

from dragonpilot.selfdrive.controls.lib import close_up as cu

DT = 0.05


def lead(d, v=0.0, status=True):
  return SimpleNamespace(status=status, dRel=d, vLead=v)


def run(d0, steps=600, lead_v=lambda t: 0.0, plan=-0.3, v0=0.0):
  """a car that does what the target asks (0.3 s lag); returns (CloseUp, rows (t, v, a_target, gap, active))"""
  c = cu.CloseUp(DT)
  v, a, gap, rows = v0, 0.0, d0, []
  for k in range(steps):
    t = k * DT
    vl = lead_v(t)
    out = c.update(v, lead(gap, vl), plan, False)
    a += ((out if c.active else (0.0 if v <= 0.0 else out)) - a) * DT / 0.3
    v = max(v + a * DT, 0.0) if c.active or v > 0.0 else 0.0
    gap += (vl - v) * DT
    rows.append((t, v, out, gap, c.active))
  return c, rows


def test_far_stop_closes_up_to_the_gap_softly():
  for d0 in (4.3, 5.3, 6.2, 9.0):
    c, rows = run(d0)
    assert not c.active                                           # finished
    assert abs(rows[-1][3] - cu.CU_GAP) < 0.35, (d0, rows[-1][3])
    assert max(r[1] for r in rows) <= cu.CU_V + 0.1               # a creep, not a launch
    assert min(r[2] for r in rows if r[4]) >= -cu.CU_B_MAX - 1e-9  # a light brake
    assert max(r[2] for r in rows if r[4]) <= cu.CU_A + 1e-9
    assert rows[int(cu.CU_WAIT / DT) - 2][1] == 0.0               # not before the wait is over


def test_not_inside_4_m_or_while_the_lead_moves_or_without_a_lead():
  c, rows = run(3.9)
  assert not any(r[4] for r in rows) and rows[-1][3] == 3.9      # within 4 m: stays
  c, rows = run(6.0, lead_v=lambda t: 0.5)
  assert not any(r[4] for r in rows)                             # a moving lead is the plan's
  c = cu.CloseUp(DT)
  for _ in range(100):
    assert c.update(0.0, lead(6.0, status=False), -0.3, False) == -0.3
    assert c.update(0.0, lead(6.0), -0.3, True) == -0.3           # off / driver brake
  assert not c.active


def test_hands_back_when_the_lead_moves_off_or_jumps():
  c, rows = run(8.0, lead_v=lambda t: 0.0 if t < 4.0 else 1.5, steps=140)
  assert any(r[4] for r in rows[:80]) and not c.active            # it was closing up, then the lead drove off
  c = cu.CloseUp(DT)
  for _ in range(60):
    c.update(0.0, lead(6.0), -0.3, False)
  assert c.active
  assert c.update(0.5, lead(8.0), -0.3, False) == -0.3 and not c.active   # dRel jumped 2 m: another lead


def test_target_moves_smoothly():
  c, rows = run(6.2)
  on = [r[2] for r in rows if r[4]]
  assert max(abs(b - a) for a, b in zip(on[:-1], on[1:], strict=True)) <= cu.CU_J * DT + 1e-9
