# dppb (owner 2026-11-19): the lead's licence plate is found in the road camera image and outlined on the screen
import numpy as np

from dragonpilot.selfdrive.ui import plate_finder as pf

H, W = 1208, 1928
K = np.array([[2648.0, 0.0, 964.0], [0.0, 2648.0, 604.0], [0.0, 0.0, 1.0]])
VIEW = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])   # view frame from car frame, level camera
CAM_H = 1.22


def scene(d_rel, y_off=0.0, plate=True, tint=0, strokes=7, body=70):
  """a grey car rear with a white plate carrying dark strokes, as the road camera would see it at d_rel"""
  rng = np.random.default_rng(1)
  y = np.full((H, W), 90, dtype=np.float64) + rng.normal(0, 2.0, (H, W))
  u = np.full((H // 2, W // 2), 128, dtype=np.uint8)
  v = np.full((H // 2, W // 2), 128, dtype=np.uint8)
  d = d_rel + pf.PF_CAM_OFFSET
  s = K[0, 0] / d
  cx, cy = 964 + y_off * s, 604 + (CAM_H - 0.55) * s            # plate centre 0.55 m above the road
  bw, bh = 1.7 * s, 1.1 * s
  y[int(cy - 0.7 * bh):int(cy + 0.3 * bh), int(cx - bw / 2):int(cx + bw / 2)] = body
  w, h = pf.PLATE_W * s, pf.PLATE_H * s
  x0, y0, x1, y1 = int(round(cx - w / 2)), int(round(cy - h / 2)), int(round(cx + w / 2)), int(round(cy + h / 2))
  if plate:
    y[y0:y1, x0:x1] = 215
    if tint:
      v[y0 // 2:y1 // 2, x0 // 2:x1 // 2] = 128 + tint
    sw = (x1 - x0) * 0.86 / (2 * strokes)
    for k in range(strokes):
      a = int(x0 + (x1 - x0) * 0.07 + 2 * k * sw + (0.6 * sw if k >= 3 else 0.0))
      y[int(y0 + 0.2 * h):int(y1 - 0.2 * h), a:a + max(int(sw * 0.8), 2)] = 40
  return np.clip(y, 0, 255).astype(np.uint8), (u, v), (x0, y0, x1, y1)


def test_finds_the_plate_at_the_radar_distance():
  for d_rel, y_off in ((2.0, 0.0), (3.5, 0.0), (3.5, 0.4), (6.0, -0.3), (10.0, 0.0)):
    y, uv, (x0, y0, x1, y1) = scene(d_rel, y_off)
    r = pf.locate(y, uv, d_rel, -y_off, CAM_H, VIEW, K)
    assert r is not None, d_rel
    w = x1 - x0
    assert abs(r[0] - x0) < 0.08 * w and abs(r[2] - x1) < 0.08 * w, (d_rel, r, x0, x1)
    assert abs(0.5 * (r[1] + r[3]) - 0.5 * (y0 + y1)) < 0.25 * (y1 - y0)


def test_nothing_without_a_plate_or_with_a_coloured_or_wrong_sized_one():
  y, uv, _ = scene(3.5, plate=False)
  assert pf.locate(y, uv, 3.5, 0.0, CAM_H, VIEW, K) is None
  y, uv, _ = scene(3.5, tint=60)                                 # a red lamp of the plate's shape
  assert pf.locate(y, uv, 3.5, 0.0, CAM_H, VIEW, K) is None
  y, uv, _ = scene(3.5)                                          # the radar says twice as far: this is not its plate
  assert pf.locate(y, uv, 9.0, 0.0, CAM_H, VIEW, K) is None


def test_box_is_shown_after_two_finds_and_sits_at_the_plate():
  y, uv, box = scene(3.5)
  f = pf.PlateFinder()
  r = pf.locate(y, uv, 3.5, 0.0, CAM_H, VIEW, K)
  f._update(r, VIEW, K)
  assert f.box() is None                                         # one find is not enough
  f._update(r, VIEW, K)
  b = f.box()
  assert b is not None and b.shape == (4, 3)
  assert abs(np.linalg.norm(b[1] - b[0]) - pf.PLATE_W) < 0.01     # its own width gives the distance
  assert abs(b[:, 0].mean() - (3.5 + pf.PF_CAM_OFFSET)) < 0.35
  assert abs(b[:, 1].mean()) < 0.05 and abs(b[:, 2].mean() - (CAM_H - 0.55)) < 0.06
  f._update(None, VIEW, K)                                       # a missed frame keeps it for a moment
  assert f.box() is not None


def test_a_find_somewhere_else_starts_again():
  f = pf.PlateFinder()
  ya, uva, _ = scene(3.5, 0.0)
  yb, uvb, _ = scene(3.5, 0.8)
  f._update(pf.locate(ya, uva, 3.5, 0.0, CAM_H, VIEW, K), VIEW, K)
  f._update(pf.locate(yb, uvb, 3.5, -0.8, CAM_H, VIEW, K), VIEW, K)
  assert f.box() is None


def test_out_of_range_or_behind_is_ignored():
  y, uv, _ = scene(3.5)
  assert pf.locate(y, uv, 3.5, 30.0, CAM_H, VIEW, K) is None       # a lead far off to the side: outside the image
