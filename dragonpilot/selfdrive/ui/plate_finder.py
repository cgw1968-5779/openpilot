"""
dragonpilot (dppb): the lead's licence plate, found in the road camera image and outlined on the screen.

Owner 2026-11-19: "when the system has found the plate, show it: a rectangle of the plate's size around it and four
straight lines from the four corners of the screen to the matching corners of the plate". Background: the plate is
a rigid object of known size (Taiwan car / truck plate 380 x 160 mm) - on 152 recorded stops its width in the image
agreed with the radar distance within +-0.17 m for cars and moved 1 px from frame to frame, so it is the candidate
for a second, radar-independent distance; this overlay is how the owner sees what the finder has got.

The device has no OpenCV, so the finder is numpy only (4 ms per frame on a PC, about 45 ms on the device, run 4 times a second in a
thread of the UI process, on its own road camera stream - the screen may be showing the wide camera):
 - where: around the radar lead (leadOne within PF_MAX_D), +-PF_HALF_W to the side, from the road up to PF_TOP;
 - expected size: the plate's width at the radar distance (camera PF_CAM_OFFSET behind what the radar measures to);
 - search (image shrunk so the expected plate is ~56 px wide, integral images, 5 scales): the box whose text block
   carries horizontal gradient (the strokes) in all four of its quarters and clearly more than the ring around it;
 - the plate itself: the run of stroke / plate-edge gradient around that box gives its rows and columns (calibrated:
   run width = plate width x 1.003 on 25 stops); it must be 0.85-1.18 x the expected width, with text 30-85 % of
   its height, and nearly colourless (a plate is white: a tail light, a painted truck body or a sign is not);
 - shown only after two finds in a row at the same place, for PF_KEEP after the last find.
Checked on the 152 stops (3 frames each): found on 75, every box looked at was on a real plate (taillights, road
markings and the big painted numbers on truck bodies are rejected); it misses dim / backlit plates and motorcycles.
Wrong boxes matter more than missed ones here.

Nothing but drawing uses the result. One hook line in augmented_road_view (plus the constructor line).
"""
import threading
import time

import numpy as np

PLATE_W, PLATE_H = 0.38, 0.16          # m (Taiwan car / truck plate)
SCALES = (0.80, 0.90, 1.00, 1.12, 1.25)
TARGET_W = 56                          # px: the search runs on an image shrunk so the expected plate is about this wide


def _integral(a):
  s = np.zeros((a.shape[0] + 1, a.shape[1] + 1), dtype=np.float64)
  np.cumsum(np.cumsum(a, axis=0, dtype=np.float64), axis=1, out=s[1:, 1:])  # float64: sums of a 400 x 250 window
  return s


def _box(ii, y0, x0, h, w, ny, nx):
  """sum over [y0+i, y0+i+h) x [x0+j, x0+j+w) for i < ny, j < nx (y0, x0 may place the box off the grid origin)"""
  return ii[y0 + h:y0 + h + ny, x0 + w:x0 + w + nx] - ii[y0:y0 + ny, x0 + w:x0 + w + nx] - \
         ii[y0 + h:y0 + h + ny, x0:x0 + nx] + ii[y0:y0 + ny, x0:x0 + nx]


def find_plate(y, roi, w0, uv=None):
  """y: full luma plane (H x W uint8); roi: (x0, y0, x1, y1); w0: expected plate width in px.
  Returns (x0, y0, x1, y1, score) in full-image px or None"""
  X0, Y0, X1, Y1 = [int(v) for v in roi]
  X0, Y0 = max(X0, 0), max(Y0, 0)
  X1, Y1 = min(X1, y.shape[1]), min(Y1, y.shape[0])
  k = max(int(round(w0 / TARGET_W)), 1)
  h_, w_ = (Y1 - Y0) // k, (X1 - X0) // k
  if h_ < 24 or w_ < 48:
    return None
  sub = y[Y0:Y0 + h_ * k, X0:X0 + w_ * k]
  # k x k block sums as strided integer adds (the device is ~10 x slower than a PC: no float image of the full window)
  acc = np.zeros((h_, w_), dtype=np.uint16)
  for i in range(k):
    for j in range(k):
      acc += sub[i::k, j::k]
  img = acc.astype(np.float32) / (k * k)
  gx = np.zeros_like(img)
  gx[:, 1:-1] = np.abs(img[:, 2:] - img[:, :-2])
  ii_g = _integral(gx)
  best = None
  for s in SCALES:
    w = max(int(round(w0 * s / k)), 12)
    h = max(int(round(w * PLATE_H / PLATE_W)), 5)
    m = max(int(round(0.30 * h)), 2)               # ring around the plate
    ny, nx = h_ - h - 2 * m, w_ - w - 2 * m
    if ny < 1 or nx < 1:
      continue
    tx, ty = max(int(round(0.07 * w)), 1), max(int(round(0.20 * h)), 1)     # text block inside the plate
    tw, th = w - 2 * tx, h - 2 * ty
    a_out = (h + 2 * m) * (w + 2 * m) - h * w
    g_out = (_box(ii_g, 0, 0, h + 2 * m, w + 2 * m, ny, nx) - _box(ii_g, m, m, h, w, ny, nx)) / a_out
    # the upper and the lower half of the text block both carry strokes (a grille bar or a bumper edge does not)
    g_top = _box(ii_g, m + ty, m + tx, th // 2, tw, ny, nx) / max((th // 2) * tw, 1)
    g_bot = _box(ii_g, m + ty + th // 2, m + tx, th - th // 2, tw, ny, nx) / max((th - th // 2) * tw, 1)
    # ... and so do its left and right halves
    g_l = _box(ii_g, m + ty, m + tx, th, tw // 2, ny, nx) / max(th * (tw // 2), 1)
    g_r = _box(ii_g, m + ty, m + tx + tw // 2, th, tw - tw // 2, ny, nx) / max(th * (tw - tw // 2), 1)
    g_min = np.minimum(np.minimum(g_top, g_bot), np.minimum(g_l, g_r))
    score = g_min - 0.7 * g_out
    j = int(np.argmax(score))
    iy, ix = divmod(j, nx)
    sc = float(score[iy, ix])
    if best is None or sc > best[0]:
      best = (sc, ix + m, iy + m, w, h, float(g_min[iy, ix]), float(g_out[iy, ix]))
  if best is None:
    return None
  sc, bx, by, w, h, g_min, g_out = best
  if g_min < 4.0 or g_min < 1.4 * g_out:
    return None
  x0, y0, x1, y1 = X0 + bx * k, Y0 + by * k, X0 + (bx + w) * k, Y0 + (by + h) * k
  r = _refine(y, x0, y0, x1, y1, w0, uv)
  return None if r is None else r + (sc,)


TEXT_W = 1.00      # width of the gradient run (strokes + the plate edges) / plate width (calibrated on 25 stops: 1.003)
TEXT_GAP = 0.14    # widest gap inside the text (between the two groups), of the plate width


def _run(e, c, thr, gap):
  """the run of e >= thr around index c, bridging gaps up to gap samples: (lo, hi) or None"""
  on = e >= thr
  n = len(e)
  if not on[max(c - gap, 0):c + gap + 1].any():
    return None
  lo = hi = int(np.clip(c, 0, n - 1))
  miss, i = 0, lo
  while i > 0 and miss <= gap:
    i -= 1
    if on[i]:
      lo, miss = i, 0
    else:
      miss += 1
  miss, i = 0, hi
  while i < n - 1 and miss <= gap:
    i += 1
    if on[i]:
      hi, miss = i, 0
    else:
      miss += 1
  return lo, hi


def _refine(y, x0, y0, x1, y1, w0=None, uv=None):
  """the plate from the extent of its text (full resolution): strokes give horizontal gradient; the text block is
  TEXT_W of the plate wide. Returns the box, or None when what was found is not a plate of the expected size"""
  w, h = x1 - x0, y1 - y0
  w0 = w0 or w
  mx = int(0.45 * w0)
  ax0, ax1 = max(x0 - mx, 1), min(x1 + mx, y.shape[1] - 1)
  ay0, ay1 = max(y0 - int(0.5 * h), 0), min(y1 + int(0.5 * h), y.shape[0])
  p = y[ay0:ay1, ax0 - 1:ax1 + 1].astype(np.float32)
  g = np.abs(p[:, 2:] - p[:, :-2])
  # rows of the text: horizontal-gradient energy per row
  er = g[:, max(x0 - ax0, 0):x1 - ax0].mean(axis=1)
  rr = _run(er, (y0 + y1) // 2 - ay0, 0.45 * float(np.percentile(er, 90)), max(int(0.06 * h), 1))
  if rr is None:
    return None
  t, b = rr
  th = b - t + 1
  # columns of the text, over those rows
  ec = g[t:b + 1].mean(axis=0)
  k = max(int(w0 / 60), 1)
  ec = np.convolve(ec, np.ones(k) / k, mode='same')
  cr = _run(ec, (x0 + x1) // 2 - ax0, 0.30 * float(np.percentile(ec, 90)), int(TEXT_GAP * w0))
  if cr is None:
    return None
  l, r = cr
  tw = r - l + 1
  pw = tw / TEXT_W
  ph = pw * PLATE_H / PLATE_W
  if not (0.85 * w0 <= pw <= 1.18 * w0) or not (0.30 * ph <= th <= 0.85 * ph):
    return None
  cx, cy = ax0 + 0.5 * (l + r), ay0 + 0.5 * (t + b)
  bx0, by0, bx1, by1 = int(round(cx - pw / 2)), int(round(cy - ph / 2)), int(round(cx + pw / 2)), int(round(cy + ph / 2))
  if uv is not None:
    # a plate is white: little colour (a tail light, a painted body or a sign is not)
    u, v = uv
    sl = (slice(max(by0 // 2, 0), max(by1 // 2, 1)), slice(max(bx0 // 2, 0), max(bx1 // 2, 1)))
    if u[sl].size == 0 or float(np.abs(u[sl].astype(np.float32) - 128).mean() + np.abs(v[sl].astype(np.float32) - 128).mean()) > PLATE_CHROMA:
      return None
  return bx0, by0, bx1, by1


PLATE_CHROMA = 22.0


PF_MAX_D = 12.0          # m (radar dRel): the plate is looked for behind a lead closer than this ...
PF_MIN_D = 0.8           # m: ... and farther than this
PF_CAM_OFFSET = 1.75     # m: camera to what the radar measures to, plus the plate's recess (fitted on the recorded stops)
PF_HALF_W = 1.1          # m to each side of the lead's centre
PF_TOP = 1.35            # m above the road
PF_PERIOD = 0.25         # s between frames
PF_KEEP = 0.5            # s the box stays after the last find
PF_SAME = 0.35           # of the plate width: a find this close to the last one is the same plate
PF_SMOOTH = 0.6          # weight of the new find
PF_INPUT_MAX_AGE = 0.5   # s: no fresh lead / calibration -> idle


class PlateFinder:
  """finds the lead's plate in the road camera stream in its own thread; box() gives the last confirmed one as four
  points in the calibrated car frame (the frame the UI projects from)"""

  def __init__(self):
    self._inputs = None      # (t, d_rel, y_rel, cam_height, view_from_calib, intrinsics)
    self._box = None         # (t, 4 x 3 points: top-left, top-right, bottom-right, bottom-left)
    self._track = None       # (cx, cy, w, hits, corners px)
    self._thread = None
    self._client = None

  def set_inputs(self, d_rel, y_rel, cam_height, view_from_calib, intrinsics):
    self._inputs = (time.monotonic(), float(d_rel), float(y_rel), float(cam_height), view_from_calib, intrinsics)
    if self._thread is None:
      self._thread = threading.Thread(target=self._run, name="plate_finder", daemon=True)
      self._thread.start()

  def box(self):
    b = self._box
    return b[1] if b is not None and time.monotonic() - b[0] < PF_KEEP else None

  def _run(self):
    while True:
      t0 = time.monotonic()
      try:
        self._step()
      except Exception:          # never take the UI down for an overlay
        self._track = None
        time.sleep(1.0)
      time.sleep(max(PF_PERIOD - (time.monotonic() - t0), 0.02))

  def _frame(self):
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    if self._client is None:
      self._client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, conflate=True)
    if not self._client.is_connected() and not self._client.connect(False):
      return None
    return self._client.recv(timeout_ms=100)

  def _step(self):
    inp = self._inputs
    if inp is None or time.monotonic() - inp[0] > PF_INPUT_MAX_AGE or not (PF_MIN_D < inp[1] < PF_MAX_D):
      self._track = None
      time.sleep(0.3)
      return
    buf = self._frame()
    if buf is None:
      return
    _, d_rel, y_rel, cam_h, view_from_calib, K = inp
    h, w, stride = buf.height, buf.width, buf.stride
    data = np.frombuffer(buf.data, dtype=np.uint8) if not isinstance(buf.data, np.ndarray) else buf.data
    y = data[:buf.uv_offset].reshape(-1, stride)[:h, :w]
    uv = data[buf.uv_offset:buf.uv_offset + (h // 2) * stride].reshape(-1, stride)[:h // 2, :w]
    found = locate(y, (uv[:, 0::2], uv[:, 1::2]), d_rel, y_rel, cam_h, view_from_calib, K)
    self._update(found, view_from_calib, K)

  def _update(self, found, view_from_calib, K):
    if found is None:
      if self._box is None or time.monotonic() - self._box[0] > PF_KEEP:
        self._track = None
      return
    x0, y0, x1, y1 = found[:4]
    cx, cy, w = 0.5 * (x0 + x1), 0.5 * (y0 + y1), float(x1 - x0)
    c = np.array([x0, y0, x1, y1], dtype=np.float64)
    tr = self._track
    if tr is not None and abs(cx - tr[0]) < PF_SAME * w and abs(cy - tr[1]) < PF_SAME * w and 0.8 < w / tr[2] < 1.25:
      c = PF_SMOOTH * c + (1.0 - PF_SMOOTH) * tr[4]
      hits = tr[3] + 1
    else:
      hits = 1
    self._track = (cx, cy, w, hits, c)
    if hits >= 2:
      self._box = (time.monotonic(), box_in_car_frame(c, view_from_calib, K))


def locate(y, uv, d_rel, y_rel, cam_height, view_from_calib, K):
  """the lead's plate in a road camera frame: (x0, y0, x1, y1, score) px or None"""
  d = d_rel + PF_CAM_OFFSET
  f = float(K[0, 0])
  w0 = f * PLATE_W / d
  T = K @ view_from_calib
  # the search window: the lead's rear, from the road up (car frame: x ahead, y right, z down; camera at the origin)
  pts = np.array([[d, -y_rel - PF_HALF_W, cam_height], [d, -y_rel + PF_HALF_W, cam_height],
                  [d, -y_rel - PF_HALF_W, cam_height - PF_TOP], [d, -y_rel + PF_HALF_W, cam_height - PF_TOP]]).T
  p = T @ pts
  if np.any(p[2] <= 0.1):
    return None
  u, v = p[0] / p[2], p[1] / p[2]
  roi = (u.min(), v.min(), u.max(), v.max())
  if roi[2] - roi[0] < 2.0 * w0 or roi[2] < 0 or roi[0] > y.shape[1] or roi[3] < 0 or roi[1] > y.shape[0]:
    return None
  return find_plate(y, roi, w0, uv)


def box_in_car_frame(c, view_from_calib, K):
  """the plate's four corners (top-left, top-right, bottom-right, bottom-left) in the calibrated car frame, at the
  distance its own width gives"""
  x0, y0, x1, y1 = [float(v) for v in c]
  depth = float(K[0, 0]) * PLATE_W / max(x1 - x0, 1.0)
  px = np.array([[x0, y0, 1.0], [x1, y0, 1.0], [x1, y1, 1.0], [x0, y1, 1.0]]).T
  rays = np.linalg.inv(K) @ px
  return (view_from_calib.T @ (rays * depth)).T


class PlateOverlay:
  """draws the found plate: its rectangle, and a line from each corner of the view to the matching corner"""

  def __init__(self):
    self.finder = PlateFinder()

  def render(self, rect, view):
    try:
      self._render(rect, view)
    except Exception:            # an overlay must never take the road view down
      pass

  def _render(self, rect, view):
    import pyray as rl
    from openpilot.selfdrive.ui.ui_state import ui_state
    sm = ui_state.sm
    cam = view.device_camera
    if cam is None or not sm.valid['radarState']:
      return
    lead = sm['radarState'].leadOne
    if lead.status:
      self.finder.set_inputs(lead.dRel, lead.yRel, view.model_renderer._path_offset_z, view.view_from_calib, cam.fcam.intrinsics)
    pts = self.finder.box()
    if pts is None or not lead.status:
      return
    p = view.model_renderer._car_space_transform @ pts.T
    if np.any(np.abs(p[2]) < 1e-6):
      return
    xs, ys = p[0] / p[2], p[1] / p[2]
    if xs.min() < rect.x or xs.max() > rect.x + rect.width or ys.min() < rect.y or ys.max() > rect.y + rect.height:
      return
    corners = [rl.Vector2(float(xs[i]), float(ys[i])) for i in range(4)]
    view_corners = [rl.Vector2(rect.x, rect.y), rl.Vector2(rect.x + rect.width, rect.y),
                    rl.Vector2(rect.x + rect.width, rect.y + rect.height), rl.Vector2(rect.x, rect.y + rect.height)]
    for a, b in zip(view_corners, corners, strict=True):
      rl.draw_line_ex(a, b, PO_LINE_W, rl.Color(*PO_LINE_COLOR))
    for i in range(4):
      rl.draw_line_ex(corners[i], corners[(i + 1) % 4], PO_BOX_W, rl.Color(*PO_BOX_COLOR))


PO_LINE_W = 2.0
PO_BOX_W = 4.0
PO_LINE_COLOR = (255, 255, 255, 150)
PO_BOX_COLOR = (0, 255, 120, 255)
