# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
dragonpilot (dpcr): what the curve outward bias has learned (curve_bias_learn.py) goes back to GitHub.

Owner 2026-11-25: "kept on the C4 it is lost when the device is set up again - I want the learned data pushed back to
GitHub" (he is the only dpcr user). The learned table lives in /data/dp_curve_bias_learn.json: outside the software
folder, so an update or a re-download keeps it, a device reset or another device does not.

While the car is off (this process only runs offroad) and the file differs from what GitHub has, it is committed to
its own branch SYNC_BRANCH as SYNC_NAME - one small file, each commit on top of the last one, never forced - and
pushed with the device's own key KEY_FILE (a deploy key of this repository only; without the key, or without a
network, nothing happens and it is tried again later). The software branch and the working tree are never touched:
the commit is built with git plumbing (hash-object / mktree / commit-tree) in the software's own .git.
The other way round: when the file is missing (a reset device), it is fetched from that branch over the public https
remote - no key needed - so learning starts from the last pushed values instead of from zero. A file on the device
always wins over the branch.
"""
import hashlib
import os
import subprocess
import time

from openpilot.common.basedir import BASEDIR
from openpilot.common.swaglog import cloudlog
from dragonpilot.selfdrive.controls.lib.curve_bias_learn import LEARN_FILE

SYNC_BRANCH = "dpcr-learn"
SYNC_NAME = "curve_bias_learn.json"
KEY_FILE = "/data/dp_learn_key"
PERIOD = 30.0          # s between looks at the file
RETRY = 300.0          # s after a failed fetch / push
GIT_TIMEOUT = 60.0     # s


def git(*args, repo=BASEDIR, env=None, stdin=None, timeout=GIT_TIMEOUT) -> str:
  e = dict(os.environ, **(env or {}))
  return subprocess.check_output(["git", "-C", repo, *args], text=True, timeout=timeout, stderr=subprocess.STDOUT,
                                 input=stdin, env=e).strip()


def digest(data: str) -> str:
  return hashlib.sha1(data.encode()).hexdigest()


class LearnSync:
  def __init__(self, learn_file=LEARN_FILE, key_file=KEY_FILE, repo=BASEDIR):
    self.learn_file, self.key_file, self.repo = learn_file, key_file, repo
    self.synced = None       # digest of the file content GitHub is known to have
    self.wait_until = 0.0

  def _git(self, *args, **kw):
    return git(*args, repo=self.repo, **kw)

  def remote(self) -> tuple[str | None, str | None]:
    """(commit, file content) of the branch on GitHub, (None, None) when it does not exist yet. Raises without a network"""
    out = self._git("ls-remote", "origin", f"refs/heads/{SYNC_BRANCH}")
    if not out:
      return None, None
    self._git("fetch", "--quiet", "origin", f"+refs/heads/{SYNC_BRANCH}:refs/remotes/origin/{SYNC_BRANCH}")
    commit = self._git("rev-parse", f"refs/remotes/origin/{SYNC_BRANCH}")
    try:
      return commit, self._git("show", f"{commit}:{SYNC_NAME}")
    except subprocess.CalledProcessError:
      return commit, None

  def restore(self, content: str):
    tmp = self.learn_file + ".tmp"
    with open(tmp, "w") as f:
      f.write(content)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, self.learn_file)

  def push(self, parent: str | None, summary: str = ""):
    blob = self._git("hash-object", "-w", self.learn_file)
    tree = self._git("mktree", stdin=f"100644 blob {blob}\t{SYNC_NAME}\n")
    who = {"GIT_AUTHOR_NAME": "C4 curve learner", "GIT_AUTHOR_EMAIL": "c4@localhost",
           "GIT_COMMITTER_NAME": "C4 curve learner", "GIT_COMMITTER_EMAIL": "c4@localhost"}
    msg = "dpcr-learn: curve outward bias learned on the car\n\n" + summary
    commit = self._git("commit-tree", tree, *(["-p", parent] if parent else []), "-m", msg, env=who)
    ssh = f"ssh -i {self.key_file} -o IdentitiesOnly=yes -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15"
    url = self._git("remote", "get-url", "--push", "origin")
    self._git("push", "--quiet", url, f"{commit}:refs/heads/{SYNC_BRANCH}", env={"GIT_SSH_COMMAND": ssh})
    self._git("update-ref", f"refs/remotes/origin/{SYNC_BRANCH}", commit)
    return commit

  def step(self, now: float) -> str:
    """one look; returns what was done (for the log and the tests)"""
    if now < self.wait_until:
      return "waiting"
    local = None
    if os.path.isfile(self.learn_file):
      with open(self.learn_file) as f:
        local = f.read()
      if digest(local) == self.synced:
        return "same"
    try:
      parent, there = self.remote()
      if local is None:
        if there is None:
          self.wait_until = now + RETRY
          return "nothing anywhere"
        self.restore(there)
        self.synced = digest(there)
        cloudlog.event("curve_learn_sync", action="restored", commit=parent)
        return "restored"
      if there == local:
        self.synced = digest(local)
        return "same"
      if not os.path.isfile(self.key_file):
        self.wait_until = now + RETRY
        return "no key"
      commit = self.push(parent, local)
      self.synced = digest(local)
      cloudlog.event("curve_learn_sync", action="pushed", commit=commit)
      return "pushed"
    except Exception as e:
      self.wait_until = now + RETRY
      cloudlog.event("curve_learn_sync", action="failed", error=str(getattr(e, 'output', e))[-300:])
      return "failed"


def main():
  sync = LearnSync()
  while True:
    sync.step(time.monotonic())
    time.sleep(PERIOD)


if __name__ == "__main__":
  main()
