import subprocess

import pytest

import dragonpilot.system.curve_learn_sync as S


def sh(*a, cwd=None):
  return subprocess.check_output(a, cwd=cwd, text=True, stderr=subprocess.STDOUT).strip()


@pytest.fixture
def env(tmp_path):
  """a 'GitHub' (bare repo), the device's software clone of it, the learned file and the key next to them"""
  hub, dev = tmp_path / "hub.git", tmp_path / "openpilot"
  sh("git", "init", "-q", "--bare", str(hub))
  sh("git", "init", "-q", str(dev))
  sh("git", "-C", str(dev), "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m", "software")
  sh("git", "-C", str(dev), "remote", "add", "origin", str(hub))
  sh("git", "-C", str(dev), "push", "-q", "origin", "HEAD:refs/heads/dpcr")
  key = tmp_path / "key"; key.write_text("k")
  learn = tmp_path / "learn.json"
  return hub, dev, learn, key


def log(hub):
  return sh("git", "-C", str(hub), "log", "--format=%s", S.SYNC_BRANCH).splitlines()


def test_learned_file_is_pushed_to_its_own_branch_on_top_of_the_last_one(env):
  hub, dev, learn, key = env
  s = S.LearnSync(str(learn), str(key), str(dev))
  learn.write_text('{"a": 1}')
  head = sh("git", "-C", str(dev), "rev-parse", "HEAD"); status = sh("git", "-C", str(dev), "status", "--porcelain")
  assert s.step(0.0) == "pushed"
  assert sh("git", "-C", str(hub), "show", f"{S.SYNC_BRANCH}:{S.SYNC_NAME}") == '{"a": 1}'
  assert s.step(30.0) == "same"
  learn.write_text('{"a": 2}')
  assert s.step(60.0) == "pushed"
  assert sh("git", "-C", str(hub), "show", f"{S.SYNC_BRANCH}:{S.SYNC_NAME}") == '{"a": 2}'
  assert len(log(hub)) == 2                                                          # history kept: nothing forced
  # the software branch and the working tree are as they were
  assert sh("git", "-C", str(dev), "rev-parse", "HEAD") == head and sh("git", "-C", str(dev), "status", "--porcelain") == status
  assert sh("git", "-C", str(hub), "rev-parse", "dpcr") == head


def test_a_fresh_process_does_not_push_what_github_already_has(env):
  hub, dev, learn, key = env
  learn.write_text('{"a": 1}')
  assert S.LearnSync(str(learn), str(key), str(dev)).step(0.0) == "pushed"
  assert S.LearnSync(str(learn), str(key), str(dev)).step(0.0) == "same"
  assert len(log(hub)) == 1


def test_without_the_key_nothing_is_pushed_and_it_is_tried_again_later(env):
  hub, dev, learn, key = env
  key.unlink(); learn.write_text('{"a": 1}')
  s = S.LearnSync(str(learn), str(key), str(dev))
  assert s.step(0.0) == "no key" and s.step(30.0) == "waiting"
  key.write_text("k")
  assert s.step(S.RETRY + 1.0) == "pushed"


def test_a_reset_device_gets_the_last_pushed_values_back(env):
  hub, dev, learn, key = env
  learn.write_text('{"a": 7}')
  S.LearnSync(str(learn), str(key), str(dev)).step(0.0)
  learn.unlink(); key.unlink()                                 # the device was reset: no file, no key
  s = S.LearnSync(str(learn), str(key), str(dev))
  assert s.step(0.0) == "restored" and learn.read_text() == '{"a": 7}'
  assert s.step(30.0) == "same"


def test_nothing_to_restore_and_no_network(env):
  hub, dev, learn, key = env
  s = S.LearnSync(str(learn), str(key), str(dev))
  assert s.step(0.0) == "nothing anywhere" and not learn.exists()
  sh("git", "-C", str(dev), "remote", "set-url", "origin", str(hub) + "-gone")
  learn.write_text('{"a": 1}')
  s = S.LearnSync(str(learn), str(key), str(dev))
  assert s.step(0.0) == "failed" and s.step(10.0) == "waiting"   # no crash, tried again after RETRY
