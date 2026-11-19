"""`scripts/ss.sh` の試験。偽の docker を `SS_DOCKER` で渡す。

本物の docker は呼ばないので、コンテナの中の `scripts/test.sh` で回る。
偽の docker の答え・状態は環境変数で決め、受けた引数は1回1行で記録ファイルに足す。
モデルの在りかは `SS_MODEL_PATH` で差し替える。
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
SCRIPT = REPO_ROOT / "scripts" / "ss.sh"

STATUS_IDLE = "状態: interface_version=3  phase=IDLE  last_reject_reason=-"
STATUS_SCANNING = "状態: interface_version=3  phase=SCANNING  last_reject_reason=-"

FAKE_DOCKER = """#!/usr/bin/env bash
set -u
printf '%s ' "$@" >> "$SS_FAKE_DOCKER_LOG"
printf '\\n' >> "$SS_FAKE_DOCKER_LOG"
running_value() {
  if [ -f "$SS_FAKE_STATE_FILE" ]; then
    cat "$SS_FAKE_STATE_FILE"
  else
    printf '%s' "${SS_FAKE_RUNNING:-0}"
  fi
}
cmd="${1:-}"
case "$cmd" in
  ps)
    has_all=0
    for a in "$@"; do
      if [ "$a" = "-aq" ] || [ "$a" = "--all" ]; then has_all=1; fi
    done
    if [ "$has_all" = "1" ]; then
      if [ "${SS_FAKE_EXISTS:-0}" = "1" ]; then echo fake-container-id; fi
    else
      if [ "$(running_value)" = "1" ]; then echo fake-container-id; fi
    fi
    exit 0
    ;;
  image)
    if [ "${SS_FAKE_IMAGE:-1}" = "1" ]; then exit 0; else exit 1; fi
    ;;
  exec)
    has_status=0
    for a in "$@"; do
      if [ "$a" = "--status" ]; then has_status=1; fi
    done
    if [ "$has_status" = "1" ]; then
      count=0
      if [ -f "$SS_FAKE_STATUS_COUNT_FILE" ]; then
        count="$(cat "$SS_FAKE_STATUS_COUNT_FILE")"
      fi
      count=$((count + 1))
      printf '%s' "$count" > "$SS_FAKE_STATUS_COUNT_FILE"
      if [ "$count" -le "${SS_FAKE_STATUS_FAIL_TIMES:-0}" ]; then
        exit "${SS_FAKE_STATUS_CODE:-3}"
      fi
      printf '%s\\n' "${SS_FAKE_STATUS_OUTPUT:-状態: interface_version=3  phase=IDLE  last_reject_reason=-}"
      exit 0
    fi
    exit "${SS_FAKE_EXEC_CODE:-0}"
    ;;
  run)
    has_d=0
    for a in "$@"; do
      if [ "$a" = "-d" ]; then has_d=1; fi
    done
    if [ "$has_d" = "1" ]; then
      if [ "${SS_FAKE_DIE_AFTER_RUN:-0}" = "1" ]; then
        printf '0' > "$SS_FAKE_STATE_FILE"
      else
        printf '1' > "$SS_FAKE_STATE_FILE"
      fi
    fi
    exit "${SS_FAKE_RUN_CODE:-0}"
    ;;
  logs)
    printf '%s\\n' "fake-log-line"
    exit 0
    ;;
  stop)
    printf '0' > "$SS_FAKE_STATE_FILE"
    exit 0
    ;;
  build|rm)
    exit 0
    ;;
  *)
    exit 0
    ;;
esac
"""


def _fake_docker(tmp_path: Path) -> Path:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    fake = bin_dir / "fake-docker"
    fake.write_text(FAKE_DOCKER, encoding="utf-8")
    fake.chmod(0o755)
    return fake


def _env(tmp_path: Path, **overrides):
    model = tmp_path / "lug_v3_s.pt"
    model.write_bytes(b"weights")
    env = os.environ.copy()
    env["SS_DOCKER"] = str(_fake_docker(tmp_path))
    env["SS_FAKE_DOCKER_LOG"] = str(tmp_path / "docker.log")
    env["SS_FAKE_STATUS_COUNT_FILE"] = str(tmp_path / "status.count")
    env["SS_FAKE_STATE_FILE"] = str(tmp_path / "state")
    env["SS_MODEL_PATH"] = str(model)
    env["SS_START_WAIT_S"] = "5"
    env["SS_FAKE_RUNNING"] = "0"
    env["SS_FAKE_EXISTS"] = "0"
    env["SS_FAKE_IMAGE"] = "1"
    env["SS_FAKE_STATUS_FAIL_TIMES"] = "0"
    env["SS_FAKE_STATUS_OUTPUT"] = STATUS_IDLE
    env["SS_FAKE_EXEC_CODE"] = "0"
    env["SS_FAKE_RUN_CODE"] = "0"
    for key in list(env):
        if key.startswith("SS_FAKE_") and key not in env:
            pass
    env.update(overrides)
    return env


def _run(tmp_path, args, cwd=None, **overrides):
    env = _env(tmp_path, **overrides)
    log = tmp_path / "docker.log"
    if log.exists():
        log.unlink()
    count = tmp_path / "status.count"
    if count.exists():
        count.unlink()
    state = tmp_path / "state"
    if state.exists():
        state.unlink()
    result = subprocess.run(
        [str(SCRIPT), *args],
        cwd=str(cwd or tmp_path),
        env=env,
        capture_output=True,
        text=True,
    )
    lines = log.read_text(encoding="utf-8").splitlines() if log.exists() else []
    return result, lines


def _find(lines, *needles):
    for line in lines:
        if all(n in line for n in needles):
            return line
    raise AssertionError("行が無い: %r\n%s" % (needles, "\n".join(lines)))


def _find_all(lines, *needles):
    return [line for line in lines if all(n in line for n in needles)]


def _has(lines, *needles):
    return any(all(n in line for n in needles) for line in lines)


def test_start_runs_node_with_params_file(tmp_path) -> None:
    result, lines = _run(tmp_path, ["start"])
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "ss-system:dev")
    assert "-d" in run.split()
    assert "--name ss-system" in run
    assert "--network host" in run
    assert "--ipc host" in run
    assert f"{REPO_ROOT}:/ws" in run
    assert "locator_node" in run
    assert "locator_node --ros-args --params-file" in run
    ros_at = run.index("--ros-args")
    params_at = run.index("--params-file")
    assert ros_at < params_at
    assert "--params-file ss_ws/src/ss_locator/config/locator.yaml" in run
    assert "--rm" not in run


def test_start_overrides(tmp_path) -> None:
    result, lines = _run(
        tmp_path,
        ["start", "--camera-host", "192.168.5.129", "--lift-host", "10.0.0.2", "--fakes"],
    )
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "ss-system:dev")
    ros_at = run.index("--ros-args")
    params_at = run.index("--params-file")
    assert ros_at < params_at
    for override in (
        "camera_host:=192.168.5.129",
        "lift_host:=10.0.0.2",
        "use_fakes:=true",
    ):
        assert params_at < run.index(override)
        assert ros_at < run.index(override)

    result, lines = _run(tmp_path, ["start"])
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "ss-system:dev")
    assert "camera_host:=" not in run
    assert "lift_host:=" not in run
    assert "use_fakes:=" not in run


def test_start_refuses_when_running(tmp_path) -> None:
    result, lines = _run(tmp_path, ["start"], SS_FAKE_RUNNING="1")
    assert result.returncode == 1
    assert not _has(lines, "run")


def test_start_removes_stale(tmp_path) -> None:
    result, lines = _run(tmp_path, ["start"], SS_FAKE_EXISTS="1")
    assert result.returncode == 0, result.stderr
    rm_at = next(i for i, line in enumerate(lines) if line.startswith("rm "))
    run_at = next(i for i, line in enumerate(lines) if line.startswith("run "))
    assert rm_at < run_at


def test_start_needs_model(tmp_path) -> None:
    missing = str(tmp_path / "nope.pt")
    result, lines = _run(tmp_path, ["start"], SS_MODEL_PATH=missing)
    assert result.returncode == 1
    assert not _has(lines, "run")

    result, lines = _run(
        tmp_path, ["start", "--fakes"], SS_MODEL_PATH=missing
    )
    assert result.returncode == 0, result.stderr
    assert _has(lines, "run", "ss-system:dev")


def test_start_waits_for_status(tmp_path) -> None:
    result, lines = _run(tmp_path, ["start"], SS_FAKE_STATUS_FAIL_TIMES="1")
    assert result.returncode == 0, result.stderr
    assert STATUS_IDLE in result.stdout

    result, lines = _run(
        tmp_path,
        ["start"],
        SS_FAKE_STATUS_FAIL_TIMES="99",
        SS_START_WAIT_S="2",
    )
    assert result.returncode == 1
    assert _has(lines, "logs", "--tail 50")
    assert "fake-log-line" in result.stdout


def test_start_exits_early_when_container_dies(tmp_path) -> None:
    import time

    started = time.monotonic()
    result, lines = _run(
        tmp_path,
        ["start"],
        SS_FAKE_STATUS_FAIL_TIMES="99",
        SS_FAKE_DIE_AFTER_RUN="1",
        SS_START_WAIT_S="30",
    )
    elapsed = time.monotonic() - started
    assert result.returncode == 1
    assert elapsed < 10.0
    assert "コンテナが落ちました" in result.stderr
    assert _has(lines, "logs", "--tail 50")


def test_start_builds_image_when_missing(tmp_path) -> None:
    result, lines = _run(tmp_path, ["start"], SS_FAKE_IMAGE="0")
    assert result.returncode == 0, result.stderr
    assert any(line.startswith("build ") for line in lines)

    result, lines = _run(tmp_path, ["start"], SS_FAKE_IMAGE="1")
    assert result.returncode == 0, result.stderr
    assert not any(line.startswith("build ") for line in lines)


def test_locate_adds_outputs(tmp_path) -> None:
    result, lines = _run(tmp_path, ["locate", "--yes"], SS_FAKE_RUNNING="1")
    assert result.returncode == 0, result.stderr
    execs = _find_all(lines, "exec", "locate_cli")
    assert execs, lines
    line = execs[0]
    assert "--yes" in line
    assert re.search(r"--plot out/\d{8}-\d{6}\.png", line), line
    assert re.search(r"--json out/\d{8}-\d{6}\.json", line), line

    result, lines = _run(
        tmp_path, ["locate", "--yes", "--plot", "x.png"], SS_FAKE_RUNNING="1"
    )
    assert result.returncode == 0, result.stderr
    line = _find(lines, "exec", "locate_cli")
    assert "--plot x.png" in line
    assert "--plot out/" not in line
    assert re.search(r"--json out/\d{8}-\d{6}\.json", line), line


def test_locate_passes_exit_code(tmp_path) -> None:
    result, lines = _run(
        tmp_path, ["locate", "--yes"], SS_FAKE_RUNNING="1", SS_FAKE_EXEC_CODE="4"
    )
    assert result.returncode == 4
    assert _has(lines, "chown")


def test_locate_not_running(tmp_path) -> None:
    result, lines = _run(tmp_path, ["locate", "--yes"], SS_FAKE_RUNNING="0")
    assert result.returncode == 1
    assert not _has(lines, "exec")


def test_stop_refuses_while_busy(tmp_path) -> None:
    result, lines = _run(
        tmp_path, ["stop"], SS_FAKE_RUNNING="1", SS_FAKE_STATUS_OUTPUT=STATUS_SCANNING
    )
    assert result.returncode == 4
    assert not _has(lines, "stop")

    result, lines = _run(
        tmp_path,
        ["stop", "--force"],
        SS_FAKE_RUNNING="1",
        SS_FAKE_STATUS_OUTPUT=STATUS_SCANNING,
    )
    assert result.returncode == 0, result.stderr
    assert _has(lines, "stop", "ss-system")

    result, lines = _run(
        tmp_path, ["stop"], SS_FAKE_RUNNING="1", SS_FAKE_STATUS_OUTPUT=STATUS_IDLE
    )
    assert result.returncode == 0, result.stderr
    assert _has(lines, "stop", "ss-system")
    assert _has(lines, "rm")


def test_calib_refuses_while_running(tmp_path) -> None:
    result, lines = _run(
        tmp_path, ["calib", "--lugs", "1,0;1,1"], SS_FAKE_RUNNING="1"
    )
    assert result.returncode == 1
    assert not _has(lines, "run")

    result, lines = _run(
        tmp_path, ["calib", "--from", "somewhere"], SS_FAKE_RUNNING="1"
    )
    assert result.returncode == 0, result.stderr
    assert _has(lines, "run", "mount_calib")


def test_calib_adds_params_and_out(tmp_path) -> None:
    result, lines = _run(
        tmp_path, ["calib", "--lugs", "1,0;1,1", "--poses", "0:20"]
    )
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "mount_calib")
    assert "--params-file ss_ws/src/ss_locator/config/locator.yaml" in run
    assert re.search(r"--out out/calib-\d{8}-\d{6}", run), run
    assert "--lugs 1,0;1,1" in run
    assert "--poses 0:20" in run

    result, lines = _run(
        tmp_path, ["calib", "--params-file", "mine.yaml", "--out", "mine"]
    )
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "mount_calib")
    assert "--params-file mine.yaml" in run
    assert "--out mine" in run
    assert "ss_ws/src/ss_locator/config/locator.yaml" not in run
    assert "--out out/calib-" not in run


def test_exec_sources_and_cwd(tmp_path) -> None:
    result, lines = _run(tmp_path, ["status"], SS_FAKE_RUNNING="1")
    assert result.returncode == 0, result.stderr
    line = _find(lines, "exec", "locate_cli")
    assert "/opt/ros/humble/setup.bash" in line
    assert "/ws/ss_ws/install/setup.bash" in line
    assert "cd /ws" in line

    result, lines = _run(tmp_path, ["locate", "--yes"], SS_FAKE_RUNNING="1")
    assert result.returncode == 0, result.stderr
    line = _find(lines, "exec", "locate_cli")
    assert "/opt/ros/humble/setup.bash" in line
    assert "/ws/ss_ws/install/setup.bash" in line
    assert "cd /ws" in line

    result, lines = _run(tmp_path, ["start"])
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "ss-system:dev")
    assert "PYTHONUNBUFFERED=1" in run


def test_calib_no_build(tmp_path) -> None:
    result, lines = _run(tmp_path, ["calib", "--lugs", "1,0;1,1"])
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "mount_calib")
    assert "colcon" not in run
    assert "PYTHONPATH=ss_ws/src/ss_locator" in run
    assert "ss_locator.mount_calib" in run


def test_start_record(tmp_path) -> None:
    result, lines = _run(tmp_path, ["start", "--record"])
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "ss-system:dev")
    params_at = run.index("--params-file")
    assert params_at < run.index("shot_log_dir:=out/shots-")
    assert re.search(r"shot_log_dir:=out/shots-\d{8}-\d{6}", run)
    assert "shot_save_images" not in run
    assert "撮影の記録: out/shots-" in result.stdout

    result, lines = _run(tmp_path, ["start", "--save-images"])
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "ss-system:dev")
    params_at = run.index("--params-file")
    assert params_at < run.index("shot_log_dir:=out/shots-")
    assert params_at < run.index("shot_save_images:=true")

    result, lines = _run(tmp_path, ["start"])
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "ss-system:dev")
    assert "shot_log_dir" not in run
    assert "shot_save_images" not in run


def test_stop_fixes_ownership(tmp_path) -> None:
    result, lines = _run(
        tmp_path, ["stop"], SS_FAKE_RUNNING="1", SS_FAKE_STATUS_OUTPUT=STATUS_IDLE
    )
    assert result.returncode == 0, result.stderr
    stop_at = next(i for i, line in enumerate(lines) if line.startswith("stop "))
    chowns = [i for i, line in enumerate(lines) if "chown" in line]
    assert chowns and chowns[0] > stop_at


def test_usage(tmp_path) -> None:
    result, _lines = _run(tmp_path, [])
    assert result.returncode == 2
    result, _lines = _run(tmp_path, ["no-such-thing"])
    assert result.returncode == 2


def test_repo_root_from_anywhere(tmp_path) -> None:
    work = tmp_path / "elsewhere"
    work.mkdir()
    result, lines = _run(tmp_path, ["start"], cwd=work)
    assert result.returncode == 0, result.stderr
    run = _find(lines, "run", "ss-system:dev")
    assert f"{REPO_ROOT}:/ws" in run
