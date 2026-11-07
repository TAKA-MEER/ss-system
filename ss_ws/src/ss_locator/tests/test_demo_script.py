"""`scripts/demo.sh` の試験。`--dry-run` の出力だけを見る。

`docker` も `xhost` も呼ばないので、コンテナの中の `scripts/test.sh` で回る。
`/dev/video<N>` の有無は `SS_DEMO_DEV_DIR` で差し替える。
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
SCRIPT = REPO_ROOT / "scripts" / "demo.sh"


def _run(args, cwd, dev_dir, display=None):
    env = os.environ.copy()
    env["SS_DEMO_DEV_DIR"] = str(dev_dir)
    env.pop("DISPLAY", None)
    if display is not None:
        env["DISPLAY"] = display
    return subprocess.run(
        [str(SCRIPT), "--dry-run", *args],
        cwd=str(cwd),
        env=env,
        capture_output=True,
        text=True,
    )


def _lines(result):
    assert result.returncode == 0, result.stderr
    return result.stdout.splitlines()


def _value(lines, flag):
    return lines[lines.index(flag) + 1]


def _dev_dir(tmp_path, *names):
    dev = tmp_path / "dev"
    dev.mkdir(exist_ok=True)
    for name in names:
        (dev / name).touch()
    return dev


def test_default_source_usb0(tmp_path) -> None:
    dev = _dev_dir(tmp_path, "video0")

    lines = _lines(_run(["--headless"], tmp_path, dev))

    assert _value(lines, "--source") == "usb:0"
    assert _value(lines, "--device") == "/dev/video0"


def test_usb_device_index(tmp_path) -> None:
    dev = _dev_dir(tmp_path, "video2")

    lines = _lines(_run(["--source", "usb:2", "--headless"], tmp_path, dev))
    assert _value(lines, "--source") == "usb:2"
    assert _value(lines, "--device") == "/dev/video2"

    empty = tmp_path / "empty"
    empty.mkdir()
    result = _run(["--source", "usb:2", "--headless"], tmp_path, empty)
    assert result.returncode == 1
    assert "--device" not in result.stdout


def test_folder_mounted_readonly(tmp_path) -> None:
    folder = tmp_path / "in"
    folder.mkdir()
    dev = _dev_dir(tmp_path)

    lines = _lines(_run(["--source", "in", "--headless"], tmp_path, dev))

    assert _value(lines, "--source") == "/input"
    assert f"{folder.resolve()}:/input:ro" in lines
    assert "--device" not in lines


def test_video_file_mounts_parent(tmp_path) -> None:
    (tmp_path / "clip.mp4").write_bytes(b"")
    dev = _dev_dir(tmp_path)

    lines = _lines(_run(["--source", "clip.mp4", "--headless"], tmp_path, dev))

    assert _value(lines, "--source") == "/input/clip.mp4"
    assert f"{tmp_path.resolve()}:/input:ro" in lines


def test_missing_path_fails(tmp_path) -> None:
    dev = _dev_dir(tmp_path)

    result = _run(["--source", "nope", "--headless"], tmp_path, dev)

    assert result.returncode == 1
    assert "--source" not in result.stdout


def test_headless_has_no_display(tmp_path) -> None:
    dev = _dev_dir(tmp_path, "video0")

    lines = _lines(_run(["--headless"], tmp_path, dev, display=":0"))
    assert "--headless" in lines
    assert "DISPLAY" not in lines
    assert not any(".X11-unix" in line for line in lines)

    lines = _lines(_run([], tmp_path, dev, display=":0"))
    assert "DISPLAY" in lines
    assert "/tmp/.X11-unix:/tmp/.X11-unix" in lines
    assert "--headless" not in lines


def test_args_passed_through(tmp_path) -> None:
    dev = _dev_dir(tmp_path, "video0")

    lines = _lines(
        _run(["--headless", "--conf", "0.4", "--max-frames", "3"], tmp_path, dev)
    )

    assert _value(lines, "--conf") == "0.4"
    assert _value(lines, "--max-frames") == "3"
    assert "--dry-run" not in lines


def test_repo_root_from_anywhere(tmp_path) -> None:
    work = tmp_path / "elsewhere"
    work.mkdir()
    dev = _dev_dir(tmp_path, "video0")

    lines = _lines(_run(["--headless"], work, dev))

    assert f"{REPO_ROOT}:/ws" in lines
