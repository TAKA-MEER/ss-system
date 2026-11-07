#!/usr/bin/env bash
# 金具認識のリアルタイム表示を Docker で起動する。
# 詳細設計: docs/plan/detailed/DetailedDesign-demo.md §4.1
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

dry_run=0
args=()
for arg in "$@"; do
  if [ "$arg" = "--dry-run" ]; then
    dry_run=1
  else
    args+=("$arg")
  fi
done

normalized=()
for arg in "${args[@]}"; do
  case "$arg" in
    --source=*) normalized+=(--source "${arg#--source=}") ;;
    *) normalized+=("$arg") ;;
  esac
done
args=("${normalized[@]}")

source_value=""
source_index=-1
headless=0
i=0
while [ "$i" -lt "${#args[@]}" ]; do
  case "${args[$i]}" in
    --source)
      if [ $((i + 1)) -lt "${#args[@]}" ]; then
        source_value="${args[$((i + 1))]}"
        source_index=$i
      else
        echo "--source requires a value" >&2
        exit 1
      fi
      ;;
    --headless) headless=1 ;;
  esac
  i=$((i + 1))
done

if [ -z "$source_value" ]; then
  args+=(--source usb:0)
  source_value="usb:0"
  source_index=$((${#args[@]} - 2))
fi

mount_args=()
if [[ "$source_value" == usb:* ]]; then
  index="${source_value#usb:}"
  dev_dir="${SS_DEMO_DEV_DIR:-/dev}"
  if [ ! -e "$dev_dir/video$index" ]; then
    echo "USB camera not found: /dev/video$index" >&2
    exit 1
  fi
  mount_args+=(--device "/dev/video$index")
else
  if [ ! -e "$source_value" ]; then
    echo "source not found: $source_value" >&2
    exit 1
  fi
  abs="$(realpath -- "$source_value")"
  if [ -d "$abs" ]; then
    mount_args+=(-v "$abs:/input:ro")
    args[source_index + 1]="/input"
  else
    mount_args+=(-v "$(dirname -- "$abs"):/input:ro")
    args[source_index + 1]="/input/$(basename -- "$abs")"
  fi
fi

run_args=(run --rm)
if [ -t 0 ]; then
  run_args+=(-it)
fi
if [ "$headless" -eq 0 ]; then
  if [ "$dry_run" -eq 0 ] && [ -z "${DISPLAY:-}" ]; then
    echo "DISPLAY is not set; use --headless if you do not need the window" >&2
    exit 1
  fi
  run_args+=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix)
fi
run_args+=("${mount_args[@]}" -v "$repo_root:/ws")

inner='cd /ws/ss_ws && colcon build >/dev/null 2>&1 && . install/setup.bash && cd /ws && exec ros2 run ss_locator lug_demo "$@"'
run_args+=(ss-system:dev bash -c "$inner" _ "${args[@]}")

if [ "$dry_run" -eq 1 ]; then
  printf '%s\n' docker "${run_args[@]}"
  exit 0
fi

if [ ! -f "$repo_root/model/lug_v3_s.pt" ]; then
  echo "model not found: $repo_root/model/lug_v3_s.pt (place the trained weights there)" >&2
  exit 1
fi

if ! docker image inspect ss-system:dev >/dev/null 2>&1; then
  docker build -t ss-system:dev -f "$repo_root/docker/Dockerfile" "$repo_root"
fi

if [ "$headless" -eq 0 ] && command -v xhost >/dev/null 2>&1; then
  xhost +local:docker
fi

exec docker run "${run_args[@]}"
