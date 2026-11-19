#!/usr/bin/env bash
# ホストの端末から本システムを起こし、依頼を出し、止める。
# 詳細設計: docs/plan/detailed/DetailedDesign-tester.md §7
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

SS_DOCKER="${SS_DOCKER:-docker}"
SS_MODEL_PATH="${SS_MODEL_PATH:-$repo_root/model/lug_v3_s.pt}"
SS_START_WAIT_S="${SS_START_WAIT_S:-180}"
container_name="ss-system"
image_name="ss-system:dev"

usage() {
  cat >&2 <<'EOF'
使い方: scripts/ss.sh <やること> [引数]

やること:
  start [--camera-host <IP>] [--lift-host <名前かIP>] [--fakes]
      本システムを起こす
  status
      状態を1回出す
  locate [locate_cli の引数]
      依頼を出す（--plot・--json が無ければ out/ に足す）
  logs [-f]
      コンテナの記録を出す
  stop [--force]
      止めてコンテナを消す（依頼の最中は --force が要る）
  calib [mount_calib の引数]
      カメラの取り付けの校正を撮る・当てはめる
EOF
}

# docker のオプション（ROS_DOMAIN_ID があれば渡す）
docker_env_args=()
if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  docker_env_args+=(-e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID")
fi

container_running() {
  [ -n "$("$SS_DOCKER" ps -q --filter 'name=^ss-system$' 2>/dev/null)" ]
}

container_exists() {
  [ -n "$("$SS_DOCKER" ps -aq --filter 'name=^ss-system$' 2>/dev/null)" ]
}

# 状態を出す exec の共通形（/opt/ros/humble/setup.bash と install を読んで cd /ws）
exec_locate_cli() {
  local inner='. /opt/ros/humble/setup.bash && . /ws/ss_ws/install/setup.bash && cd /ws && exec ros2 run ss_locator locate_cli "$@"'
  if [ -t 0 ]; then
    "$SS_DOCKER" exec -it "${docker_env_args[@]}" "$container_name" bash -c "$inner" _ "$@"
  else
    "$SS_DOCKER" exec "${docker_env_args[@]}" "$container_name" bash -c "$inner" _ "$@"
  fi
}

fix_out_ownership() {
  "$SS_DOCKER" run --rm -v "$repo_root:/ws" "$image_name" chown -R "$(id -u):$(id -g)" /ws/out || true
}

cmd_start() {
  local camera_host="" lift_host="" use_fakes=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --camera-host)
        [ $# -ge 2 ] || { usage; exit 2; }
        camera_host="$2"; shift 2 ;;
      --camera-host=*) camera_host="${1#--camera-host=}"; shift ;;
      --lift-host)
        [ $# -ge 2 ] || { usage; exit 2; }
        lift_host="$2"; shift 2 ;;
      --lift-host=*) lift_host="${1#--lift-host=}"; shift ;;
      --fakes) use_fakes=1; shift ;;
      *) usage; exit 2 ;;
    esac
  done

  if container_running; then
    echo "コンテナ $container_name は既に動いています" >&2
    exit 1
  fi
  if container_exists; then
    "$SS_DOCKER" rm "$container_name"
  fi
  if [ "$use_fakes" -eq 0 ] && [ ! -f "$SS_MODEL_PATH" ]; then
    echo "モデルがありません: $SS_MODEL_PATH (--fakes なら無くても起こせます)" >&2
    exit 1
  fi
  if ! "$SS_DOCKER" image inspect "$image_name" >/dev/null 2>&1; then
    "$SS_DOCKER" build -t "$image_name" -f "$repo_root/docker/Dockerfile" "$repo_root"
  fi

  local overrides=()
  if [ -n "$camera_host" ]; then
    overrides+=(-p "camera_host:=$camera_host")
  fi
  if [ -n "$lift_host" ]; then
    overrides+=(-p "lift_host:=$lift_host")
  fi
  if [ "$use_fakes" -eq 1 ]; then
    overrides+=(-p "use_fakes:=true")
  fi
  # overrides は --params-file より後ろに付ける（引数の上書きが勝つため）
  local inner=". /opt/ros/humble/setup.bash && cd /ws/ss_ws && colcon build && . install/setup.bash && cd /ws && exec ros2 run ss_locator locator_node --ros-args --params-file ss_ws/src/ss_locator/config/locator.yaml"
  if [ "${#overrides[@]}" -gt 0 ]; then
    inner="$inner ${overrides[*]}"
  fi

  "$SS_DOCKER" run -d --name "$container_name" --network host --ipc host \
    -v "$repo_root:/ws" "${docker_env_args[@]}" -e PYTHONUNBUFFERED=1 \
    "$image_name" bash -c "$inner"

  # 状態が読めるまで待つ
  local deadline=$((SECONDS + SS_START_WAIT_S))
  while :; do
    set +e
    local status_output
    status_output="$(exec_locate_cli --status 2>/dev/null)"
    local status_code=$?
    set -e
    if [ "$status_code" -eq 0 ]; then
      printf '%s\n' "$status_output"
      exit 0
    fi
    # コンテナが落ちていたら上限まで待たない
    if ! container_running; then
      echo "コンテナが落ちました" >&2
      "$SS_DOCKER" logs --tail 50 "$container_name" || true
      exit 1
    fi
    if [ "$SECONDS" -ge "$deadline" ]; then
      echo "状態が読めませんでした" >&2
      "$SS_DOCKER" logs --tail 50 "$container_name" || true
      exit 1
    fi
    sleep 1
  done
}

cmd_status() {
  if ! container_running; then
    echo "コンテナ $container_name は動いていません" >&2
    exit 1
  fi
  exec_locate_cli --status
}

cmd_locate() {
  if ! container_running; then
    echo "コンテナ $container_name は動いていません" >&2
    exit 1
  fi
  local has_plot=0 has_json=0 arg
  for arg in "$@"; do
    case "$arg" in
      --plot|--plot=*) has_plot=1 ;;
      --json|--json=*) has_json=1 ;;
    esac
  done
  local args=("$@")
  if [ "$has_plot" -eq 0 ] || [ "$has_json" -eq 0 ]; then
    local stamp
    stamp="$(date +%Y%m%d-%H%M%S)"
    mkdir -p "$repo_root/out"
    if [ "$has_plot" -eq 0 ]; then
      args+=(--plot "out/$stamp.png")
    fi
    if [ "$has_json" -eq 0 ]; then
      args+=(--json "out/$stamp.json")
    fi
  fi
  set +e
  exec_locate_cli "${args[@]}"
  local code=$?
  set -e
  fix_out_ownership
  exit "$code"
}

cmd_logs() {
  "$SS_DOCKER" logs "$@" "$container_name"
}

cmd_stop() {
  local force=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --force) force=1; shift ;;
      *) usage; exit 2 ;;
    esac
  done

  if container_running; then
    set +e
    local status_output
    status_output="$(exec_locate_cli --status 2>/dev/null)"
    set -e
    local phase
    phase="$(printf '%s\n' "$status_output" | sed -n 's/.*phase=\([A-Z]*\).*/\1/p' | head -n 1)"
    # 状態が読めないとき（phase が空）は止める。読めて IDLE でないときは止めない
    if [ -n "$phase" ] && [ "$phase" != "IDLE" ] && [ "$force" -eq 0 ]; then
      echo "依頼の最中です（phase=$phase）。止めるには --force を付けてください" >&2
      exit 4
    fi
    "$SS_DOCKER" stop "$container_name"
    "$SS_DOCKER" rm -f "$container_name"
    exit 0
  fi
  if container_exists; then
    "$SS_DOCKER" rm "$container_name"
  fi
  exit 0
}

cmd_calib() {
  local has_from=0 has_params=0 has_out=0 arg
  for arg in "$@"; do
    case "$arg" in
      --from|--from=*) has_from=1 ;;
      --params-file|--params-file=*) has_params=1 ;;
      --out|--out=*) has_out=1 ;;
    esac
  done

  # --from だけのとき（撮らない）は動いていてもよい。それ以外は動いていたら撮らない
  if [ "$has_from" -eq 0 ] && container_running; then
    echo "コンテナ $container_name が動いているので撮れません" >&2
    exit 1
  fi

  local args=("$@")
  if [ "$has_params" -eq 0 ]; then
    args+=(--params-file ss_ws/src/ss_locator/config/locator.yaml)
  fi
  if [ "$has_out" -eq 0 ] && [ "$has_from" -eq 0 ]; then
    local stamp
    stamp="$(date +%Y%m%d-%H%M%S)"
    mkdir -p "$repo_root/out"
    args+=(--out "out/calib-$stamp")
  fi

  local inner='cd /ws && PYTHONPATH=ss_ws/src/ss_locator python3 -m ss_locator.mount_calib "$@"'
  local run_args=(run --rm --network host)
  if [ -t 0 ]; then
    run_args+=(-it)
  fi
  run_args+=("${docker_env_args[@]}" -v "$repo_root:/ws" "$image_name" bash -c "$inner" _ "${args[@]}")

  set +e
  "$SS_DOCKER" "${run_args[@]}"
  local code=$?
  set -e
  fix_out_ownership
  exit "$code"
}

if [ $# -eq 0 ]; then
  usage
  exit 2
fi

command_name="$1"
shift
case "$command_name" in
  start) cmd_start "$@" ;;
  status) cmd_status "$@" ;;
  locate) cmd_locate "$@" ;;
  logs) cmd_logs "$@" ;;
  stop) cmd_stop "$@" ;;
  calib) cmd_calib "$@" ;;
  -h|--help|help) usage; exit 0 ;;
  *) usage; exit 2 ;;
esac
