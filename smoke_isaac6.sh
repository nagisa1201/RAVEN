#!/usr/bin/env bash
# Test the migrated simulator in a disposable container; never start navigation.
set -Eeuo pipefail
RAVEN_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
STACK="$RAVEN_ROOT/AirStack"
set -a
source "$STACK/.env"
set +a
image="${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:v${ISAAC_SIM_IMAGE_TAG}_isaac-sim"
report_dir=${RAVEN_SMOKE_DIR:-"$HOME/.cache/raven/isaac6-smoke/$(date +%Y%m%d_%H%M%S)"}
container="raven-isaac6-smoke-$$"
scene=${RAVEN_SMOKE_SCENE:-}
cache_root="$HOME/docker/isaac-sim/6.0/cache"
offline_options=()
if [[ ${RAVEN_SMOKE_OFFLINE:-0} == 1 ]]; then
    [[ -n "$scene" ]] || { echo '离线自检需要 RAVEN_SMOKE_SCENE' >&2; exit 1; }
    offline_options=(--network none -e RAVEN_SMOKE_OFFLINE=1
        -e "ISAACSIM_ASSET_ROOT=/isaac-sim/AirStack/scenes/$scene/.raven-offline/isaac-assets")
    # An online run's asset cache must not hide missing files in the bundle.
    cache_root="$report_dir/cold-cache"
fi
if [[ -n "$scene" ]]; then
    [[ "$scene" =~ ^[A-Za-z0-9_]+$ && -s "$STACK/scenes/$scene/${scene}_Export.usd" ]] || {
        printf '冒烟场景不存在：%s\n' "$scene" >&2; exit 1;
    }
fi
driver=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n 1)
if [[ "$driver" != 595.71.05 ]]; then
    printf '需要已加载的 NVIDIA 595.71.05，当前是 %s。安装驱动后请先重启。\n' "$driver" >&2
    exit 1
fi
docker image inspect "$image" >/dev/null
mkdir -p "$report_dir"
# Docker creates missing cache mount directories; older installs may have a
# root-owned ~/docker parent that the invoking user cannot write into.
cleanup() { docker rm -f "$container" >/dev/null 2>&1 || true; }
trap cleanup EXIT
printf 'Isaac Sim 6.0 冒烟测试；结果目录：%s\n' "$report_dir"
# Container variables are expanded by its bash, not by the host shell.
# shellcheck disable=SC2016
timeout --signal=TERM --kill-after=30s 1800s docker run --rm --init --name "$container" \
    --gpus 1 --ipc=host "${offline_options[@]}" \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ALLOW_ROOT=1 \
    -e PYTHONUNBUFFERED=1 \
    -e HOME=/isaac-sim -e ROS_DOMAIN_ID=213 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e RAVEN_SMOKE_REPORT=/results/smoke.json -e RAVEN_SMOKE_SECONDS=180 \
    -e RAVEN_SMOKE_SCENE="$scene" \
    -v "$STACK:/isaac-sim/AirStack:ro" \
    -v "$STACK/simulation/isaac-sim/extensions/PegasusSimulator/extensions/pegasus.simulator:/isaac-sim/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator:ro" \
    -v "$cache_root/main:/isaac-sim/.cache" \
    -v "$cache_root/computecache:/isaac-sim/.nv/ComputeCache" \
    -v "$cache_root/kit:/isaac-sim/kit/cache" \
    -v "$report_dir:/results" --entrypoint /bin/bash "$image" -ec \
    'source /opt/ros/jazzy/setup.bash
source /isaac-sim/jazzy_ws/install/setup.bash
kit_options=()
if [[ ${RAVEN_SMOKE_OFFLINE:-0} == 1 ]]; then kit_options+=(--/app/extensions/registryEnabled=false); fi
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/tests/smoke_isaac6.py --ext-folder /isaac-sim/.local/share/ov/data/documents/Kit/shared/exts "${kit_options[@]}"' \
    2>&1 | tee "$report_dir/console.log"
python3 - "$report_dir/smoke.json" <<'PY'
import json, sys
report = json.load(open(sys.argv[1]))
assert report['passed'], report
print('PASS: PX4 heartbeat, physics, ROS clock, stereo RGB, depth, camera info, lidar')
PY
