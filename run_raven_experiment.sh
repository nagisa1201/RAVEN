#!/usr/bin/env bash
# shellcheck disable=SC2034
# 改下面的配置后直接运行 ./run_raven_experiment.sh，无需重复输入长命令。
# ./run_raven_experiment.sh --dry-run 仅预览；额外参数会原样传给 Python 启动器。
# Ctrl+C 交给统一启动器保存结果并停止本轮容器。
set -Eeuo pipefail
# Variables in the configuration block are read via ${!name} by add_switch.

# ==================== 实验配置 ====================
SCENE="ConstructionSite"        # 场景名称，与 *_Launch.py 对应
TASK="cabling winches"          # 支持带空格的类别，如 "water tower"
USE_LVLM=true
LOW_VRAM=false                    # 节省观察窗口显存；不显示 Isaac 主窗口
DISABLE_MACVO=true               # 当前仿真控制链路未使用的 MACVO
KEEP_RANDOM_WALK=false           # 通常关闭，避免与 RayFronts 竞争全局路径
AUTO_TAKEOFF=true
AUTO_PLAN=true

ENABLE_EVALUATION=true
EPISODE_SECONDS=600              # 导航确认后的仿真秒数
GOAL_RADIUS=6                    # 真实目标包围盒评分半径；本地验证配置，非论文公布值
WAYPOINT_RADIUS=5                # 体素导航候选的航点到达阈值；与评分半径独立

ENABLE_VRAM_PROTECTION=false      # true 开启显存保护；false 关闭；仅在 ENABLE_EVALUATION=true 时生效
MIN_FREE_VRAM=512                # 开启保护时的空闲显存阈值（MiB）；持续低于阈值触发退出

EPISODE_WALL_TIMEOUT=3600        # 导航开始后的现实时间保护上限（秒）
DATA_TIMEOUT=60                 # 关键数据中断保护（现实秒）
AUTO_TIMEOUT=900                # 自动启动各等待阶段的超时（现实秒）

RESTART_EXISTING=true           # true 会结束已有 RAVEN 仿真再启动
SKIP_ROS_BUILD=false            # true 跳过本轮容器内 ROS 工作区编译
LOG_DIR=""                       # 留空：每轮自动生成独立时间目录
POSE_ARGS=()                     # 留空用场景默认值；例：(--x 0 --y 0 --z 0.07 --qw 1)
# ==============================================================

RAVEN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LAUNCHER="$RAVEN_ROOT/launch_raven_console.py"
[[ -f "$LAUNCHER" ]] || { printf '找不到启动器：%s\n' "$LAUNCHER" >&2; exit 1; }

# 安装脚本读取同一份配置，不复制或修改实验参数。
if [[ ${1:-} == --deployment-config ]]; then
    python3 - "$SCENE" "$USE_LVLM" <<'PYCONFIG'
import json, sys
print(json.dumps({'scene':sys.argv[1], 'lvlm':sys.argv[2]=='true'}))
PYCONFIG
    exit 0
fi

# An imported bundle uses the same scene/parameters with local asset addresses.
# The importer writes READY only after its offline build/model checks pass.
offline_dir="$RAVEN_ROOT/AirStack/scenes/$SCENE/.raven-offline"
if [[ -f "$offline_dir/READY" ]]; then
    export RAVEN_OFFLINE=1 HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1
    export ISAACSIM_ASSET_ROOT="/isaac-sim/AirStack/scenes/$SCENE/.raven-offline/isaac-assets"
fi

check_environment() (
    set +u
    source "$RAVEN_ROOT/AirStack/.env"
    set -u
    [[ ${ISAAC_SIM_VERSION:-} == 6.0.0 ]] || { echo 'AirStack 必须配置 ISAAC_SIM_VERSION=6.0.0' >&2; exit 1; }
    for command in docker nvidia-smi python3 Xvfb xauth xdpyinfo; do
        command -v "$command" >/dev/null || { echo "缺少 $command；请由老师执行 RAVEN_setup.sh" >&2; exit 1; }
    done
    docker info >/dev/null 2>&1 || { echo '当前账号不能访问 Docker；老师安装完成后请重新连接 SSH，以加载 docker 组权限。' >&2; exit 1; }
    nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
    images=(rayfronts:desktop "${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:v${ISAAC_SIM_IMAGE_TAG}_isaac-sim"
        "${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:v${DOCKER_IMAGE_TAG}_robot-x86-64" "airstack-cli:v${DOCKER_IMAGE_TAG}")
    [[ "$USE_LVLM" != true ]] || images+=(lvlm:latest)
    docker image inspect "${images[@]}" >/dev/null || { echo '实验镜像未构建完整；执行 build_raven_images.sh --prepare-runtime' >&2; exit 1; }
    [[ -s "$RAVEN_ROOT/AirStack/scenes/$SCENE/${SCENE}_Export.usd" ]] || { echo "缺少 $SCENE 场景；执行 build_raven_images.sh --prepare-runtime" >&2; exit 1; }
    [[ -s "$RAVEN_ROOT/RayFronts/rayfronts/annotations/$SCENE.json" ]] || { echo '缺少场景评估标注' >&2; exit 1; }
    if [[ ${RAVEN_OFFLINE:-0} == 1 ]]; then
        [[ -s "$offline_dir/isaac-assets/Isaac/Sensors/Stereolabs/ZED_X/ZED_X.usdc" &&
           -s "$offline_dir/isaac-assets/Isaac/Sensors/Ouster/OS1/OS1.usd" ]] || {
            echo '离线传感器资源缺失，请重新导入完整运行包' >&2; exit 1;
        }
        echo '[离线模式] 模型使用本地缓存，Isaac 使用已导入的本地资源。'
    fi
    echo '[检查通过] Isaac Sim 6.0 镜像、GPU、Docker 权限、SSH 显示工具及当前场景齐备。'
)
if [[ ${1:-} == --check ]]; then check_environment; exit; fi
# --dry-run/--help 保持纯预览，不能启动 X server、容器或下载。
preview=false
for argument in "$@"; do
    [[ $argument != --dry-run && $argument != --help && $argument != -h ]] || preview=true
done
if [[ $preview == false ]]; then
    check_environment
    # SSH 无桌面时给现有 RViz/RQT 提供虚拟 X 显示；算法及实验参数不变。
    if [[ -z ${DISPLAY:-} ]] || ! xdpyinfo >/dev/null 2>&1; then
        [[ ${RAVEN_XVFB_ACTIVE:-0} != 1 ]] || { echo '虚拟显示启动失败' >&2; exit 1; }
        export RAVEN_XVFB_ACTIVE=1
        exec xvfb-run -a -s '-screen 0 1920x1080x24 -nolisten tcp' bash "$0" "$@"
    fi
    if [[ ${RAVEN_XVFB_ACTIVE:-0} == 1 ]]; then
        # Xvfb's hostname-specific cookie must also authorize Docker hostnames.
        xauth_entries=$(xauth -f "$XAUTHORITY" nlist "$DISPLAY")
        printf '%s\n' "$xauth_entries" | sed 's/^..../ffff/' | xauth -f "$XAUTHORITY" nmerge -
    fi
fi

args=(--env "$SCENE" --task "$TASK" --waypoint-radius "$WAYPOINT_RADIUS"
    --episode-wall-timeout "$EPISODE_WALL_TIMEOUT"
    --data-timeout "$DATA_TIMEOUT" --auto-timeout "$AUTO_TIMEOUT")

add_switch() {
    local name="$1" flag="$2"
    case "${!name}" in
        true) args+=("$flag") ;;
        false) ;;
        *) printf '%s 必须是 true 或 false\n' "$name" >&2; exit 1 ;;
    esac
}
add_switch USE_LVLM --lvlm
add_switch LOW_VRAM --low-vram
add_switch DISABLE_MACVO --disable-macvo
add_switch KEEP_RANDOM_WALK --keep-random-walk
add_switch AUTO_TAKEOFF --auto-takeoff
add_switch AUTO_PLAN --auto-plan
add_switch RESTART_EXISTING --restart
add_switch SKIP_ROS_BUILD --skip-build
case "$ENABLE_VRAM_PROTECTION" in
    true) args+=(--min-free-vram "$MIN_FREE_VRAM") ;;
    false) args+=(--min-free-vram 0) ;;
    *) printf 'ENABLE_VRAM_PROTECTION 必须是 true 或 false\n' >&2; exit 1 ;;
esac
case "$ENABLE_EVALUATION" in
    true) args+=(--episode-seconds "$EPISODE_SECONDS" --goal-radius "$GOAL_RADIUS") ;;
    false) ;; # 持续运行，不按目标数量/预算/显存阈值自动结束；需要手动 Ctrl+C。
    *) printf 'ENABLE_EVALUATION 必须是 true 或 false\n' >&2; exit 1 ;;
esac
if [[ -n "$LOG_DIR" ]]; then args+=(--log-dir "$LOG_DIR"); fi
args+=("${POSE_ARGS[@]}" "$@")

printf '[启动命令] '; printf '%q ' python3 -u "$LAUNCHER" "${args[@]}"; printf '\n'
exec python3 -u "$LAUNCHER" "${args[@]}"
