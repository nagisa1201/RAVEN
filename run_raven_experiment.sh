#!/usr/bin/env bash
# 改下面的配置后直接运行 ./run_raven_experiment.sh，无需重复输入长命令。
# ./run_raven_experiment.sh --dry-run 仅预览；额外参数会原样传给 Python 启动器。
# Ctrl+C 交给统一启动器保存结果并停止本轮容器。
set -Eeuo pipefail

# ==================== 实验配置 ====================
SCENE="ConstructionSite"        # 场景名称，与 *_Launch.py 对应
TASK="cabling winches"          # 支持带空格的类别，如 "water tower"
USE_LVLM=true
LOW_VRAM=true                    # 节省观察窗口显存；不显示 Isaac 主窗口
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
