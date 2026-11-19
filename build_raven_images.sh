#!/usr/bin/env bash
# RAVEN 桌面实验镜像的一次性构建入口；依据各子目录 README。
# 用法：./build_raven_images.sh          # 构建，保留完整终端日志
#       ./build_raven_images.sh --dry-run # 只打印步骤，不构建、不下载
# 前提：完整仓库/子模块，当前用户能使用 Docker。运行实验另需 NVIDIA 驱动和 Toolkit。
# 新机器先用 git clone --recurse-submodules 获取仓库；已有本地修改不会被 git update 覆盖。

set -Eeuo pipefail

# ==================== 可修改配置 ====================
BUILD_LVLM=true                  # 默认包含 LVLM
PULL_BASE_IMAGES=false           # true：构建时检查更新基础镜像；通常保留缓存即可
NO_CACHE=false                  # true：不用构建缓存，耗时和下载量更大
CPP_BUILD_JOBS=4                 # RayFronts C++ 编译并发数
INSTALL_HOST_DEPS=false          # 新机器可改 true：先执行 AirStack 的 install（可能要求 sudo）
DOWNLOAD_SCENES=false            # 镜像之外的可选步骤：运行官方场景下载脚本，会覆盖同名场景文件
# ==================================================

readonly RAVEN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly AIRSTACK_DIR="$RAVEN_ROOT/AirStack"
readonly BUILD_CONTAINER="raven-build-rayfronts-$$"
DRY_RUN=false
COMPILING=false
CURRENT_STEP='初始化与环境检查'

die() { printf '[错误] %s\n' "$*" >&2; exit 1; }
usage() {
    printf '%s\n' '用法：build_raven_images.sh [--dry-run]' \
        '构建 RayFronts + C++ 扩展、AirStack CLI/Robot/Isaac Sim、LVLM。' \
        '各开关在本文件顶部；构建失败立即停止，可再次运行以复用 Docker 缓存。' \
        '日志位于 ~/.cache/raven/build/；镜像位置由 Docker Root Dir 决定。' \
        '不会启动实验。完成后运行同目录的 run_raven_experiment.sh。'
}
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        -h|--help) usage; exit 0 ;;
        *) die "不支持参数：$arg；请使用 --help" ;;
    esac
done
for setting in BUILD_LVLM PULL_BASE_IMAGES NO_CACHE INSTALL_HOST_DEPS DOWNLOAD_SCENES; do
    [[ "${!setting}" == true || "${!setting}" == false ]] || die "$setting 必须是 true 或 false"
done
[[ "$CPP_BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || die 'CPP_BUILD_JOBS 必须为正整数'

# 所有真正执行的外部步骤都经过这里，路径/参数按 shell 转义打印。
run() {
    printf '+ '; printf '%q ' "$@"; printf '\n'
    if [[ "$DRY_RUN" == false ]]; then "$@"; fi
}
run_in() (
    local directory="$1"; shift
    printf '[目录] %s\n' "$directory"
    cd -- "$directory"
    run "$@"
)
cleanup() {
    local status=$?
    if [[ "$COMPILING" == true && "$DRY_RUN" == false ]]; then
        docker rm -f "$BUILD_CONTAINER" >/dev/null 2>&1 || true
    fi
    if (( status != 0 )); then
        printf '[失败] %s，退出码 %s；请查看上方输出及构建日志。\n' "$CURRENT_STEP" "$status" >&2
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

required=(AirStack/.env AirStack/airstack.sh AirStack/docker-compose.yaml
    AirStack/simulation/isaac-sim/docker/user_TEMPLATE.config.json
    RayFronts/docker/desktop.Dockerfile RayFronts/compile.sh)
if [[ "$BUILD_LVLM" == true ]]; then required+=(LVLM/Dockerfile); fi
for path in "${required[@]}"; do
    [[ -f "$RAVEN_ROOT/$path" ]] || die "缺少 $path；请先准备完整仓库及子模块"
done

if [[ "$DRY_RUN" == false ]]; then
    mkdir -p "$HOME/.cache/raven/build"
    exec 9>>"$HOME/.cache/raven/console.lock"
    flock -n 9 || die '统一启动器或另一个构建正在运行，请先结束它再构建'
    LOG_FILE="$HOME/.cache/raven/build/$(date +%Y%m%d-%H%M%S)-$$.log"
    exec > >(tee -a "$LOG_FILE") 2>&1
    printf '[日志] %s\n' "$LOG_FILE"
fi

if [[ "$INSTALL_HOST_DEPS" == true ]]; then
    run_in "$AIRSTACK_DIR" bash ./airstack.sh install
fi
if [[ "$DRY_RUN" == false ]]; then
    command -v docker >/dev/null || die '缺少 Docker；可配置 INSTALL_HOST_DEPS=true 后重试'
fi
run docker info --format 'Docker Root Dir={{.DockerRootDir}}'

# setup 中实验实际需要的本地文件：只补缺失项，保留已有配置，不询问 Nucleus token。
isaac_config="$AIRSTACK_DIR/simulation/isaac-sim/docker/user.config.json"
omni_env="$AIRSTACK_DIR/simulation/isaac-sim/docker/omni_pass.env"
for path in "$isaac_config" "$omni_env"; do
    [[ ! -e "$path" || -f "$path" ]] || die "$path 应为文件，但目前是其他类型；请先检查"
done
if [[ ! -f "$isaac_config" ]]; then
    run cp "$AIRSTACK_DIR/simulation/isaac-sim/docker/user_TEMPLATE.config.json" "$isaac_config"
fi
if [[ ! -f "$omni_env" ]]; then run touch "$omni_env"; fi

# 与统一启动器相同：载入受信任的本地 .env，不打印其中的凭据。
set +u
set -a
source "$AIRSTACK_DIR/.env"
set +a
set -u
: "${PROJECT_DOCKER_REGISTRY:?AirStack/.env 缺少 PROJECT_DOCKER_REGISTRY}"
: "${PROJECT_NAME:?AirStack/.env 缺少 PROJECT_NAME}"
: "${DOCKER_IMAGE_TAG:?AirStack/.env 缺少 DOCKER_IMAGE_TAG}"
# 仅使用 legacy builder 和 BuildKit 都支持的选项。
# --progress 属于 Buildx；宿主未安装该插件时 docker build 会直接退出 125。
# stdout/stderr 已由 tee 保存，省略 --progress 仍有完整构建日志。
build_flags=()
if [[ "$PULL_BASE_IMAGES" == true ]]; then build_flags+=(--pull); fi
if [[ "$NO_CACHE" == true ]]; then build_flags+=(--no-cache); fi

CURRENT_STEP='构建 RayFronts 镜像'
printf '\n[1/4] %s\n' "$CURRENT_STEP"
run_in "$RAVEN_ROOT/RayFronts" docker build "${build_flags[@]}" \
    -f docker/desktop.Dockerfile -t rayfronts:desktop .

CURRENT_STEP='编译并验证 RayFronts C++ 扩展'
printf '\n[2/4] 在临时 CPU 容器内%s\n' "$CURRENT_STEP"
COMPILING=true
run docker run --rm --init --name "$BUILD_CONTAINER" --network none --runtime runc \
    -e NVIDIA_VISIBLE_DEVICES=void -e CMAKE_INSTALL_PREFIX=/usr/local \
    -e CMAKE_BUILD_PARALLEL_LEVEL="$CPP_BUILD_JOBS" -e LD_LIBRARY_PATH=/usr/local/lib \
    -e PYTHONPATH=/workspace/RayFronts/rayfronts/csrc/build \
    -e RAVEN_BUILD_UID="$(id -u)" -e RAVEN_BUILD_GID="$(id -g)" \
    -v "$RAVEN_ROOT/RayFronts:/workspace/RayFronts" -w /workspace/RayFronts \
    --entrypoint /bin/bash rayfronts:desktop -e -c \
    'bash -e ./compile.sh
python3 -B -c "import rayfronts_cpp; print(rayfronts_cpp.__file__)"
chown -R "$RAVEN_BUILD_UID:$RAVEN_BUILD_GID" rayfronts/csrc/build'
COMPILING=false

CURRENT_STEP='构建 AirStack Robot 和 Isaac Sim'
printf '\n[3/4] %s（CLI 镜像缺失时由 AirStack 自动构建）\n' "$CURRENT_STEP"
# 只构建此实验用到的服务；不构建文档站、Gazebo/simple-sim、Jetson 等其他用途镜像。
run_in "$AIRSTACK_DIR" bash ./airstack.sh build "${build_flags[@]}" isaac-sim robot

CURRENT_STEP='构建 LVLM'
printf '\n[4/4] %s\n' "$CURRENT_STEP"
if [[ "$BUILD_LVLM" == true ]]; then
    run_in "$RAVEN_ROOT/LVLM" docker build "${build_flags[@]}" -t lvlm:latest .
else
    printf 'BUILD_LVLM=false，跳过；运行实验时也请将 USE_LVLM 改成 false。\n'
fi

if [[ "$DOWNLOAD_SCENES" == true ]]; then
    CURRENT_STEP='下载场景'
    run_in "$AIRSTACK_DIR/scenes" bash -e ./download_scenes.sh
fi
CURRENT_STEP='检查构建结果'

images=(rayfronts:desktop "airstack-cli:v${DOCKER_IMAGE_TAG}"
    "${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:v${DOCKER_IMAGE_TAG}_isaac-sim"
    "${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:v${DOCKER_IMAGE_TAG}_robot-x86-64")
if [[ "$BUILD_LVLM" == true ]]; then images+=(lvlm:latest); fi
run docker image inspect --format '{{join .RepoTags ", "}} -> {{.Id}}' "${images[@]}"

if [[ "$DRY_RUN" == true ]]; then
    printf '\n[预览结束] 没有执行构建、下载或配置写入。\n'
else
    # Console 会复用辅助容器；提示旧容器与新镜像不一致的情况，不删除容器中的用户数据。
    for pair in 'rayfronts_container rayfronts:desktop' 'lvlm_container lvlm:latest'; do
        read -r container image <<< "$pair"
        if [[ "$container" == lvlm_container && "$BUILD_LVLM" == false ]]; then continue; fi
        old_id=$(docker inspect --type container --format '{{.Image}}' "$container" 2>/dev/null || true)
        new_id=$(docker image inspect --format '{{.Id}}' "$image")
        if [[ -n "$old_id" && "$old_id" != "$new_id" ]]; then
            printf '[注意] %s 仍使用旧镜像。下次启动前须备好容器内数据并重建此容器，才能使用新镜像。\n' "$container"
        fi
    done
    printf '\n[完成] 镜像及 C++ 扩展已准备好。日志：%s\n' "$LOG_FILE"
    printf '场景和 NVIDIA 容器运行环境准备好后，执行：%q\n' "$RAVEN_ROOT/run_raven_experiment.sh"
    printf '模型权重按原程序在首次运行时下载，并使用启动器配置的宿主缓存。\n'
fi
