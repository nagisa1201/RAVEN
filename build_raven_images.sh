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
DOWNLOAD_SCENES=false            # true：准备当前实验场景和模型；不下载其他地图
# ==================================================

RAVEN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly RAVEN_ROOT
readonly AIRSTACK_DIR="$RAVEN_ROOT/AirStack"
readonly BUILD_CONTAINER="raven-build-rayfronts-$$"
DRY_RUN=false
PREPARE_RUNTIME=false
COMPILING=false
CURRENT_STEP='初始化与环境检查'

die() { printf '[错误] %s\n' "$*" >&2; exit 1; }
usage() {
    printf '%s\n' '用法：build_raven_images.sh [--prepare-runtime] [--dry-run]' \
        '构建 RayFronts + C++ 扩展、AirStack CLI/Robot/Isaac Sim、LVLM。' \
        '各开关在本文件顶部；构建失败立即停止，可再次运行以复用 Docker 缓存。' \
        '日志位于 ~/.cache/raven/build/；镜像位置由 Docker Root Dir 决定。' \
        '--prepare-runtime 同时准备当前实验场景、模型缓存和 ROS 工作区；不会启动实验。'
}
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        --prepare-runtime) PREPARE_RUNTIME=true ;;
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
build_in() {
    local attempt
    for attempt in 1 2 3; do
        if run_in "$@"; then return 0; fi
        if ((attempt < 3)); then
            printf '[重试] 镜像构建失败，第 %s/3 次；10 秒后复用已完成的构建层重试。\n' "$attempt" >&2
            sleep 10
        fi
    done
    return 1
}
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
isaac_config="$AIRSTACK_DIR/simulation/isaac-sim/docker/user.6.0.config.json"
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
build_in "$RAVEN_ROOT/RayFronts" docker build "${build_flags[@]}" \
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
build_in "$AIRSTACK_DIR" bash ./airstack.sh build "${build_flags[@]}" isaac-sim robot

CURRENT_STEP='构建 LVLM'
printf '\n[4/4] %s\n' "$CURRENT_STEP"
if [[ "$BUILD_LVLM" == true ]]; then
    build_in "$RAVEN_ROOT/LVLM" docker build "${build_flags[@]}" -t lvlm:latest .
else
    printf 'BUILD_LVLM=false，跳过；运行实验时也请将 USE_LVLM 改成 false。\n'
fi

if [[ "$DOWNLOAD_SCENES" == true ]]; then
    PREPARE_RUNTIME=true
fi
if [[ "$PREPARE_RUNTIME" == true ]]; then
    CURRENT_STEP='准备当前实验的场景、模型和 ROS 工作区'
    if [[ "$DRY_RUN" == true ]]; then
        printf '%s\n' '[预览] 按 run_raven_experiment.sh 读取场景/LVLM 设置；HTTP Range 只取选定场景。' \
            '[预览] 预编译 Robot ROS 工作区、检查 RayFronts C++，预取 RADIO/SigLIP 和启用的 InternVL 权重。'
    else
        deployment=$(bash "$RAVEN_ROOT/run_raven_experiment.sh" --deployment-config)
        scene=$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["scene"])' "$deployment")
        use_lvlm=$(python3 -c 'import json,sys; print(str(json.loads(sys.argv[1])["lvlm"]).lower())' "$deployment")
        [[ $use_lvlm != true || $BUILD_LVLM == true ]] || die '实验启用了 LVLM，但 BUILD_LVLM=false'
        # Read ZIP directory/ranges only. Never download the other scenes, and
        # reject HTTP servers that silently ignore Range instead of fetching 4+ GB.
        python3 - "$AIRSTACK_DIR/scenes" "$scene" <<'PYSCENE'
import io, os, pathlib, re, shutil, sys, time, urllib.request, zipfile
root, scene = pathlib.Path(sys.argv[1]), sys.argv[2]
assert re.fullmatch(r'[A-Za-z0-9_]+', scene), scene
expected = root / scene / (scene + '_Export.usd')
staging = root / ('.' + scene + '.preparing')
if expected.is_file() and expected.stat().st_size and not staging.exists():
    print('SCENE_READY', expected, flush=True)
    sys.exit(0)
url = 'https://drive.usercontent.google.com/download?id=1LJGZUzaJGkZ27hmXFuu7C97dlZv_w8jg&export=download&confirm=t'
class RemoteFile(io.RawIOBase):
    def __init__(self):
        request = urllib.request.Request(url, method='HEAD')
        with urllib.request.urlopen(request, timeout=90) as response:
            self.size = int(response.headers['Content-Length'])
        self.pos, self.start, self.cache = 0, -1, b''
    def seekable(self): return True
    def readable(self): return True
    def tell(self): return self.pos
    def seek(self, offset, whence=0):
        self.pos = (0 if whence == 0 else self.pos if whence == 1 else self.size) + offset
        if self.pos < 0: raise ValueError('Negative seek')
        return self.pos
    def read(self, n=-1):
        n = min(self.size-self.pos, n if n >= 0 else self.size-self.pos)
        if n <= 0: return b''
        if not (self.start <= self.pos and self.pos+n <= self.start+len(self.cache)):
            end = min(self.size-1, self.pos+max(n, 8*1024*1024)-1)
            request = urllib.request.Request(url, headers={'Range':f'bytes={self.pos}-{end}', 'Accept-Encoding':'identity'})
            for attempt in range(5):
                try:
                    with urllib.request.urlopen(request, timeout=90) as response:
                        if response.status != 206 or response.headers.get('Content-Range') != f'bytes {self.pos}-{end}/{self.size}':
                            raise RuntimeError('服务器未返回请求的 ZIP 范围；未下载全部地图。请将当前场景复制到 AirStack/scenes 后重试。')
                        self.cache = response.read(end-self.pos+1)
                        if len(self.cache) != end-self.pos+1: raise IOError('Incomplete HTTP range')
                    self.start = self.pos
                    break
                except (OSError, IOError):
                    if attempt == 4: raise
                    time.sleep(2**attempt)
        data = self.cache[self.pos-self.start:self.pos-self.start+n]
        self.pos += len(data)
        return data
# Stage separately: an interrupted extraction never masquerades as a ready scene.
staging.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(RemoteFile()) as archive:
    selected = []
    for item in archive.infolist():
        parts = pathlib.PurePosixPath(item.filename).parts
        if scene not in parts or '__MACOSX' in parts: continue
        tail = parts[parts.index(scene)+1:]
        if not tail or item.is_dir(): continue
        if '..' in tail or pathlib.PurePosixPath(item.filename).is_absolute(): raise ValueError(item.filename)
        selected.append((item, pathlib.Path(*tail)))
    if not selected: raise RuntimeError(f'官方场景包中没有 {scene}')
    print(f'仅准备 {scene}：{len(selected)} 个文件，压缩数据 {sum(i.compress_size for i,_ in selected)/1024**3:.2f} GiB', flush=True)
    for number, (item, relative) in enumerate(selected, 1):
        target = staging / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        with archive.open(item) as source, target.open('wb') as destination:
            shutil.copyfileobj(source, destination, 1024*1024)
        print(f'SCENE {number}/{len(selected)} {relative}', flush=True)
assert (staging / (scene+'_Export.usd')).is_file(), '场景导出文件缺失'
# Existing repository placeholders may exist; preserve their contents.
shutil.copytree(staging, root/scene, dirs_exist_ok=True)
shutil.rmtree(staging)
print('SCENE_READY', expected, flush=True)
PYSCENE
        # Compile with the same mounts used at experiment time, without starting
        # Compose services, simulation, navigation, RViz or a persistent container.
        robot_image="${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:v${DOCKER_IMAGE_TAG}_robot-x86-64"
        run docker run --rm --init --runtime runc -e NVIDIA_VISIBLE_DEVICES=void \
            -v "$AIRSTACK_DIR:/root/AirStack" \
            -v "$AIRSTACK_DIR/common/ros_packages:/root/AirStack/robot/ros_ws/src/common" \
            --entrypoint /bin/bash "$robot_image" -ec \
            'source /opt/ros/humble/setup.bash
cd /root/AirStack/robot/ros_ws
colcon build --symlink-install --executor sequential
source install/setup.bash
ros2 pkg prefix robot_bringup
ros2 pkg prefix behavior_tree_msgs'
        mkdir -p "$HOME/.cache/raven/torch" "$HOME/.cache/raven/huggingface" "$HOME/.cache/huggingface"
        run docker run --rm --init --runtime runc -e NVIDIA_VISIBLE_DEVICES=void \
            -e HF_HOME=/root/.cache/huggingface \
            -v "$RAVEN_ROOT/RayFronts:/workspace/RayFronts" -w /workspace/RayFronts \
            -v "$HOME/.cache/raven/torch:/root/.cache/torch" \
            -v "$HOME/.cache/raven/huggingface:/root/.cache/huggingface" \
            --entrypoint /bin/bash rayfronts:desktop -ec \
            'source /opt/ros/humble/setup.bash
export PYTHONPATH=/workspace/RayFronts/rayfronts/csrc/build:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
python3 -u - <<"PYMODEL"
from pathlib import Path
import torch
from hydra import compose, initialize_config_dir
from hydra.utils import instantiate
import rayfronts_cpp
# Pre-trust the same public repository the existing encoder already executes.
cache=Path(torch.hub.get_dir()); cache.mkdir(parents=True,exist_ok=True)
trusted=cache/"trusted_list"
entries=set(trusted.read_text().splitlines()) if trusted.exists() else set()
entries.add("NVlabs_RADIO"); trusted.write_text("\n".join(sorted(entries))+"\n")
with initialize_config_dir(config_dir="/workspace/RayFronts/rayfronts/configs", version_base=None):
    cfg=compose(config_name="default", overrides=["dataset=ros2isaacsim", "mapping=semantic_ray_frontiers_map"])
    model=instantiate(cfg.encoder, device="cpu", input_resolution=[448,448])
    print("RAYFRONTS_MODEL_READY", type(model).__name__, flush=True)
PYMODEL'
        if [[ $use_lvlm == true ]]; then
            run docker run --rm --init --runtime runc -e NVIDIA_VISIBLE_DEVICES=void \
                -e HF_HOME=/root/.cache/huggingface -e TRANSFORMERS_CACHE=/root/.cache/huggingface/hub \
                -v "$HOME/.cache/huggingface:/root/.cache/huggingface" \
                --entrypoint python3 lvlm:latest -u -c \
                'from huggingface_hub import snapshot_download; from transformers import AutoConfig, AutoTokenizer
path=snapshot_download("OpenGVLab/InternVL3-2B")
AutoConfig.from_pretrained(path, trust_remote_code=True)
AutoTokenizer.from_pretrained(path, trust_remote_code=True, use_fast=False)
print("LVLM_MODEL_READY", path, flush=True)'
        fi
        # Validate GPU injection separately from Kit startup. Uses an already
        # required image, so this check does not pull an extra CUDA test image.
        run docker run --rm --gpus all --entrypoint nvidia-smi \
            "${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:v${ISAAC_SIM_IMAGE_TAG}_isaac-sim" \
            --query-gpu=name,driver_version --format=csv,noheader
    fi
fi
CURRENT_STEP='检查构建结果'

images=(rayfronts:desktop "airstack-cli:v${DOCKER_IMAGE_TAG}"
    "${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:v${ISAAC_SIM_IMAGE_TAG:-0.16.2-isaac6.0.0}_isaac-sim"
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
    if [[ "$PREPARE_RUNTIME" == true ]]; then
        printf '当前场景、模型权重和 ROS 工作区已准备好，运行时复用同一份宿主缓存。\n'
    else
        printf '模型权重在首次运行时下载；提前准备请加 --prepare-runtime。\n'
    fi
fi
