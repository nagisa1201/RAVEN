#!/usr/bin/env bash
# Ubuntu 22.04/24.04 amd64；已安装 NVIDIA 595.71.05 的 SSH 服务器。
# 老师执行：sudo bash RAVEN_setup.sh --user 学生账号
# 预览：bash RAVEN_setup.sh --user 学生账号 --dry-run
# 普通用户复查：bash RAVEN_setup.sh --check
set -Eeuo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
TARGET_USER=${SUDO_USER:-$(id -un)}
MODE=install
DOCKER_DATA_ROOT=""
OFFLINE_HOST=""
fail() { printf '[错误] %s\n' "$*" >&2; exit 1; }
while (($#)); do
    case "$1" in
        --user) [[ $# -ge 2 ]] || fail '--user 缺少账号'; TARGET_USER=$2; shift 2 ;;
        --docker-data-root) [[ $# -ge 2 ]] || fail '缺少 Docker 数据目录'; DOCKER_DATA_ROOT=$2; shift 2 ;;
        --offline-host) [[ $# -ge 2 ]] || fail '--offline-host 缺少离线依赖包目录'; OFFLINE_HOST=$(readlink -f -- "$2"); shift 2 ;;
        --dry-run) MODE=preview; shift ;;
        --check) MODE=check; shift ;;
        -h|--help)
            printf '%s\n' 'sudo bash RAVEN_setup.sh --user 学生账号 [--docker-data-root /大容量磁盘/docker]' \
                '仅新装 Docker 可指定数据目录；现有 Docker 保持原配置。' \
                '安装系统依赖和 GPU 容器环境，构建全部实验镜像，预取当前场景和模型，最后检查。' \
                '不安装宿主 CUDA/ROS，不修改驱动或实验参数，不启动正式实验。' \
                '--offline-host DIR：仅从已校验的本地 deb 安装管理员负责的主机环境，不访问在线软件源。' \
                '该模式不构建/拉取镜像或下载模型；后续镜像和资源导入由学生账号完成。' \
                '--dry-run 只预览；--check 普通用户检查已安装环境。'; exit 0 ;;
        *) fail "未知参数：$1" ;;
    esac
done
[[ $MODE != check ]] || exec bash "$ROOT/run_raven_experiment.sh" --check
id "$TARGET_USER" >/dev/null 2>&1 || fail "用户不存在：$TARGET_USER"
[[ $(id -u "$TARGET_USER") != 0 ]] || fail '请用 --user 指定运行实验的普通账号'
TARGET_HOME=$(getent passwd "$TARGET_USER" | cut -d: -f6)
TARGET_UID=$(id -u "$TARGET_USER")
TARGET_GID=$(id -g "$TARGET_USER")
[[ -f "$ROOT/AirStack/.env" && -f "$ROOT/RayFronts/docker/desktop.Dockerfile" ]] || fail '请传输完整的已升级 RAVEN 工程（包括所有子模块），不能只传这个 sh'
source /etc/os-release
[[ $ID == ubuntu && ($VERSION_ID == 22.04 || $VERSION_ID == 24.04) && $(uname -m) == x86_64 ]] || fail '仅支持 Ubuntu 22.04/24.04 amd64'
if [[ -n $OFFLINE_HOST ]]; then
    [[ -f "$OFFLINE_HOST/bundle.json" && -f "$OFFLINE_HOST/SHA256SUMS" ]] || fail '离线包缺少 bundle.json 或 SHA256SUMS'
    (cd "$OFFLINE_HOST" && sha256sum --check --strict --quiet SHA256SUMS) || fail '离线包校验失败，请重新传输；尚未安装软件'
    python3 - "$OFFLINE_HOST/bundle.json" "$VERSION_ID" <<'PY'
import json, sys
with open(sys.argv[1]) as stream:
    data = json.load(stream)
assert data['format'] == 'raven-offline-host-v1', '不支持的离线包格式'
assert data['ubuntu'] == sys.argv[2] and data['arch'] == 'amd64', '离线包与服务器系统不匹配'
PY
    shopt -s nullglob
    offline_debs=("$OFFLINE_HOST"/debs/*.deb)
    shopt -u nullglob
    ((${#offline_debs[@]})) || fail '离线包内没有 deb 文件'
fi
# Read only the supplied packages and the host's installed-package database.
# Empty source/list paths prevent a stale online APT index from satisfying a
# missing dependency in the simulation but then failing during installation.
offline_apt() (
    local lists
    lists=$(mktemp -d)
    trap 'rm -rf -- "$lists"' EXIT
    LC_ALL=C apt-get -o Dir::Etc::sourcelist=/dev/null -o Dir::Etc::sourceparts=- \
        -o Dir::State::lists="$lists" -o Dir::Cache::pkgcache= -o Dir::Cache::srcpkgcache= \
        --no-download --no-remove --no-install-recommends "$@" install "${offline_debs[@]}"
)
if [[ -n $DOCKER_DATA_ROOT ]]; then
    [[ $DOCKER_DATA_ROOT == /* && $DOCKER_DATA_ROOT != / && $DOCKER_DATA_ROOT != *$'\n'* ]] || fail 'Docker 数据目录必须是独立的绝对路径'
    command -v dockerd >/dev/null && fail '已安装 Docker，脚本不会迁移既有镜像目录；省略 --docker-data-root'
fi
if [[ -z $DOCKER_DATA_ROOT && ! -f /etc/docker/daemon.json ]] && ! command -v dockerd >/dev/null; then
    DOCKER_DATA_ROOT="$TARGET_HOME/.local/share/raven/docker"
fi
if [[ $MODE == preview ]]; then
    printf '目标用户：%s；工程：%s；Docker 目录：%s\n' "$TARGET_USER" "$ROOT" "${DOCKER_DATA_ROOT:-保留现状，新装默认 /var/lib/docker}"
    if [[ -n $OFFLINE_HOST ]]; then
        printf '[离线预检] 校验包内容并模拟 APT 依赖解析，不安装软件或启动服务。\n'
        offline_apt --simulate
        printf '[预检通过] 实际安装还会配置 Docker/NVIDIA runtime、存储目录及学生权限。\n'
        exit 0
    fi
    printf '%s\n' '1. 检查已加载的 NVIDIA 595.71.05，保持驱动不变。' \
        '2. apt 安装 curl、签名工具、Git/LFS、Python、Xvfb、xauth、构建和诊断工具。' \
        '3. 安装 Docker Engine、Buildx、Compose、NVIDIA Container Toolkit。' \
        '4. 配置 nvidia runtime；授予目标账号 docker 组权限；准备可写缓存。' \
        '5. 以目标账号构建 RayFronts、Robot、Isaac Sim 6.0、LVLM；编译 ROS/C++。' \
        '6. 仅准备 run_raven_experiment.sh 选定的场景，预取实际使用的模型权重。' \
        '7. 运行容器 GPU、依赖及 Isaac 6 冒烟检查；成功才报告安装完成。'
    exec bash "$ROOT/build_raven_images.sh" --prepare-runtime --dry-run
fi
((EUID == 0)) || fail '系统依赖安装由老师用 sudo 执行；学生后续运行不需要 sudo'
command -v nvidia-smi >/dev/null || fail '请先安装并加载 NVIDIA 595.71.05（目标服务器的既定前提）'
drivers=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | sort -u)
[[ $drivers == 595.71.05 ]] || fail "需要已加载 595.71.05，实际为 $drivers；本脚本不替换服务器驱动"

for directory in "$TARGET_HOME/.cache" "$TARGET_HOME/.cache/raven" "$TARGET_HOME/.cache/raven/setup" \
    "$TARGET_HOME/.local" "$TARGET_HOME/.local/share" "$TARGET_HOME/.local/share/raven"; do
    if [[ ! -e $directory ]]; then install -d -m 0755 -o "$TARGET_UID" -g "$TARGET_GID" "$directory"; fi
done
LOG="$TARGET_HOME/.cache/raven/setup/$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1
trap 'printf "[失败] 行 %s；详情：%s；修复后重复执行同一命令可复用缓存。\n" "$LINENO" "$LOG" >&2' ERR
printf '[安装] user=%s repo=%s\n' "$TARGET_USER" "$ROOT"
export DEBIAN_FRONTEND=noninteractive
if [[ -n $OFFLINE_HOST ]]; then
    printf '[离线主机安装] %s\n' "$OFFLINE_HOST"
    offline_apt --simulate
    # Set the storage path before the Docker package starts its service.
    if ! command -v dockerd >/dev/null && [[ -n $DOCKER_DATA_ROOT ]]; then
        [[ ! -e /etc/docker/daemon.json ]] || fail '已有 daemon.json，请先由管理员确认存储设置'
        install -d -m 0711 "$DOCKER_DATA_ROOT"
        install -d /etc/docker
        python3 - "$DOCKER_DATA_ROOT" > /etc/docker/daemon.json <<'PY'
import json, sys
print(json.dumps({'data-root': sys.argv[1], 'features': {'containerd-snapshotter': False}}))
PY
    fi
    offline_apt --assume-yes
else
apt-get update
apt-get install -y --no-install-recommends ca-certificates curl gnupg software-properties-common
add-apt-repository -y universe
apt-get update
apt-get install -y --no-install-recommends git git-lfs python3 python3-venv python3-pip \
    unzip rsync util-linux pciutils procps iproute2 xvfb xauth x11-utils x11-xserver-utils \
    mesa-utils tmux build-essential pkg-config shellcheck
install -d -m 0755 /etc/apt/keyrings
if ! command -v dockerd >/dev/null; then
    curl -fL --retry 5 https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    chmod a+r /etc/apt/keyrings/docker.asc
    printf 'deb [arch=amd64 signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu %s stable\n' "$VERSION_CODENAME" > /etc/apt/sources.list.d/docker.list
    # Configure an explicitly requested data directory before the daemon first starts.
    if [[ -n $DOCKER_DATA_ROOT ]]; then
        [[ ! -e /etc/docker/daemon.json ]] || fail '已有 daemon.json，请先由管理员确认存储设置'
        install -d -m 0711 "$DOCKER_DATA_ROOT"
        install -d /etc/docker
        python3 - "$DOCKER_DATA_ROOT" > /etc/docker/daemon.json <<'PY'
import json, sys
print(json.dumps({'data-root':sys.argv[1], 'features':{'containerd-snapshotter':False}}))
PY
    fi
    apt-get update
    apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
elif ! docker buildx version >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
    # Existing Docker stays in place; use the packages matching its distribution.
    if dpkg-query -W -f='${Status}' docker.io 2>/dev/null | grep -q 'install ok installed'; then
        apt-get install -y docker-buildx docker-compose-v2
    else
        curl -fL --retry 5 https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
        printf 'deb [arch=amd64 signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu %s stable\n' "$VERSION_CODENAME" > /etc/apt/sources.list.d/docker.list
        apt-get update
        apt-get install -y docker-buildx-plugin docker-compose-plugin
    fi
fi
if ! command -v nvidia-ctk >/dev/null; then
    curl -fL --retry 5 https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -fL --retry 5 https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' > /etc/apt/sources.list.d/nvidia-container-toolkit.list
    apt-get update
    apt-get install -y nvidia-container-toolkit
fi
fi
systemctl is-enabled --quiet docker || systemctl enable docker
systemctl is-active --quiet docker || systemctl start docker
install -d /etc/docker
runtime_config=$(mktemp)
trap 'rm -f "$runtime_config"' EXIT
if [[ -f /etc/docker/daemon.json ]]; then cp /etc/docker/daemon.json "$runtime_config"; else printf '{}\n' > "$runtime_config"; fi
nvidia-ctk runtime configure --runtime=docker --config="$runtime_config"
if ! cmp -s "$runtime_config" /etc/docker/daemon.json; then
    [[ -z $(docker ps -q) ]] || fail 'Docker 正在运行其他容器，不能在安装中重启它；请老师在空闲时执行'
    if [[ -f /etc/docker/daemon.json ]]; then cp -a /etc/docker/daemon.json "/etc/docker/daemon.json.raven-backup.$(date +%s)"; fi
    dockerd --validate --config-file="$runtime_config"
    install -m 0644 "$runtime_config" /etc/docker/daemon.json
    systemctl restart docker
fi
getent group docker >/dev/null || groupadd docker
usermod -aG docker "$TARGET_USER"
# A checkout unpacked by the teacher must be writable by the experiment account.
# Do not follow symlinks or change ownership outside this project.
chown -hR "$TARGET_UID:$TARGET_GID" "$ROOT"
for directory in "$TARGET_HOME/.cache/raven" "$TARGET_HOME/.cache/huggingface" "$TARGET_HOME/docker/isaac-sim/6.0"; do
    install -d -o "$TARGET_UID" -g "$TARGET_GID" "$directory"
done
as_user() { runuser -u "$TARGET_USER" -- env HOME="$TARGET_HOME" USER="$TARGET_USER" LOGNAME="$TARGET_USER" "$@"; }
as_user docker info --format 'Docker Root Dir={{.DockerRootDir}}'
as_user docker buildx version
as_user docker compose version
if [[ -n $OFFLINE_HOST ]]; then
    for command in nvidia-ctk Xvfb xauth xdpyinfo git-lfs python3; do
        command -v "$command" >/dev/null || fail "主机依赖未安装完整：$command"
    done
    as_user docker info --format 'GPU runtimes={{json .Runtimes}}'
    printf '\n[主机环境完成] Docker、GPU runtime 和学生权限已配置；没有拉取镜像或下载模型。\n'
    printf '%s 重新连接 SSH 后可以免 sudo 导入离线镜像和资源。\n' "$TARGET_USER"
    printf '实验尚未验收：必须先导入镜像、模型和 Isaac 本地资产，再运行 run_raven_experiment.sh --check。\n'
    printf '安装日志：%s\n' "$LOG"
    exit 0
fi
as_user bash "$ROOT/build_raven_images.sh" --prepare-runtime
as_user bash "$ROOT/run_raven_experiment.sh" --check
deployment=$(as_user bash "$ROOT/run_raven_experiment.sh" --deployment-config)
scene=$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["scene"])' "$deployment")
as_user env RAVEN_SMOKE_SCENE="$scene" bash "$ROOT/smoke_isaac6.sh"
printf '\n[完成] 所有安装及检查通过。%s 重新连接 SSH 后执行（无需 sudo）：\n' "$TARGET_USER"
printf 'cd %q && bash ./run_raven_experiment.sh\n' "$ROOT"
printf '安装日志：%s\n' "$LOG"
