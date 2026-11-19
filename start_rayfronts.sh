#!/usr/bin/env bash
# Reuse an existing RayFronts container instead of failing on a name conflict.
set -euo pipefail
RAVEN="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
container=rayfronts_container
environment=(-e NVIDIA_DRIVER_CAPABILITIES=all -e ROS_DOMAIN_ID=1)
for variable in ISAAC_SIM_SCRIPT_NAME DRONE_X DRONE_Y DRONE_Z DRONE_QX DRONE_QY DRONE_QZ DRONE_QW; do
    if [[ -v "$variable" ]]; then
        environment+=(-e "$variable=${!variable}")
    fi
done

if docker container inspect "$container" >/dev/null 2>&1; then
    mount_source=$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/workspace/RayFronts"}}{{.Source}}{{end}}{{end}}' "$container")
    if [[ "$mount_source" != "$RAVEN/RayFronts" ]]; then
        echo "Existing $container mounts a different RayFronts directory: $mount_source" >&2
        exit 1
    fi
    if [[ $(docker inspect --format '{{.State.Running}}' "$container") != true ]]; then
        docker start "$container"
    fi
    if docker exec "$container" pgrep -f '^python3 -m rayfronts.mapping_server_rosnode' >/dev/null; then
        echo "RayFronts mapping server is already running in $container."
        exit 0
    fi
    echo "Starting mapping server in existing $container."
    exec docker exec -it "${environment[@]}" -w /workspace/RayFronts "$container" \
        bash -i /workspace/RayFronts/run_mapping_server_rosnode.sh
else
    cache_root="${XDG_CACHE_HOME:-$HOME/.cache}/raven"
    mkdir -p "$cache_root/torch" "$cache_root/huggingface"
    exec docker run -it --rm --name "$container" --gpus all --network host \
        --ipc host --privileged --runtime=nvidia "${environment[@]}" \
        -v "$RAVEN/RayFronts:/workspace/RayFronts" \
        -v "$cache_root/torch:/root/.cache/torch" \
        -v "$cache_root/huggingface:/root/.cache/huggingface" -w /workspace/RayFronts \
        rayfronts:desktop bash -i /workspace/RayFronts/run_mapping_server_rosnode.sh
fi
