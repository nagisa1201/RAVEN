# 本地复现实验改动

此 fork 保存截至 2026-11-19 的本地源码和配置改动，保留原项目历史、作者与许可证。它不是已经完成论文指标复现的版本。

## 与原仓库的比较基线

发布前已 fetch 并检查：以下本地基线与相应上游分支一致，均无领先或落后的提交。改动在这些基线上单独提交。

| 仓库 | 上游分支 | 原始提交 | 本 fork 发布分支 |
| --- | --- | --- | --- |
| [castacks/RAVEN](https://github.com/castacks/RAVEN) | main | `4f4aaae9ae8a177b7ea403c1f4bf39967174585a` | main |
| [castacks/AirStack](https://github.com/castacks/AirStack) | raven | `278acbffaf748cd6e0102b3a25cfea544e031c83` | raven |
| [seungchan-kim/RayFronts](https://github.com/seungchan-kim/RayFronts) | raven | `8d838d79e444a035b71865034e4c33020b981fc8` | raven |
| [seungchan-kim/LVLM](https://github.com/seungchan-kim/LVLM) | main | `7e5c652f4c56a4a7f2fe77940d79bf20898dae69` | main |

主仓库的三个子模块地址指向 `nagisa1201` 下的 fork，并固定到包含本地改动的提交。AirStack 内部未修改的子模块继续使用原地址和固定提交。

## 获取和启动

```bash
git clone --recurse-submodules https://github.com/nagisa1201/RAVEN.git
cd RAVEN

# 只预览构建步骤。
./build_raven_images.sh --dry-run
# 按需编辑此脚本顶部配置，再构建。
./build_raven_images.sh

# 编辑场景、任务、LVLM、显存保护、起飞和评估参数。
${EDITOR:-nano} run_raven_experiment.sh
./run_raven_experiment.sh --dry-run
./run_raven_experiment.sh
```

构建入口统一执行 RayFronts 镜像/C++ 扩展、AirStack Robot/Isaac Sim 和可选 LVLM 的构建步骤。宿主依赖安装和场景下载默认关闭，分别由 `INSTALL_HOST_DEPS`、`DOWNLOAD_SCENES` 控制；首次使用仍需准备 NVIDIA 驱动、Container Toolkit、Docker 权限、场景和模型下载所需网络。AirStack Dockerfile 使用 BuildKit 的缓存挂载语法，构建该部分需要可用的 BuildKit/Buildx 环境。Docker 存储目录由宿主 Docker 配置决定，脚本不迁移 Docker 数据。

也可以直接运行 `python3 launch_raven_console.py --help` 查看所有参数。终端统一打印各组件输出；Ctrl+C 保存本轮结果并停止启动器管理的容器。日志默认为 `~/.cache/raven/console/<时间>/`，包含 `all.log`、组件日志、`result.json` 和 `trajectory.jsonl`。原 tmux 启动入口仍保留。

## 保存的功能改动

### 主仓库（发布整理前 5 个源码文件）

- `launch_raven_console.py`：前台统一日志、容器状态与 GPU 监控、任务发布、可选 LVLM、自动起飞/规划、启动互斥、失败清理、按轮记录和结束条件。
- `run_raven_experiment.sh`：集中配置场景、任务、位姿、预算、评分半径、航点半径、显存保护及各启动开关。
- `build_raven_images.sh`：统一镜像构建、C++ 扩展编译验证和构建日志，支持 dry-run，省略旧 Docker 不支持的 `--progress` 参数。
- `start_rayfronts.sh`：检查现有容器挂载并复用容器，持久化模型缓存。
- `launch_raven.sh`：原 tmux 入口调用新的 RayFronts 启动辅助脚本。

自动起飞等待 PX4 pre-arm 检查、真正解锁、起飞完成和路径就绪；飞控状态消息乱序时有同步等待。一旦看到解锁或空中证据，就不再重试解锁。

### AirStack（14 个文件）

- `robot/docker/Dockerfile.robot`：pip 重试/缓存、SymPy 安装兼容、保留 MACVO 所需 matplotlib 并处理 mpl_toolkits 冲突。
- `simulation/isaac-sim/docker/Dockerfile.isaac-ros`：PX4 浅克隆/重试、独立 Python 构建环境、NuttX tag 元数据、Isaac Sim 依赖版本兼容。
- 感知和全局规划 launch XML：增加 MACVO 与随机规划器的环境开关。
- `robot.rviz`：LiDAR QoS/显示、MACVO 话题路径、语义体素改为点显示。
- 9 个场景启动脚本：`RAVEN_HEADLESS` 控制观察窗口，移除旧 `airlab.pegasus` 扩展加载；RetroNeighborhood 另降低观察窗口分辨率。相机和 LiDAR 传感器渲染仍保留。

### RayFronts（3 个文件）

- `rayfronts/behaviors/voxel_behavior.py`：`RAVEN_WAYPOINT_RADIUS` 配置候选航点到达半径，并打印选择/访问记录。直接原始启动默认仍为 3 m，统一启动器默认传入 5 m。
- `rayfronts/visualizers/base.py`：深度图没有有限像素时跳过空张量归一化，保留诊断图。
- `rayfronts/utils.py`：余弦相似度改为二维矩阵乘法，并以 32768 个体素/射线行为一块计算。保留特征精度、归一化、全部查询间的 softmax、输出顺序和梯度；消除广播产生的大型临时张量。数学规则不变，浮点舍入结果不保证逐位一致。

### LVLM（2 个文件）

- `Dockerfile`：固定 `transformers==4.48.3`。
- `baseline.py`：保留本地增加的可执行权限，文件内容未改。

本次发布整理另增加缓存忽略规则、本说明、README 入口，并更新 `.gitmodules` 和子模块提交指针。

## 当前实验参数与评估边界

`run_raven_experiment.sh` 当前保存 ConstructionSite / `cabling winches`，开启 LVLM、无窗口模式、自动起飞和规划，关闭 MACVO 与随机规划器。预算为 600 秒仿真时间，真实目标包围盒评分半径 6 m，候选航点到达半径 5 m。当前显存保护开关为 **false**；需要时将 `ENABLE_VRAM_PROTECTION=true`，阈值由 `MIN_FREE_VRAM` 设置。

Progress 按真实目标实例去重计数。评分条件是估计位置到真实目标有向包围盒的三维距离不超过评分半径。候选航点到达阈值与此独立。预算和距离均为本地验证配置，不能声称是论文作者的原始参数；结果中标记 `paper_exact_reproduction=false`。没有碰撞成功认证或 oracle 最优路径，因此不提供已验证的 PPL。

## 验证记录与已知问题

- 发布前检查：14 个 Python、4 个 Shell、2 个 XML 文件语法通过；现有 28 项自动启动测试和 18 项评估测试通过；构建和实验入口 dry-run 通过。测试文件保留在本机验证缓存中，此次未迁入项目。没有重新构建完整镜像或重新运行整轮仿真。
- 显存优化曾在 RTX 4080 / PyTorch 2.9.1 / CUDA 13.0 中进行 48 组数值测试和梯度检查。250000×512 体素、5 个查询的 float32 合成基准，额外分配峰值从约 5377 MiB 降到 69.6 MiB；最大概率差约 `4.53e-6`。这是算子基准，不代表完整仿真显存都减少同样比例，也不保证长时间运行不会 OOM。
- **尚未修复：卡住后的自动恢复未接通。** 当前自动启动发送 `Global Plan Commanded`，该行为树分支只跟随全局路径；`Stuck → Rewind` 在 `Autonomously Explore Commanded` 分支。已经观察到全部局部轨迹被判为碰撞/未知后长期悬停。尚未确定轨迹被拒绝的最底层原因，此次发布不改变该行为。
- 2026-11-19 14:14:54 启动的 ConstructionSite 轮次最终因 600 秒仿真预算耗尽结束，Progress 0/1；这不是成功导航验证。该轮进程在上述显存优化写入前启动，不能用它证明新算子已经通过整轮验证。

不上传本机 Docker 数据、镜像、模型缓存、编译产物、实验原始日志或凭据。仓库文件之外的驱动、Docker data-root、系统依赖和容器内手工状态不会随 Git 迁移；新机器应重新准备和构建。
