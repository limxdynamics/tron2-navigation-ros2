# LimX ROS 2 导航集成

[English](README.md) | [中文](README_zh-CN.md)

本仓库公开导航集成中采用宽松许可证（permissive license）的部分：

- LimX 自有的 ROS 2 集成、部署、安全与测试代码，适用顶层 Apache-2.0 许可；
- 生产用 ROS 2 SCAN-Planner 子集，适用 Apache-2.0 —— 其 ROS 2 移植层是外部社区贡献，
  LimX Dynamics 的生产集成建立在该层之上；
- RoboSense `rslidar_sdk`、内嵌的 `rs_driver` 与 `rslidar_msg`，适用 BSD-3-Clause。

`FAST_LIO`、`FAST_LIO_LOCALIZATION2`、PCT_planner 及其内嵌代码与 Git 历史**刻意未包含**在本
仓库内，它们属于外部 GPL 依赖。相关说明见
[外部 GPL 依赖](docs/GPL_EXTERNAL_DEPENDENCIES.md)、[许可范围](LICENSING.md)、
[第三方声明](THIRD_PARTY_NOTICES.md)与[硬件兼容性](docs/HARDWARE_COMPATIBILITY.md)。

## 仓库范围

顶层 Apache-2.0 许可**仅**覆盖版权归 LimX Dynamics Technology Co., Ltd. 所有的自研集成文件。
每个第三方组件保留自己的 LICENSE 与 NOTICE。本仓库不分发任何 GPL 源码、GPL 头文件、
GPL Python 扩展、GPL 二进制、地图、录包、构建产物、部署快照、站点凭据、内网地址或机器人标识。

被排除的 GPL 程序在部署系统中是相互独立的 ROS 2 进程。跨组件集成通过 ROS 2/DDS 消息与
有据可查的文件完成，并未把 GPL 库链接进本仓库所含的 SCAN 或 RoboSense 二进制。
这一工程上的分离本身并不构成对「聚合作品 / 衍生作品」的法律判定。

### SCAN-Planner 归属

`SCAN-Planner-ros2-community/` 由两部分归属不同的贡献组成。其 ROS 2 移植层 ——
ROS 2 Humble、ament_cmake、colcon、rclcpp、tf2_ros、RViz2 以及 launch 集成 ——
由外部个人贡献者 `xiaoqi371317` 独立完成，并作为上游
<https://github.com/wuyi2121/SCAN-Planner> 的 `ros2-community` 分支发布。
该贡献者不是 SCAN-Planner 的原始研究作者，与 LimX Dynamics 亦无隶属关系。
建立在该层之上的生产集成属于 LimX Dynamics 自有工作，适用顶层 Apache-2.0 许可。
文件级归属划分与 Apache-2.0 第 4(b) 条修改声明见
[SCAN-Planner-ros2-community/NOTICE](SCAN-Planner-ros2-community/NOTICE)。

## 不含 GPL 依赖时可构建的内容

在 Ubuntu 22.04 aarch64 + ROS 2 Humble 上：

```bash
./install.sh nx --bootstrap --jobs 4 --with-tests
```

若 NX 上的 Humble 环境已就绪，可去掉 `--bootstrap`；只做只读环境检查时加 `--preflight-only`。
该流程只构建并校验仓库内包含的 RoboSense 与生产 SCAN 包。它不会下载 GPL 代码、不会启动导航、
不会连接底盘，也不会发送速度指令。

仓库内的 WebSocket 桥接测试使用本地桩与 RFC 5737 文档地址，因此其协议校验不需要机器人硬件。

## 完整导航链路需要外部 GPL 程序

完整的生产链路为：

```text
RoboSense (BSD-3-Clause)
  -> 外部 FAST-LIO/定位 (GPL-2.0-only)
  -> 外部 PCT 全局规划器 (GPLv2-or-later)
  -> SCAN 局部规划/控制 (Apache-2.0)
  -> LimX 既有 WebSocket 网关 (Apache-2.0)
```

### v1.1.0 集成变更

- 单次 FAST-LIO 建图会话会以可回滚的事务方式，同时保存全量定位 PCD 与近场 PCT 源 PCD；
- 由一份清单绑定两个 PCD 的文件名、SHA-256 值、坐标系、量程与建图会话，地图激活时会拒绝
  混搭或被改动过的文件对；
- FAST-LIO 配准使用各自的近场上限（默认 15 m），而完整定位地图可保留 100 m 数据；
- PCT 转换接受显式分辨率，包括在 16 GiB NX 上对 0.2 m 单体素图过大的地图使用 0.4 m；
- `navi.sh nav --no-obstacle-avoidance --speed MPS` 是显式的高危维护模式，以暂停状态启动、
  速度上限 0.60 m/s，且需要另行输入 `NO_OBSTACLE_GO` 确认；
- `--stair-centerline` 会额外启用经验证的 PCT 楼梯中线算法，且在该维护模式未激活时会被拒绝。

无避障模式会刻意关闭局部点云订阅，不得当作常规导航的默认配置。常规导航始终保持局部避障开启。

兼容的 GPL 源码单独发布，以保持其 GPL 条款完整：

| 运行单元 | 仓库 | 提交 | 标签 |
|---|---|---|---|
| FAST-LIO 建图与定位 | <https://github.com/limxdynamics/tron2-navigation-fastlio-gpl> | `0fbf6e9cca72a66c330a22d13823f52d49f05948` | `v1.1.0` |
| PCT 全局规划器 | <https://github.com/limxdynamics/tron2-navigation-pct-gpl> | `1c553202e50798819715679a7f8c8a878b247ace` | `v1.1.0` |

上游克隆地址与基线提交记录在
[GPL_EXTERNAL_DEPENDENCIES.md](docs/GPL_EXTERNAL_DEPENDENCIES.md)。
请勿用上述上游基线替代上方列出的兼容衍生提交。

### 推荐：自动获取

普通的 `git clone` 从不执行仓库内的脚本，因此只克隆主仓库并不会立即访问那两个 GPL 仓库。
请先克隆主仓库，再使用仓库内附带的获取脚本：

```bash
mkdir tron2-navigation-workspace
cd tron2-navigation-workspace
git clone https://github.com/limxdynamics/tron2-navigation-ros2.git
cd tron2-navigation-ros2
./deployment/link_external_gpl_sources.sh --fetch --verify-only
```

获取脚本会在主克隆的同级目录下建立两个缺失的公开 GPL 克隆，检出各自固定的 `v1.1.0` 标签，
校验精确提交与源码清单，并保持已存在的路径不变。最终的目录结构为：

```text
tron2-navigation-workspace/
  tron2-navigation-ros2/
  tron2-navigation-fastlio-gpl/
  tron2-navigation-pct-gpl/
```

在已准备好的 Ubuntu 22.04 aarch64 + ROS 2 Humble 系统上，用以下命令构建全部三个源码单元：

```bash
./install.sh full-nx --jobs 4 --with-tests
```

直接运行 `full-nx` 时，若任一 GPL 仓库缺失，也会先行获取再构建。在全新的受支持系统上，
可把系统依赖的安装一并交给构建流程：

```bash
./install.sh full-nx --bootstrap --jobs 4 --with-tests
```

全新刷机的 NX 请使用这一 `--bootstrap` 形式，而不是面向已准备就绪 NX 的形式。它会显式安装
干净 NX 部署过程中所需的 Open3D、transforms3d、websocket-client、PCL、BLAS/LAPACK、
编译器、ROS 2 与测试依赖，随后在独立的 `.map-tools/site` 目录中安装 CUDA PCT 地图工具。
安装器会在编译前校验所需的 Python/ROS 模块并运行一次小规模 CuPy 计算，因此运行环境不完整会在
安装阶段就失败，而不是等到部署之后。

对已经下载过仓库的环境，用 `--preflight-only` 做只读检查。安装器只创建被忽略的本地兼容软链
与各自独立的 install 前缀。它不会启动导航，也不会下发底盘指令。

### 备选：手动克隆

若改为手动下载依赖，请在 `tron2-navigation-ros2` 的同级目录执行：

```bash
git clone --branch v1.1.0 https://github.com/limxdynamics/tron2-navigation-fastlio-gpl.git
git clone --branch v1.1.0 https://github.com/limxdynamics/tron2-navigation-pct-gpl.git
cd tron2-navigation-ros2
./deployment/link_external_gpl_sources.sh --verify-only
```

## 硬件与固件前提

没有经授权的兼容 LimX 硬件、固件、信令服务以及有效分配的机器人标识，就无法进行完整的
底盘作业。见 [HARDWARE_COMPATIBILITY.md](docs/HARDWARE_COMPATIBILITY.md)。

仅可在目标机器人上把公开模板复制为未被跟踪的站点文件：

```bash
cp config/navigation.env.example config/navigation.env
${EDITOR:-vi} config/navigation.env
```

运行端点与机器人标识**没有默认值**。底盘输出初始为禁用且暂停状态。
LimX 既有 WebSocket 网关是唯一受支持的速度通路；旧的 MROS 指令转发与协议探测不在本仓库内。

源码交付不包含定位/PCT 地图、端点或标识取值、授权、标定、ROS 2、CUDA 或已编译产物。
安装完成后，需先提供站点配置，并生成配对的匹配地图，方可启动导航。

在经授权的兼容硬件上，常规的构建后流程为：

```bash
./navi.sh map
./navi.sh save
./navi.sh pct
./navi.sh nav
# 确认定位、路径、安全间隙与急停均正常之后：
./navi.sh go
```

用 `./navi.sh pause`、`./navi.sh cancel`、`./navi.sh stop` 分别暂停运动、取消目标、停止整条
导航栈。在站点配置、匹配地图对、定位、唯一指令通路以及物理安全项全部通过之前，请勿运行 `go`。

## 安全约定

- 启动即为暂停状态；
- 端点/标识缺失或格式错误时按失败安全处理（fail closed）；
- 强制速度与偏航限幅；
- 通信超时或断连会锁定暂停，并持续发送零速度；
- 拒绝旧的指令桥与重复网关；
- 恢复运行需要真实终端，并输入大写的 `GO`；
- 安装与测试绝不启动导航，也不发送非零速度。

## 外部运行时集成

在兼容的外部 GPL 仓库已检出到预期的同级目录、并构建进各自独立的 install 前缀之后，
仓库内既有的集成脚本可通过 ROS 2 topic 对它们进行编排。关键接口如下：

| 生产者 | 消费者 | 接口 |
|---|---|---|
| RoboSense | FAST-LIO | `/rslidar_points`（`sensor_msgs/PointCloud2`）与 `/rslidar_imu_data`（`sensor_msgs/Imu`） |
| 定位 | PCT | `/pose_stamped`（`geometry_msgs/PoseStamped`） |
| 定位 | SCAN | `/pose_stamped` 与 `/corrected_current_pcd` |
| PCT | SCAN | `/pct_path`（`nav_msgs/Path`） |
| SCAN 控制器 | LimX 网关 | `/sdk_cmd_vel`（`geometry_msgs/Twist`） |

地图、坐标系、QoS、时间戳与版本兼容性须由部署方自行校验。
PCD/PCT 地图属于运行时数据，本仓库刻意不提供。

## 源码完整性

针对解包后的发布副本运行范围内的源码检查：

```bash
./deployment/audit_permissive_source.sh /path/to/extracted/repository
```

该检查会核验源码清单、许可文件、GPL 源码根目录与仿真/示例依赖的缺失情况、隐私占位符、
生成物/二进制文件、路径长度、Markdown 链接、Git 可跟踪性与可执行位。

## 许可证

本仓库中的自研集成代码、部署辅助脚本、测试与文档以 Apache License 2.0 分发，
见 [`LICENSE`](LICENSE)，版权归 LimX Dynamics Technology Co., Ltd. 所有。
SPDX 标识符：`Apache-2.0`。

第三方组件保留各自的许可与声明，其署名记录在
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) 以及各组件目录内的
`LICENSE` / `NOTICE` 文件中。顶层许可的覆盖范围见 [`LICENSING.md`](LICENSING.md)；
单独授权的伴生仓库列于
[`docs/GPL_EXTERNAL_DEPENDENCIES.md`](docs/GPL_EXTERNAL_DEPENDENCIES.md)。
