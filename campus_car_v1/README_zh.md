# Campus Car V1：P1放车与校园Mesh对齐

本阶段只验证一件事：在txy自己的 Isaac Sim 5.1 Headless环境中加载灰色校园Mesh，将程序生成的通用四轮车放到P1附近并自动贴地。

## 不包含的内容

- 不加载3DGS或NuRec；
- 不运行完整七点轨迹；
- 不建立轮胎、电机、悬架或强化学习控制器；
- 不依赖RealVNC窗口；
- 不读取或修改lzh的Python/Conda环境；
- 不修改原来的 `campus_g1_v2` 或G1工程。

## 服务器安装

把压缩包传到服务器项目目录，然后执行：

```bash
cd /mnt/16T_2/txy/campus_robot/project
tar -xzf campus_car_v1_bundle.tar.gz
cd campus_car_v1
chmod +x scripts/*.sh
```

## 一键运行

```bash
cd /mnt/16T_2/txy/campus_robot/project/campus_car_v1
bash scripts/run_placement.sh
```

运行脚本内部已经保存完整日志，不需要再接外部 `tee`。固定Python路径为：

```text
/mnt/16T_2/txy/envs/unitree_sim_51/bin/python
```

脚本会验证版本必须为 `5.1.x`，且不会扫描或使用 `/home/lzh`。

## 输入

- `config/waypoints.csv`：七个近似路径点。第一阶段只使用P1的XY和P1→P2方向。
- `config/placement.json`：校园资产、探地参数、车体尺寸及输出路径。

原始Z只保留用于审计。小车实际Z由校园Mesh三角面重新计算。

## 输出

```text
outputs/placement_p1/
├── placement.json
├── ground_probe_diagnostics.json
├── close.png
├── side.png
└── top.png

stages/
└── campus_car_p1.usd

logs/
└── 01_place_car_p1.log
```

成功时日志末尾出现：

```text
CAMPUS_CAR_P1_PLACEMENT_PASS
CAMPUS_CAR_P1_PASS
```

## 自动验收

- P1的XY必须落在校园Mesh包围盒内；
- P1半径1.5 m内必须找到坡度不超过30度的三角面；
- 车轮最低点不得低于地面；
- 车轮与地面间隙不得超过5 cm；
- 车头朝向P2；
- 输出三张检查图和可复用USD Stage。

找不到可靠地面时脚本会停止，不会猜一个Z继续运行。候选面、场景包围盒及失败原因保存在 `ground_probe_diagnostics.json`。

## 第二阶段：七点联合探地

P1放置通过后运行：

```bash
bash scripts/run_ground_waypoints.sh
```

它会在每点2.5 m范围内搜索，并按高度分层保留平坦地面候选，避免道路候选被屋顶、树木或破碎表面挤掉；再通过相邻高度和局部坡度选择连续地面高度。七个原始XY被严格锁定，附近Mesh候选只提供Z，绝不再把路线横向移动到人行道或小路。最大路面坡度为15度。旧路径的绝对Z不会用于排除路面；已经验证的P1地面 `z=-0.4627926` 是垂直基准，其余Z仅作为低权重的相对起伏提示。输出：

```text
outputs/ground_waypoints/ground_waypoints.csv
outputs/ground_waypoints/ground_waypoints_diagnostics.json
outputs/ground_waypoints/route_overview.png
outputs/ground_waypoints/route_side.png
stages/campus_ground_waypoints.usd
```

成功标志：`CAMPUS_GROUND_WAYPOINTS_PASS`。

## 第三阶段：逐帧移动小车

```bash
bash scripts/run_car_animation.sh
```

该步骤按15 FPS生成18秒、共271帧的运动轨迹。每一帧都有独立的位置、航向和贴地高度。整车仍采用第一阶段约定的运动Xform，这是规定轨迹的运动复现，不宣称为轮胎动力学仿真。输出：

```text
outputs/car_animation/trajectory_frames.csv
outputs/car_animation/animation_summary.json
stages/campus_car_animated.usd
```

在Isaac Sim时间轴播放 `campus_car_animated.usd` 即可看到小车逐帧运动。成功标志：`CAMPUS_CAR_ANIMATION_PASS`。

## 第四阶段：车载第一视角视频

```bash
bash scripts/run_first_person.sh
```

相机固定在 `/World/Car/FirstPersonCamera`，因此自动继承小车全部逐帧Pose。脚本输出271张960x540 RGB图并自动编码为：

```text
outputs/first_person/campus_car_first_person.mp4
```

成功标志：`CAMPUS_CAR_FIRST_PERSON_PASS`。

## 第五阶段：3DGS车载相机五帧校准

```bash
bash scripts/run_3dgs_camera_preview.sh
```

先只渲染0%、25%、50%、75%、100%五个位置，输出 `outputs/3dgs_camera/camera_preview_strip.png`。确认方向、位置和高度后再渲染完整271帧，避免坐标错误时浪费时间。

## 第六阶段：完整3DGS第一视角视频

```bash
MAX_GAUSSIANS=5000000 bash scripts/run_3dgs_full_video.sh
```

支持断点续跑，已存在且有效的PNG帧会自动跳过。锁定路线版本使用独立输出目录，避免误用旧路线缓存；最终输出 `outputs/3dgs_locked_route/campus_3dgs_first_person.mp4`。

## 第八阶段：原始HTML七关键帧严格预览

```bash
MAX_GAUSSIANS=11879665 bash scripts/run_html_reference_preview.sh
```

七组position、target与垂直FOV 75度从HTML原样保存，不使用车辆Pose、Mesh探地或车载相机偏移。输出 `outputs/html_reference/camera_preview_strip.png`。

## 重要坐标约定

```text
坐标系：Isaac/Usd
单位：米
上方向：+Z
四元数顺序：x,y,z,w
车体+X：前方
```

## 第九阶段：人工采集准确道路路线

旧七点来自HTML相机轨迹，不是车辆道路轨迹。要重新采集准确路线，先在服务器生成带世界坐标的俯视地图和单文件采集器：

```bash
bash scripts/run_route_picker.sh
```

成功后下载并双击打开彩色完整版：

```text
outputs/route_collection/route_picker_color.html
```

使用方法：

1. 沿目标大路中心按行驶顺序左键点选；
2. 直线段每2–4米一个控制点，转弯处每1–2米一个点；
3. 鼠标滚轮缩放，右键拖动，按钮可撤销或清空；
4. 红色虚线只是旧HTML七点参考，不参与新路线导出；
5. 导出 `route_control_points.csv` 后上传回服务器。

底图来自原始彩色3DGS PLY的SH0颜色，而不是无纹理灰色Mesh；默认自动覆盖高斯分布1%到99%的完整有效校园范围，最长边最高4096像素，并支持在浏览器中缩放。CSV中的 `x_world,y_world` 是由地图像素和已知世界边界精确换算得到的。后续探地只能补充Z，不允许更改XY。输出还包含 `route_map_metadata.json`，记录边界、像素分辨率、方向和地图SHA256，便于审计。

当前正式路线保存在 `config/route_control_points.csv`。控制点只定义道路中心XY与期望速度；程序会按固定空间间距自动加密，用户不需要在长直路上手工增加大量点。

## 第十阶段：锁定XY探地并生成车辆动画

```bash
bash scripts/run_control_route.sh
```

该步骤读取人工采集的控制点，以0.25米间距加密。所有输出XY严格等于人工折线的空间插值，Mesh只能提供Z。路线走廊没有直接地面交点的少量样本会标记为 `INTERPOLATED_Z`，不会横向寻找并替换XY。主要输出：

```text
outputs/control_route_ground/grounded_control_route.csv
outputs/control_route_ground/route_xy_check.png
outputs/control_route_ground/route_z_profile.png
outputs/control_route_ground/summary.json
outputs/control_route_animation/trajectory_frames.csv
outputs/control_route_animation/animation_summary.json
stages/campus_car_control_route.usd
```

高度剖面出现尖峰时先不要渲染完整视频，运行9帧沿途诊断预览：

```bash
bash scripts/run_control_route_preview.sh
```

查看 `outputs/control_route_preview/control_route_preview_sheet.png`，用于判断小车在哪些路段悬空、入地或落在错误表面。

正式路线的地面高度同时保存 `raw_ground_z` 与最终 `ground_z`。前者用于审计Mesh原始探测，后者经过约8米中值去碎片、约5米低通以及8%最大道路坡度约束；该处理不修改任何XY。

## 第十二阶段：新路线高质量3DGS车载视频

```bash
bash scripts/run_control_3dgs_hq_video.sh
```

默认使用过滤后的1000万高斯、1280x720、12 FPS和H.264 CRF 16，支持断点续跑。输出：

```text
outputs/control_3dgs_hq/campus_control_route_3dgs_hq.mp4
```

可通过环境变量覆盖，例如 `MAX_GAUSSIANS=12000000 VIDEO_WIDTH=1600 VIDEO_HEIGHT=900`，但单张RTX 3090若显存不足，应先回退到默认1000万。

当源PLY只有SH0时，先用路线局部全量高斯做5帧清晰度对比：

```bash
bash scripts/run_control_3dgs_local_preview.sh
```

它默认裁剪车辆路线周围120米，并在该范围保留全部有效高斯，不进行全校园随机抽样。查看 `outputs/control_3dgs_local_preview/camera_preview_strip.png`；确认优于旧视频后，使用 `MAX_GAUSSIANS=0 ROUTE_CROP_MARGIN=120 bash scripts/run_control_3dgs_hq_video.sh` 重跑完整视频。

对比相机高度是否改善航拍3DGS的地面视角：

```bash
bash scripts/run_camera_height_comparison.sh
```

输出 `outputs/camera_height_comparison/camera_preview_strip.png`，依次包含低1.08米、中2.5米和高4米三组，每组5个路线位置。高相机同时分别向下俯视0、10和18度。

## Git更新方式

工程源码通过Git管理；`outputs/`、`logs/`、`stages/`、视频和图片属于服务器运行产物，已由 `.gitignore` 排除。服务器首次获取或后续更新都应使用仓库分支，不再解压增量压缩包。

## 最小直行物理Demo

该Demo复用当前简单四轮车和正式路线第一段，只验证5米直行。车辆由四个旋转关节的速度驱动，程序不会逐帧改写车辆根Pose。碰撞地面是一条由校园Mesh探测高度生成的窄路带，不对整个校园Mesh开启碰撞。

默认运行物理仿真并生成3DGS五帧预览：

```bash
bash scripts/run_straight_physics_demo.sh
```

仅运行物理直行、跳过3DGS：

```bash
RENDER_3DGS_PREVIEW=0 bash scripts/run_straight_physics_demo.sh
```

五帧方向确认正确后，生成完整短视频：

```bash
RENDER_ALL_3DGS=1 bash scripts/run_straight_physics_demo.sh
```

可以覆盖距离和速度，例如：

```bash
DEMO_DISTANCE_M=3 DEMO_SPEED_MPS=0.20 bash scripts/run_straight_physics_demo.sh
```

主要输出：

```text
outputs/straight_physics_demo/trajectory_frames.csv
outputs/straight_physics_demo/summary.json
stages/campus_car_straight_physics.usd
outputs/straight_physics_3dgs/camera_preview_strip.png
outputs/straight_physics_3dgs/straight_physics_3dgs.mp4
```

成功标志：`CAMPUS_STRAIGHT_PHYSICS_DEMO_PASS`。
