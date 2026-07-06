# BotBrain 工作区 GitHub 备份规划 - 总览

生成时间: 2026-07-06
执行状态: **已完成，已推送**（2026-07-06）
目标仓库: git@github.com:imaxwel/botbrain_ws_aitech.git（SSH 直连，无需代理）
工作区路径: /data/unitree/botbrain_ws （当前总大小 3.9G，入库后 .git 体积 12M）

## 最终结果

- 首次提交 `f88e0c2` 已推送到 `origin/main`
- 1582 个文件纳入版本管理，`.git` 目录 12M
- clone 验证通过，`.env`、`robot_config.yaml` 等敏感文件未出现在克隆结果中
- 二进制资源（模型权重、3D网格、地图点云、第三方库测试数据/文档）最终方案为**彻底不要，全部 .gitignore 排除**，未使用 Git LFS（详见 03 文档的方案变更记录）

## 目标

把以下内容备份进 GitHub：
1. 所有自研/定制代码（ROS2 包、前端、hardware 相关脚本）
2. Docker Compose 及自启动脚本（docker-compose.yaml、docker/Dockerfile.*、botbrain.service、web_server.service、install.sh、robot_select.sh、ros_env.sh 等）
3. 必要的配置文件模板（不含明文密钥/密码）

不备份：
1. 编译临时产物（colcon build/install/log、C++ build 目录、__pycache__、.next、node_modules）
2. 预编译第三方大库（deps/open3d141，576M）
3. 明文密钥/密码（.env、robot_config.yaml 中的敏感字段）
4. 第三方 OCR 子项目 yolonas_ocr（整体忽略，不保留内容/历史）
5. 设备相关的编译产物（TensorRT engine 文件）与运行时数据（rtabmap.db 等地图数据）

## 现状体积分布（关键项）

| 路径 | 大小 | 处理方式 |
|---|---|---|
| botbrain_ws/build | 510M | 排除（.gitignore） |
| botbrain_ws/install | 605M | 排除（.gitignore） |
| botbrain_ws/log | 21M | 排除（.gitignore） |
| botbrain_ws/src | 903M | 保留，其中部分子项单独处理 |
| deps/open3d141 | 576M | 排除，文档记录获取/构建方式 |
| frontend/node_modules | 286M | 排除（已有 frontend/.gitignore 覆盖） |
| frontend/.next | 382M | 排除（已有 frontend/.gitignore 覆盖，需确认根 .gitignore 也生效） |
| frontend/public/robot-models | 135M | 保留（前端必需静态资源），建议 LFS |
| src/g1_right_dex3/yolonas_ocr | 273M（含 .git 110M pack + 150M .pb 模型） | 排除，整个目录忽略（已确认不保留内容/历史） |
| src/g1_right_dex3/unitree_dex3_cpp/build | 79M | 排除（嵌套 build 目录） |
| src/fast_lio/doc/*.gif, *.pdf 及整个 doc/ 目录 | ~98M+ | 排除（第三方库文档演示资源，已确认 ignore） |
| 各 mesh 文件 (*.STL/*.obj/*.dae) | 累计 ~50M+ | 排除（已确认彻底不要，不用 LFS） |
| bot_yolo/models/*.pt, *.onnx | ~16M | 排除（已确认彻底不要，不用 LFS） |
| bot_yolo/models/*.engine (及其他 *.engine) | 8.2M | 排除（TensorRT 编译产物，设备相关，已确认 ignore） |
| g1_pkg/maps/rtabmap.db | 12M | 排除（运行时地图数据，已确认 ignore） |
| src/fcl/test/fcl_resources/、src/joystick-bot/docs/ | 合计数十 MB | 排除（第三方库自带测试数据/演示视频，已确认 ignore） |
| hardware/**/*.3mf | ~1.5M | 排除（3D 打印文件，已确认 ignore） |
| src/fcl/.git | 0（损坏的空仓库残留） | 已删除该 .git 目录，fcl 源码作为普通文件保留 |

最终实际入库 `.git` 体积仅 **12M**，1582 个文件。二进制资源最终决定彻底不要，未使用 Git LFS。

## 执行顺序（对应后续文档）

1. `01-secrets-and-config.md` — 先处理敏感信息，这是不可逆操作前必须做的第一步
2. `02-gitignore-plan.md` — 制定 .gitignore 规则
3. `03-nested-repos-and-lfs.md` — 处理嵌套 git 仓库（fcl 空壳删除、yolonas_ocr 整体忽略）与二进制资源（最终方案：彻底不要，不用 LFS）
4. `04-execution-steps.md` — 实际执行命令清单（git init → 首次提交 → push）
5. `05-restore-and-rebuild.md` — 备份完成后，新机器如何还原/重新构建运行环境

## 风险提示（执行记录）

- 首次 git 化（新增文件，不涉及删除现有数据），密钥处理（`.env`、`robot_config.yaml` 替换为 `.example` 模板）在 `git add` 之前完成，已通过 clone 验证确认未泄露。
- 仓库可见性（private/public）未在本次操作中确认，建议登录 GitHub 手动检查 `imaxwel/botbrain_ws_aitech` 的仓库设置，如为 public 建议改为 private。
