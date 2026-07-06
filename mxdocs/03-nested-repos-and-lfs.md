# 03 嵌套 Git 仓库处理 与二进制资源处理

> **最终方案变更记录**：原计划对模型权重/网格/文档等二进制文件使用 Git LFS 管理，执行过程中确认改为**彻底不要这些二进制资源**，全部通过 `.gitignore` 排除，仓库不再使用 Git LFS（已执行 `git lfs uninstall` 并删除 `.gitattributes`）。以下内容按最终方案更新。

## 3.1 嵌套 .git 目录处理

扫描结果，`botbrain_ws/src` 下有两个嵌套 `.git`：

| 路径 | 状态 | 处理方式 |
|---|---|---|
| `src/fcl/.git` | 空/损坏（0 个 object，HEAD/refs 均为空壳，git 命令报 "not a git repository"） | 直接 `rm -rf` 删除这个 `.git` 目录，fcl 源码本身作为普通文件纳入主仓库 |
| `src/g1_right_dex3/yolonas_ocr/.git` | 有效仓库，remote 指向 `https://github.com/SatArw/yolonas_ocr.git`，当前在 main 分支，有大量本地删除但未提交的文件变更（`results/`、`test/` 下的图片被删除但未 commit） | **确认结果：整个 `yolonas_ocr` 目录不入库，直接 `.gitignore` 排除**（不保留本地删除状态，不做 submodule） |

### yolonas_ocr 处理（已确认：整体忽略）

不需要保留其内容或历史，也不需要 submodule 化。直接在 `.gitignore` 中排除整个目录即可，目录本身留在磁盘上正常使用，只是不进 git 仓库。

```gitignore
# .gitignore 中加入
botbrain_ws/src/g1_right_dex3/yolonas_ocr/
```

不需要额外的 `rm`、`mv`、`submodule add` 操作，磁盘上现有目录原样保留。

### fcl 目录处理

```bash
rm -rf /data/unitree/botbrain_ws/botbrain_ws/src/fcl/.git
```
删除后 fcl 作为普通源码目录纳入主仓库（无需 submodule，因为这个 .git 本身是空壳，不含任何有效历史）。

## 3.2 二进制资源处理（最终方案：彻底不要，不使用 LFS）

机器上安装了 `git-lfs 3.0.2`，最初计划用它管理大文件，但执行过程中确认：**这些二进制资源全部不入库**，仓库只保留代码和脚本。因此撤销了 LFS 配置：

```bash
git lfs uninstall           # 已执行，移除仓库级 LFS hooks
rm -f .gitattributes         # 已执行，删除 LFS track 记录
```

改为在 `.gitignore` 中用模式规则整体排除，不再逐个 track。

### 排除的文件类型与对应 .gitignore 规则

| 类型 | 规则 | 示例文件 |
|---|---|---|
| AI 模型/权重 | `**/*.pt` `**/*.pth` `**/*.onnx` `**/*.tflite` `**/*.pb` `**/*.h5` `**/*.caffemodel` `**/*.engine` `**/Ultralytics/` | src/bot_yolo/models/yolo11n.{pt,onnx,engine} |
| 3D 模型/网格 | `**/*.STL` `**/*.stl` `**/*.dae` `**/*.obj` `**/*.step` `**/*.stp` | src/bot_description/meshes/botbrain.obj、各 torso_link*.STL |
| 地图/点云（运行时数据） | `**/*.pgm` `**/*.pcd` `**/*.ply` `**/rtabmap.db` | src/g1_pkg/maps/rtabmap.db、accumulated.pgm |
| 大型文档/演示媒体 | `**/*.gif` `**/*.pdf` | src/fast_lio/doc/*.gif、*.pdf |
| 第三方库自带测试数据/文档（追加确认） | 见下方专项规则 | fcl 测试用例、fast_lio doc 目录、joystick-bot docs 目录 |
| 3D 打印/CAD 交换文件（追加确认） | `**/*.3mf` | hardware/**/*.3mf |

### 追加确认：第三方库自带的测试数据与文档演示资源

首次 `git add` 后发现 `.git` 体积仍有 336M，排查后发现是几个第三方 ROS 包自带的非代码资源（不是自己写的代码），确认一并忽略：

```gitignore
# 第三方库自带的测试数据/文档演示图片视频（已确认 ignore）
botbrain_ws/src/fcl/test/fcl_resources/
botbrain_ws/src/fast_lio/doc/
botbrain_ws/src/joystick-bot/docs/

# 3D 打印/CAD 交换文件
**/*.3mf
```

涉及文件举例：`fcl/test/fcl_resources/**`（FCL 碰撞检测库测试用例，单个最大 5.9M）、`fast_lio/doc/results/*.png`（README 展示图，单个最大 5M）、`joystick-bot/docs/images/running_example.mp4`（4.5M）、`hardware/**/*.3mf`（3D 打印文件）。

补齐这条规则后重新 `git add .`，`.git` 从 336M 降到 **12M**。

### 已确认事项汇总

1. **`yolo11n.engine`**：ignore。部署文档里写明"用 `yolo11n.pt` 通过 export 命令在目标设备上重新生成"。
2. **`g1_pkg/maps/rtabmap.db`**：ignore，运行时数据。
3. **`yolonas_ocr`**：整个目录 ignore，不保留、不 submodule。
4. **所有模型权重/网格/地图/文档演示类二进制文件**：彻底不要，不使用 LFS，全部 ignore。

## 已完成

- [x] 确认 yolo11n.engine 处理方式 — ignore
- [x] 确认 rtabmap.db 处理方式 — ignore
- [x] 确认 yolonas_ocr 处理方式 — 整体 ignore
- [x] 删除 fcl 的空壳 .git
- [x] 撤销 Git LFS（`git lfs uninstall` + 删除 `.gitattributes`），改用 `.gitignore` 规则整体排除二进制资源
- [x] 追加排除第三方库自带的测试数据/文档演示资源（fcl/test、fast_lio/doc、joystick-bot/docs、*.3mf）
