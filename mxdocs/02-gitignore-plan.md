# 02 .gitignore 规划（最终已执行版本）

根目录 `/data/unitree/botbrain_ws/.gitignore`（已写入并生效）：

```gitignore
# ===== 敏感配置 =====
.env
.env.*
!.env.example
botbrain_ws/robot_config.yaml
botbrain_ws/robot_config.yaml.bak
*.bak
*.bak.*
*.bak_*

# ===== ROS2 / colcon 编译产物 =====
botbrain_ws/build/
botbrain_ws/install/
botbrain_ws/log/

# ===== 通用编译/缓存产物 =====
**/build/
**/__pycache__/
**/*.pyc
**/CMakeFiles/
**/CMakeCache.txt
**/*.o
**/*.so
!deps/**/*.so

# ===== 前端 (node/next) =====
frontend/node_modules/
frontend/.next/
frontend/out/
frontend/coverage/
frontend/*.tsbuildinfo

# ===== 第三方预编译依赖，不入库 =====
deps/open3d141/

# ===== 第三方 OCR 子项目，整体不入库（已确认） =====
botbrain_ws/src/g1_right_dex3/yolonas_ocr/

# ===== AI 模型/权重文件，整体不入库（已确认：彻底不要二进制资源） =====
**/*.pt
**/*.pth
**/*.onnx
**/*.tflite
**/*.pb
**/*.h5
**/*.caffemodel
**/*.engine
**/Ultralytics/

# ===== 3D 模型/网格文件，整体不入库（已确认：彻底不要二进制资源） =====
**/*.STL
**/*.stl
**/*.dae
**/*.obj
**/*.step
**/*.stp

# ===== 地图与点云数据（运行时生成，已确认不入库） =====
**/*.pgm
**/*.pcd
**/*.ply
**/rtabmap.db

# ===== 大型文档/演示媒体（已确认不入库） =====
**/*.gif
**/*.pdf

# ===== 系统/编辑器杂项 =====
.DS_Store
*.swp
*~

# ===== 日志与运行时数据 =====
**/log/
*.log

# ===== 第三方库自带的测试数据/文档演示图片视频（已确认 ignore） =====
botbrain_ws/src/fcl/test/fcl_resources/
botbrain_ws/src/fast_lio/doc/
botbrain_ws/src/joystick-bot/docs/

# ===== 3D 打印/CAD 交换文件 =====
**/*.3mf
```

## 说明与取舍

1. **`**/build/` 是否会误伤源码目录？**
   检查过 src 下没有名为 `build` 的源码目录（只有编译产物目录），此规则安全。但 `unitree_dex3_cpp/build` 属于嵌套编译产物，会被这条规则命中排除，符合预期。

2. **`deps/open3d141/` 整体排除**：这是预编译第三方库（576M），不属于自己的代码，建议在 README 里写明如何获取/编译（版本、下载地址或构建脚本），而不是入库。

3. **yolonas_ocr 整体忽略**（已确认，见 03 文档）：不做 submodule，也不保留其内容/历史，已在上面的规则中直接排除整个目录。

4. **二进制资源最终方案：彻底不要，不用 LFS**（方案变更，见 03 文档）。
   最初计划用 Git LFS 管理 mesh/模型权重/pdf/gif 等大文件，执行时确认改为**全部 ignore，不入库**，因此这里直接用模式规则排除，而不是走 LFS track。

5. **fast_lio 的 .gitignore/.gitmodules 如何处理**：
   `fast_lio` 目录内已经带有自己的 `.gitignore` 和 `.gitmodules`（引用 `hku-mars/ikd-Tree`），但 `include/ikd-Tree` 目录内容已经是展开的源码（88K，非 submodule 状态，没有 `.git`）。保留现状作为普通目录纳入主仓库（当作 vendored 代码），不强行恢复成 submodule，避免破坏现有可编译状态。`fast_lio/doc/` 目录本身（文档演示图片/gif/pdf）已被排除，不影响源码。

6. **第三方库自带测试数据/文档（追加确认）**：首次 `git add` 后 `.git` 体积异常（336M），排查发现是 `fcl/test/fcl_resources`（碰撞检测库测试用例）、`fast_lio/doc`（README 展示图）、`joystick-bot/docs`（演示视频）这几个第三方包自带的非代码资源，确认一并 ignore，补齐规则后 `.git` 降到 12M。

## frontend/.gitignore 现状复核

已存在且规则完善（覆盖 node_modules、.next、.env* 等），无需改动，主仓库根 .gitignore 里的等价规则是为了防止在根目录直接执行 `git add .` 时的双重保险，两者不冲突。

## 执行结果

- [x] 已在 `/data/unitree/botbrain_ws/.gitignore` 写入以上完整内容
- [x] `frontend/.gitignore` 保持不变
- [x] 最终 `.git` 体积 12M，1582 个文件纳入版本管理
