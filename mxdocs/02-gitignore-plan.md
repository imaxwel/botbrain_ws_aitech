# 02 .gitignore 规划

根目录 `/data/unitree/botbrain_ws/.gitignore`（新建，仓库根 .gitignore）：

```gitignore
# ===== 敏感配置 =====
.env
.env.*
!.env.example
botbrain_ws/robot_config.yaml
botbrain_ws/robot_config.yaml.bak
*.bak
*.bak.*

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

# ===== 设备相关编译产物 / 运行时数据（已确认 ignore） =====
**/yolo11n.engine
botbrain_ws/src/g1_pkg/maps/rtabmap.db

# ===== 系统/编辑器杂项 =====
.DS_Store
*.swp
*~

# ===== 日志与运行时数据 =====
**/log/
*.log
```

## 说明与取舍

1. **`**/build/` 是否会误伤源码目录？**
   检查过 src 下没有名为 `build` 的源码目录（只有编译产物目录），此规则安全。但 `unitree_dex3_cpp/build` 属于嵌套编译产物，会被这条规则命中排除，符合预期。

2. **`deps/open3d141/` 整体排除**：这是预编译第三方库（576M），不属于自己的代码，建议在 README 里写明如何获取/编译（版本、下载地址或构建脚本），而不是入库。

3. **yolonas_ocr 整体忽略**（已确认，见 03 文档）：不做 submodule，也不保留其内容/历史，已在上面的规则中直接排除整个目录。

4. **大文件是否要 ignore？**
   本方案倾向于用 Git LFS 管理二进制大文件（mesh、模型权重、pdf/gif 文档），而不是直接 ignore，因为这些是项目实际会用到的资源，不像 build 产物可以随时重新生成。哪些文件用 LFS，见 `03-nested-repos-and-lfs.md`。

5. **fast_lio 的 .gitignore/.gitmodules 如何处理**：
   `fast_lio` 目录内已经带有自己的 `.gitignore` 和 `.gitmodules`（引用 `hku-mars/ikd-Tree`），但 `include/ikd-Tree` 目录内容已经是展开的源码（88K，非 submodule 状态，没有 `.git`）。建议保留现状作为普通目录纳入主仓库（当作 vendored 代码），在文档中注明其上游来源，不强行恢复成 submodule，避免破坏现有可编译状态。

## frontend/.gitignore 现状复核

已存在且规则完善（覆盖 node_modules、.next、.env* 等），无需改动，主仓库根 .gitignore 里的等价规则是为了防止在根目录直接执行 `git add .` 时的双重保险，两者不冲突。

## 待执行

- [ ] 在 `/data/unitree/botbrain_ws/.gitignore` 写入以上内容
- [ ] 确认 `frontend/.gitignore` 保持不变
