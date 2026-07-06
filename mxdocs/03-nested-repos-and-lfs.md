# 03 嵌套 Git 仓库处理 与 Git LFS 规划

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

## 3.2 Git LFS 规划

机器上已安装 `git-lfs 3.0.2`，建议纳管以下几类文件：

```bash
git lfs install   # 在主仓库根目录执行一次

git lfs track "*.STL"
git lfs track "*.stl"
git lfs track "*.obj"
git lfs track "*.dae"
git lfs track "*.pb"
git lfs track "*.onnx"
git lfs track "*.engine"
git lfs track "*.pt"
git lfs track "*.db"
git lfs track "*.gif"
git lfs track "*.pdf"
```

执行后会生成/更新 `.gitattributes`，需要把这个文件加入首次提交。

### 具体涉及的文件（供参考核对）

| 文件 | 大小 | 类型 |
|---|---|---|
| src/fast_lio/doc/real_experiment2.gif | 50M | 演示动图 |
| src/fast_lio/doc/ulhkwh_fastlio.gif | 35M | 演示动图 |
| src/fast_lio/doc/Fast_LIO_2.pdf | 13M | 论文文档 |
| src/bot_description/meshes/botbrain.obj | 17M | 模型网格 |
| src/g1_right_dex3/unitree_g1_dex3_stack/robots/g1_description/meshes/torso_link_23dof_rev_1_0.STL | 7.5M | 模型网格 |
| src/g1_pkg/meshes/torso_link_23dof_rev_1_0.STL | 7.5M | 模型网格（重复文件，多个包共用同一个 mesh，可考虑后续去重，本次不处理） |
| src/g1_manipulation_pkg/description_files/meshes/torso_link_23dof_rev_1_0.STL | 7.5M | 同上 |
| src/go2w_pkg/meshes/base.dae | 11M | 模型网格 |
| src/go2_pkg/meshes/trunk.dae | 11M | 模型网格 |
| src/bot_yolo/models/yolo11n.onnx | 11M | 模型权重 |
| src/bot_yolo/models/yolo11n.engine | 8.2M | 模型权重（TensorRT engine，通常是设备相关的编译产物，建议评估是否真的要入库，还是运行时在设备上重新生成） |
| src/bot_yolo/models/yolo11n.pt | 5.4M | 模型权重 |
| src/g1_pkg/maps/rtabmap.db | 12M | 运行时地图数据库，**已确认：按运行时数据处理，ignore，不入库** |

### 已确认事项

1. **`yolo11n.engine`**：已确认不保留，加入 `.gitignore`（仅列出路径，不做 LFS）。部署文档里写明"用 `yolo11n.pt` 通过 export 命令在目标设备上重新生成"。
2. **`g1_pkg/maps/rtabmap.db`**：已确认按运行时数据处理，加入 `.gitignore`。
3. **`yolonas_ocr`**：已确认整个目录忽略，不保留、不 submodule。

对应 `.gitignore` 追加：

```gitignore
# TensorRT 编译产物，设备相关，不入库
**/yolo11n.engine

# 运行时地图数据
botbrain_ws/src/g1_pkg/maps/rtabmap.db

# 第三方 OCR 子项目，整体不入库
botbrain_ws/src/g1_right_dex3/yolonas_ocr/
```

## 待执行

- [x] 确认 yolo11n.engine 处理方式 — ignore
- [x] 确认 rtabmap.db 处理方式 — ignore
- [x] 确认 yolonas_ocr 处理方式 — 整体 ignore
- [ ] 删除 fcl 的空壳 .git
- [ ] 执行 git lfs track 并生成 .gitattributes（不含 yolonas_ocr 相关文件，因整体已 ignore）
