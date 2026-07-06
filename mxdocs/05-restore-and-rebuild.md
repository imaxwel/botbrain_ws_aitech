# 05 备份还原 / 新机器重建指南

本文档面向：在新的 unitree G1 设备或者恢复现场时，如何从 GitHub 仓库还原出可运行的 `/data/unitree/botbrain_ws` 环境。

## 1. Clone 仓库

```bash
cd /data/unitree
git clone git@github.com:imaxwel/botbrain_ws_aitech.git botbrain_ws
```

> 说明：仓库未使用 Git LFS（模型权重/3D网格/地图/文档演示等二进制资源已确认彻底不要，不入库），因此不需要 `git lfs pull`。这些资源需要按下文清单另行获取。

## 2. 还原被排除的敏感配置

```bash
cp .env.example .env
# 编辑 .env，填入真实 Supabase URL / anon key / PORT

cp frontend/.env.example frontend/.env
# 编辑 frontend/.env，填入真实值

cp botbrain_ws/robot_config.yaml.example botbrain_ws/robot_config.yaml
# 编辑 robot_config.yaml，填入真实 wifi_ssid / wifi_password / wifi_interface / openai_api_key
```

## 3. 还原第三方预编译依赖（deps/open3d141，未入库）

需要按 open3d141 的原始来源重新获取（编译或下载预编译包），具体版本/编译参数建议在此补充记录：

- Open3D 版本：<TODO: 补充具体版本号，比如 0.14.1>
- 编译选项：<TODO: 补充 cmake 参数，比如是否开启 CUDA/ARM64 等>
- 或者：如果是从 Unitree/BotBrain 官方镜像内提取的，记录提取路径/镜像 tag

> 建议后续找机会把 open3d141 的构建脚本或下载地址整理进这里，现在先留 TODO，不能凭空猜测具体版本参数。

## 4. 还原运行时数据（如 rtabmap.db，若按方案未入库）

如果 `g1_pkg/maps/rtabmap.db` 按方案作为运行时数据被 ignore，新机器需要重新执行建图流程（fast_lio + 3d mapping）生成，或从原机器手工拷贝：

```bash
scp unitree@<原机器IP>:/data/unitree/botbrain_ws/botbrain_ws/src/g1_pkg/maps/rtabmap.db \
    ./botbrain_ws/src/g1_pkg/maps/rtabmap.db
```

## 5. 编译工作区

```bash
cd /data/unitree/botbrain_ws
# 参照 docker-compose.yaml 里 builder_base / builder_yolo / builder_dex3 三个 service 的 command
docker compose run --rm builder_base
docker compose run --rm builder_yolo
docker compose run --rm builder_dex3
```

## 6. 安装并启用自启动服务

```bash
sudo cp botbrain.service /etc/systemd/system/botbrain.service
sudo cp web_server.service /etc/systemd/system/web_server.service

# 注意：原 service 文件里 WorkingDirectory 是占位符 BOTBRAIN_WORKSPACE_PATH
# 需要手工替换为实际路径，例如 /data/unitree/botbrain_ws
sudo sed -i 's#BOTBRAIN_WORKSPACE_PATH#/data/unitree/botbrain_ws#' /etc/systemd/system/botbrain.service
sudo sed -i 's#BOTBRAIN_WORKSPACE_PATH#/data/unitree/botbrain_ws#' /etc/systemd/system/web_server.service

sudo systemctl daemon-reload
sudo systemctl enable --now botbrain.service
sudo systemctl enable --now web_server.service
```

> 备注：已确认现网机器上 `/etc/systemd/system/botbrain.service` 和 `web_server.service` 里的 `WorkingDirectory` 已经手工改成了实际绝对路径 `/data/unitree/botbrain_ws`，仓库里保留的是原始模板（含占位符），新机器部署时记得替换。

## 7. 验证

```bash
systemctl status botbrain.service
systemctl status web_server.service
docker compose ps
```

## 已知不在仓库中的内容清单（需要在新机器另行准备）

| 内容 | 原因 | 获取方式 |
|---|---|---|
| deps/open3d141 | 预编译第三方库，576M | 见上文 Step 3，需补充具体来源 |
| .env / frontend/.env | 含密钥 | 从 .env.example 手工填写 |
| botbrain_ws/robot_config.yaml | 含 WiFi 密码 | 从 .example 手工填写 |
| g1_pkg/maps/rtabmap.db | 运行时地图数据（已确认不入库） | 重新建图或从原机器拷贝 |
| yolo11n.engine | TensorRT 编译产物，设备相关（已确认不入库） | 用 yolo11n.pt 在目标设备重新 export |
| botbrain_ws/src/g1_right_dex3/yolonas_ocr | 第三方 OCR 子项目（已确认整体不入库） | 需要时从原机器直接拷贝该目录，或从其原始来源 `https://github.com/SatArw/yolonas_ocr` 重新获取 |
| 所有模型权重 (*.pt/*.onnx/*.pth/*.engine 等) | 已确认彻底不要，不入库 | bot_yolo 等模型需重新训练/下载，或从原机器拷贝 |
| 所有 3D 网格文件 (*.STL/*.obj/*.dae 等) | 已确认彻底不要，不入库 | 从原机器拷贝，或从 CAD/上游模型源重新导出 |
| botbrain_ws/src/fcl/test/fcl_resources、fast_lio/doc、joystick-bot/docs | 第三方库自带测试数据/文档演示资源，已确认不入库 | 不影响编译运行，如需完整文档可从对应上游仓库重新获取 |
| hardware/**/*.3mf | 3D 打印文件，已确认不入库 | 从原机器拷贝 |
| colcon build/install/log | 编译产物 | 在新机器重新 colcon build |
