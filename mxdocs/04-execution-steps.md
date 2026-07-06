# 04 执行步骤清单（实际命令）

以下命令均在 `unitree@unitree-g1-nx` 机器上，`/data/unitree/botbrain_ws` 目录内执行。
执行前提：01、02、03 文档中的确认项已经处理完毕。

## Step 0 前置检查

```bash
cd /data/unitree/botbrain_ws
git --version         # 已确认 2.34.1
git-lfs version        # 已确认 3.0.2
ssh -T git@github.com   # 已确认直连成功，账号 imaxwel
```

## Step 1 处理敏感文件（对应文档 01）

```bash
cd /data/unitree/botbrain_ws

# 生成 env 模板（值需要按实际 .env 内容手工核对后填占位符，不要直接复制真实值）
cat > .env.example << 'EOF'
NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key
PORT=3000
EOF

cp frontend/.env frontend/.env.example
# 然后手工编辑 frontend/.env.example，把真实值替换成占位符

# robot_config 模板
cp botbrain_ws/robot_config.yaml botbrain_ws/robot_config.yaml.example
# 手工编辑 robot_config.yaml.example，替换 wifi_ssid / wifi_password / wifi_interface 为占位符
```

## Step 2 清理垃圾与损坏的嵌套仓库

```bash
# 删除损坏的空壳 git 仓库
rm -rf /data/unitree/botbrain_ws/botbrain_ws/src/fcl/.git

# 删除今天的 compose 备份文件（改动已在当前 docker-compose.yaml 里，历史交给 git 管理）
rm -f /data/unitree/botbrain_ws/docker-compose.yaml.bak.20260706155057
rm -f /data/unitree/botbrain_ws/botbrain_ws/robot_config.yaml.bak
```

## Step 3 写入 .gitignore（对应文档 02）

把文档 02 中的内容写入 `/data/unitree/botbrain_ws/.gitignore`（新建文件）。

## Step 4 初始化仓库并配置 LFS

```bash
cd /data/unitree/botbrain_ws
git init
git lfs install

git lfs track "*.STL" "*.stl" "*.obj" "*.dae" "*.pb" "*.onnx" "*.pt" "*.gif" "*.pdf"
# rtabmap.db / yolo11n.engine / yolonas_ocr 已确认整体 ignore，不做 LFS track

git remote add origin git@github.com:imaxwel/botbrain_ws_aitech.git
```

## Step 5 首次提交前的最终扫描

```bash
cd /data/unitree/botbrain_ws
git add -n .   # dry-run，先看看哪些文件会被加入，人工检查列表里没有 .env / robot_config.yaml / build 等

# 密钥扫描（见文档01）
grep -rIn --exclude-dir={build,install,log,node_modules,.next,.git} \
  -E "(AKIA[0-9A-Z]{16}|password\s*[:=]|secret|api[_-]?key|token\s*[:=])" \
  . 2>/dev/null | grep -v -E "\.example|README|\.md:"
```

确认无误后：

```bash
git add .
git status   # 再次确认 staged 文件列表合理，重点看体积是否符合预期（无 build/install/node_modules）
du -sh .git  # 提交前后对比，确认没有意外的大文件被 LFS track 遗漏
```

## Step 6 提交与推送

```bash
git commit -m "Initial commit: botbrain_ws workspace backup (code, compose, autostart scripts)"

git branch -M main
git push -u origin main
```

推送是网络操作，且目标仓库当前为空，属于低风险（新增，不覆盖任何远程已有内容）。

## Step 7 推送后验证

```bash
# 本地验证 clone 是否完整（建议在另一个目录测试，不要覆盖当前工作区）
cd /tmp
git clone git@github.com:imaxwel/botbrain_ws_aitech.git verify_clone
du -sh verify_clone
cd verify_clone && git lfs pull   # 确认 LFS 文件能正常拉取
```

验证完成后删除 verify_clone 测试目录：

```bash
rm -rf /tmp/verify_clone
```

## 后续维护建议

- 后续新增大文件时先执行 `git lfs track "<pattern>"`，再 `git add`，避免大文件直接进普通 git 历史。
- `docker-compose.yaml` 每次改动后不要再手工留 `.bak` 文件，直接靠 git commit/diff 追踪变更。
- 定期（如每次固件/环境升级后）检查 `.gitignore` 是否需要补充新的编译产物路径。
