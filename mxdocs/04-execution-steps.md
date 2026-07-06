# 04 执行步骤清单（实际执行记录）

**状态：已全部执行完成，已推送到 GitHub。**

以下为在 `unitree@unitree-g1-nx` 机器上、`/data/unitree/botbrain_ws` 目录内实际执行的步骤记录。

## Step 0 前置检查（已完成）

```bash
git --version          # 2.34.1
git-lfs version        # 3.0.2
ssh -T git@github.com  # 直连成功，账号 imaxwel
git config --global user.name / user.email   # 已配置：iMaxwel / miscxw@gmail.com
```

## Step 1 处理敏感文件（已完成）

```bash
cd /data/unitree/botbrain_ws

cat > .env.example << 'EOF'
NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key
PORT=3000
EOF

cat > frontend/.env.example << 'EOF'
NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
NEXT_PUBLIC_SUPABASE_PUBLISHABLE_KEY=your-publishable-key
NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key
EOF

cat > botbrain_ws/robot_config.yaml.example << 'EOF'
robot_configuration:
  robot_name: "g1_robot"
  robot_model: "g1"
  description_file_type: "xacro"
  network_interface: "enP8p1s0"
  tita_namespace: ""
  openai_api_key: ""
  wifi_interface: "YOUR_WIFI_INTERFACE"
  wifi_ssid: "YOUR_WIFI_SSID"
  wifi_password: "YOUR_WIFI_PASSWORD"
  default_map: "rtabmap.db"
  default_static_map: "accumulated.yaml"
EOF
```

## Step 2 清理垃圾与损坏的嵌套仓库（已完成）

```bash
rm -rf /data/unitree/botbrain_ws/botbrain_ws/src/fcl/.git
rm -f /data/unitree/botbrain_ws/docker-compose.yaml.bak.20260706155057
rm -f /data/unitree/botbrain_ws/botbrain_ws/robot_config.yaml.bak
```

## Step 3 写入 .gitignore（已完成，经过一次修正）

写入了 02 文档中的完整 `.gitignore` 内容。中途发现二进制资源处理方案需要变更为"彻底不要"，补充了 AI 模型/3D网格/地图/文档演示等排除规则，以及第三方库测试数据（fcl/test、fast_lio/doc、joystick-bot/docs）和 `.3mf` 文件的排除规则。

## Step 4 初始化仓库（已完成，含一次方案调整）

```bash
cd /data/unitree/botbrain_ws
git init
git remote add origin git@github.com:imaxwel/botbrain_ws_aitech.git
```

> **方案调整记录**：最初执行了 `git lfs install` 并 `git lfs track` 了 STL/obj/dae/pb/onnx/pt/gif/pdf 等格式，生成了 `.gitattributes`。随后确认二进制资源应彻底不要而非用 LFS 保留，因此执行了：
> ```bash
> git lfs uninstall
> rm -f .gitattributes
> ```
> 且由于 `git add` 已经把大文件写成了 git object（`.git` 一度达到 336M），单纯 `git reset` 不会清理已写入的 blob，因此彻底 `rm -rf .git` 后重新 `git init`，避免残留悬空对象占用体积。

## Step 5 最终扫描与首次提交（已完成）

```bash
cd /data/unitree/botbrain_ws

# dry-run 检查，确认敏感/大文件/第三方资源不在待提交列表中
git add -n . | grep -E '(\.env$|robot_config\.yaml$|/build/|/install/|/log/|node_modules|\.next/|yolonas_ocr|rtabmap\.db|\.engine$|\.bak|\.pt$|\.onnx$|\.pb$|\.STL$|\.stl$|\.dae$|\.obj$|\.gif$|\.pdf$|\.pgm$|\.pcd$|\.ply$|\.3mf$|fcl/test/fcl_resources|fast_lio/doc/|joystick-bot/docs/)'
# 结果：无命中

# 密钥扫描
grep -rIn --exclude-dir={build,install,log,node_modules,.next,.git,deps} \
  -E "(AKIA[0-9A-Z]{16}|password[[:space:]]*[:=]|secret|api[_-]?key|token[[:space:]]*[:=])" \
  . 2>/dev/null | grep -v -E "\.example|README|\.md:"
# 结果：命中项均为正常代码引用（变量名/UI组件/测试框架内部注释），
# 唯一真实敏感命中是 robot_config.yaml（已被 .gitignore 排除，未进入待提交列表）

git add .
du -sh .git   # 12M，符合预期
```

## Step 6 提交与推送（已完成）

```bash
git commit -m "Initial commit: botbrain_ws workspace backup (code, compose, autostart scripts)"
# commit f88e0c2, 1582 个文件

git branch -M main
git push -u origin main
# 成功推送到 origin/main
```

## Step 7 推送后验证（已完成）

```bash
cd /tmp
git clone git@github.com:imaxwel/botbrain_ws_aitech.git verify_clone
du -sh verify_clone    # 26M
ls verify_clone/.env                           # 不存在，确认
ls verify_clone/botbrain_ws/robot_config.yaml  # 不存在，确认
rm -rf /tmp/verify_clone
```

验证结果：clone 成功，体积 26M，敏感文件未泄露，仅 `.env.example`、`robot_config.yaml.example` 等模板文件存在。

## 最终结果汇总

| 项目 | 结果 |
|---|---|
| 首次提交 | `f88e0c2` |
| 远程分支 | `origin/main` |
| 入库文件数 | 1582 |
| .git 体积 | 12M |
| Git LFS | 未使用（已撤销） |
| 敏感信息泄露检查 | 通过 |

## 后续维护建议

- 后续新增文件时，先跑一次 `git add -n .` 加敏感信息/大文件 grep 检查，再正式 `git add`。
- `docker-compose.yaml` 每次改动后不要再手工留 `.bak` 文件，直接靠 git commit/diff 追踪变更。
- 定期检查 `.gitignore` 是否需要补充新的编译产物路径，尤其是新引入第三方 ROS 包时，注意其自带的 doc/test 资源目录。
