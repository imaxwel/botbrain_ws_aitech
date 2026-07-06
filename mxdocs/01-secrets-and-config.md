# 01 敏感信息与配置文件处理

## 已发现的敏感信息

| 文件 | 内容 | 风险 |
|---|---|---|
| /data/unitree/botbrain_ws/.env | NEXT_PUBLIC_SUPABASE_URL、NEXT_PUBLIC_SUPABASE_ANON_KEY、PORT | Supabase anon key 泄露风险（虽然是 anon key，但仍建议不入库） |
| /data/unitree/botbrain_ws/frontend/.env | 同上（前端自己的一份） | 同上 |
| /data/unitree/botbrain_ws/botbrain_ws/robot_config.yaml | wifi_ssid、**wifi_password 明文**、openai_api_key（当前为空） | WiFi 密码明文，高风险 |
| /data/unitree/botbrain_ws/botbrain_ws/robot_config.yaml.bak | 同上的备份 | 同上 |

## 处理方案

### 1. .env 文件
- 两处 `.env` 都不入库，已有 `frontend/.gitignore` 里的 `.env*` 规则覆盖 frontend 下的，根目录 `.env` 需要在根 `.gitignore` 里补一条 `.env` 排除规则。
- 各生成一份 `.env.example`，把 KEY 保留、VALUE 换成占位符，入库作为模板：

```
# /data/unitree/botbrain_ws/.env.example
NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key
PORT=3000
```

（frontend/.env.example 同理，字段以实际 frontend/.env 的 key 为准）

### 2. robot_config.yaml
- 这个文件是运行时配置，且包含 WiFi 密码，**不建议原文件入库**。
- 方案：
  1. 根目录/工作区加一条 `.gitignore` 规则排除 `botbrain_ws/robot_config.yaml` 和 `*.bak`
  2. 新建 `botbrain_ws/robot_config.yaml.example`，敏感字段替换为占位符：

```yaml
robot_configuration:
  robot_name: "g1_robot"
  robot_model: "g1"
  description_file_type: "xacro"
  network_interface: "enP8p1s0"
  tita_namespace: ""
  openai_api_key: ""
  wifi_interface: "wlxXXXXXXXXXXXX"
  wifi_ssid: "YOUR_WIFI_SSID"
  wifi_password: "YOUR_WIFI_PASSWORD"
  default_map: "rtabmap.db"
  default_static_map: "accumulated.yaml"
```

  3. README 里注明：首次部署需要 `cp robot_config.yaml.example robot_config.yaml` 并填写真实值。

### 3. docker-compose.yaml
- 检查后未发现明文密钥（仅镜像名、网络配置、GPU 声明等），可以直接入库。
- `docker-compose.yaml.bak.20260706155057` 是今天的备份文件，属于临时文件，不建议入库（.gitignore 排除 `*.bak*`），如果需要保留变更历史，直接用 git commit 历史代替 .bak 文件即可。

### 4. 二次检查命令（执行 git add 前必须跑一遍）

```bash
# 在 /data/unitree/botbrain_ws 下执行，扫描可能残留的密钥模式
grep -rIn --exclude-dir={build,install,log,node_modules,.next,.git} \
  -E "(AKIA[0-9A-Z]{16}|password\s*[:=]|secret|api[_-]?key|token\s*[:=])" \
  . 2>/dev/null | grep -v -E "\.example|README|\.md:" | less
```

执行后人工复核每一条命中，确认没有真实密钥被纳入待提交范围。

## 执行清单

- [ ] 创建 `.env.example`（根目录）
- [ ] 创建 `frontend/.env.example`
- [ ] 创建 `botbrain_ws/robot_config.yaml.example`
- [ ] 在根 `.gitignore` 加入 `.env`、`botbrain_ws/robot_config.yaml`、`*.bak*`
- [ ] 跑二次检查命令确认无遗漏
