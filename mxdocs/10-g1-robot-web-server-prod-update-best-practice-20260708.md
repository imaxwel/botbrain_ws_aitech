# G1 Robot Web Server Prod 更新最佳实践

日期：2026-07-08

目标容器：`g1_robot_web_server_prod`

项目目录：`/data/botbrain_ws/botbrain_project-main`

前端目录：`/data/botbrain_ws/botbrain_project-main/frontend`

## 当前结论

`g1_robot_web_server_prod` 不是自定义构建镜像，而是使用 `node:22-bullseye`，并通过 bind mount 将宿主机的 `./frontend` 挂载到容器 `/app`：

```yaml
web_server_prod:
  image: node:22-bullseye
  working_dir: /app
  volumes:
    - ./frontend:/app
  env_file:
    - ./frontend/.env
  environment:
    - NODE_ENV=production
    - PORT=80
  command: npm run start
  restart: unless-stopped
  network_mode: host
```

因此，更新最佳实践不是 `docker pull` 或只重启容器，而是：

1. 确认 Git 代码版本。
2. 保护现场配置和未提交修改。
3. 使用 compose 的 builder 服务重新安装依赖并构建 `.next`。
4. 使用 `docker compose up -d --force-recreate web_server_prod` 重建生产容器，让新的 `.env` 和构建产物同时生效。
5. 做日志和 HTTP 健康检查。

## 本次已执行结果

本次检查结果：

```bash
cd /data/botbrain_ws/botbrain_project-main
git fetch origin main
git rev-list --left-right --count HEAD...origin/main
```

结果为：

```text
0       0
```

说明当前 `HEAD` 与 `origin/main` 一致，提交为：

```text
17e36ab 同步g1 old仓库代码
```

当前仍存在一个工作区修改：

```text
 M frontend/package-lock.json
```

该修改在本次操作前已经存在，主要表现为 lockfile name 和 npm 元数据差异。不要在未确认来源前直接丢弃它。

本次已经执行：

```bash
cd /data/botbrain_ws/botbrain_project-main
docker compose stop web_server_prod
docker compose run --rm --no-deps web_server_builder
docker compose up -d --force-recreate web_server_prod
```

构建结果：

```text
Next.js 15.5.9
Compiled successfully
Generated static pages: 20/20
```

新容器：

```text
2282dbd6c33c node:22-bullseye g1_robot_web_server_prod Up
```

启动日志：

```text
Next.js 15.5.9
Local:   http://localhost:80
Network: http://192.168.123.174:80
Ready in 596ms
```

健康检查：

```bash
curl -I --max-time 10 http://127.0.0.1/
```

返回：

```text
HTTP/1.1 200 OK
```

## 标准更新流程

### 1. 登录机器人

```bash
ssh g1edu
cd /data/botbrain_ws/botbrain_project-main
```

### 2. 记录当前状态

```bash
git status --short --branch
git rev-parse --short HEAD
docker ps --filter name=g1_robot_web_server_prod \
  --format '{{.ID}} {{.Image}} {{.Names}} {{.Status}}'
docker inspect g1_robot_web_server_prod \
  --format 'Started={{.State.StartedAt}} Restart={{.HostConfig.RestartPolicy.Name}} NetworkMode={{.HostConfig.NetworkMode}}'
```

### 3. 确认 `.env` 已在正确位置

生产容器读取的是：

```text
/data/botbrain_ws/botbrain_project-main/frontend/.env
```

检查文件存在即可，不要把密钥内容打印到终端或写进文档：

```bash
test -f frontend/.env && ls -l frontend/.env
```

如果 `.env` 有更新，必须 recreate 容器；单纯 `docker restart` 不保证 compose 的 `env_file` 重新装载到新配置。

### 4. 处理 Git 工作区

先拉取远端引用，不改工作区：

```bash
git fetch origin main
git rev-list --left-right --count HEAD...origin/main
git status --short
```

如果输出是 `0 0`，说明本地代码已经是最新。

如果本地落后，且 `git status --short` 没有业务修改，可以执行：

```bash
git merge --ff-only origin/main
```

如果存在未提交修改，先判断来源：

```bash
git diff -- frontend/package-lock.json
```

推荐处理方式：

- 如果修改是预期代码变更：提交到 Git 后再上线。
- 如果只是 npm 版本造成的 lockfile 元数据变化：确认后可以单独恢复或重新生成，但不要在不了解原因时直接覆盖。
- 如果需要临时保留现场修改：使用 `git stash push -u -m "pre-web-prod-update-$(date +%Y%m%d-%H%M%S)"`，更新后再决定是否恢复。

### 5. 停止生产 Web 容器

```bash
docker compose stop web_server_prod
```

说明：当前部署方式会直接在 bind-mounted `frontend` 目录里更新 `node_modules` 和 `.next`，构建期间让旧的 Next 进程继续运行会有读写同一目录的风险。因此推荐在维护窗口内先停服务再构建。

### 6. 重新安装依赖并构建

使用 compose 里已有的 builder 服务，保证 Node 版本和生产容器一致：

```bash
docker compose run --rm --no-deps web_server_builder
```

该服务实际执行：

```bash
npm ci && npm run build:oss
```

成功标准：

```text
Compiled successfully
Generating static pages
Finalizing page optimization
```

如果构建失败，不要启动新生产容器。先查看报错并修复代码或 `.env`。

### 7. 重建生产容器

```bash
docker compose up -d --force-recreate web_server_prod
```

必须使用 `--force-recreate` 的原因：

- `frontend/.env` 是通过 compose `env_file` 注入容器环境变量。
- Next.js 的部分变量可能在 build 阶段固化，部分变量在容器启动阶段读取。
- recreate 能同时保证新容器环境和新 `.next` 构建产物一致。

### 8. 验证

```bash
docker compose ps web_server_prod
docker logs --tail 80 g1_robot_web_server_prod
curl -I --max-time 10 http://127.0.0.1/
```

期望结果：

```text
g1_robot_web_server_prod Up
Ready
HTTP/1.1 200 OK
```

外部访问地址按当前日志为：

```text
http://192.168.123.174:80
```

## 回滚流程

如果上线后 Web 不正常，先保留日志：

```bash
docker logs --tail 200 g1_robot_web_server_prod
```

回滚到上一个确认可用提交：

```bash
cd /data/botbrain_ws/botbrain_project-main
git log --oneline -5
git reset --hard <GOOD_COMMIT>
docker compose stop web_server_prod
docker compose run --rm --no-deps web_server_builder
docker compose up -d --force-recreate web_server_prod
curl -I --max-time 10 http://127.0.0.1/
```

注意：`git reset --hard` 会丢弃未提交修改。执行前必须确认 `.env` 不受 Git 管理，并确认没有需要保留的本地代码改动。

## 不建议的做法

不要只执行：

```bash
docker restart g1_robot_web_server_prod
```

原因：这不会更新 Git 代码，不会重新安装依赖，不会重新构建 `.next`，也不是加载 compose `env_file` 变更的最稳妥方式。

不要直接清理 compose 提示的 orphan containers：

```bash
docker compose up -d --remove-orphans
```

除非已经确认这些 orphan containers 不属于正在运行的机器人业务。当前 compose 曾提示以下 orphan containers：

```text
g1_robot_mapping
botbrain_project-main-bringup-run-bbee4e886f21
botbrain_project-main-bringup-run-c1e5c25e7a36
botbrain_project-main-bringup-run-ebdda9fcab64
```

这些容器是否可清理应作为独立运维任务确认。

## 建议后续优化

当前生产容器直接 bind mount `frontend`，构建产物和运行态耦合在同一目录。更稳的长期方案是：

1. 为前端增加专用 Dockerfile。
2. 在镜像构建阶段执行 `npm ci && npm run build:oss`。
3. 生产容器只运行构建好的镜像，不挂载整个源码目录。
4. 使用镜像 tag 或 Git SHA 作为版本号。
5. 回滚时只切换镜像 tag，不需要在生产目录执行 `git reset --hard`。

在当前机器人现场部署方式下，上面的标准更新流程是最小改动、风险较低、可重复执行的实践。
