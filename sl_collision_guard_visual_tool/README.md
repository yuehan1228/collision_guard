# 防碰撞系统可视化调试工具

一个面向工控机的本地 Web 工具，用来对防碰撞系统进行可视化、调试，以及管理相关 `systemd` 服务。

支持：

- 查看服务运行状态
- 启动、停止、重启服务
- 设置开机自启
- 编辑指定配置文件
- 查看 `journalctl` 日志
- 打包为可安装的 `.deb`

## 运行方式

这个项目的推荐部署形态是：

- 后端作为 `systemd` 服务常驻运行
- 默认监听 `127.0.0.1:8080`
- 桌面图标只负责打开本地页面

这样在工控机上安装后，用户点击图标即可进入调试面板，而核心权限仍由后台服务持有。

## 目录

- `app.py`: 后端 HTTP 服务
- `config/services.json`: 默认服务白名单
- `static/`: 前端页面和图标资源
- `systemd/`: 服务文件、桌面入口、环境配置
- `packaging/debian/`: Debian 安装脚本
- `scripts/build-deb.sh`: 本地构建 `.deb`

网页可视化依赖的前端库已随项目放在 `static/vendor/`，离线服务器不需要访问外网 CDN。

## 配置服务

默认配置文件格式：

```json
{
  "services": [
    {
      "id": "myapp",
      "display_name": "My App",
      "unit": "myapp.service",
      "config_path": "/etc/myapp/config.yaml",
      "config_format": "yaml",
      "restart_after_save": true
    }
  ]
}
```

字段说明：

- `id`: 前端和 API 使用的固定 ID
- `display_name`: 页面显示名称
- `unit`: 实际 systemd unit 名称
- `config_path`: 允许编辑的配置文件路径
- `config_format`: 仅用于前端展示提示
- `restart_after_save`: 保存配置后是否默认重启服务

开发时默认读取仓库内的 `config/services.json`。  
安装成 `.deb` 后默认读取 `/etc/collision-guard-visual-debugger/services.json`。

## 本地启动

```bash
python3 app.py
```

默认监听 `0.0.0.0:8080`。

也可以通过环境变量覆盖：

```bash
CG_DEBUG_TOOL_HOST=127.0.0.1 \
CG_DEBUG_TOOL_PORT=9000 \
CG_DEBUG_TOOL_CONFIG=/etc/collision-guard-visual-debugger/services.json \
python3 app.py
```

兼容旧变量名 `SERVICE_ADMIN_HOST`、`SERVICE_ADMIN_PORT`、`SERVICE_ADMIN_CONFIG`。

## 打包 `.deb`

构建机需要有 `dpkg-deb`。

```bash
./scripts/build-deb.sh
```

产物会生成在：

```text
build/collision-guard-visual-debugger_0.1.0_<arch>.deb
```

## 安装

推荐：

```bash
sudo apt install ./build/collision-guard-visual-debugger_0.1.0_<arch>.deb
```

也可以：

```bash
sudo dpkg -i ./build/collision-guard-visual-debugger_0.1.0_<arch>.deb
```

安装后会完成这些动作：

- 安装程序到 `/opt/collision-guard-visual-debugger/current`
- 安装服务文件到 `/lib/systemd/system/collision-guard-visual-debugger.service`
- 安装默认环境文件到 `/etc/default/collision-guard-visual-debugger`
- 首次创建 `/etc/collision-guard-visual-debugger/services.json`
- 自动执行 `systemctl enable --now collision-guard-visual-debugger.service`
- 安装桌面图标 `防碰撞系统可视化调试工具`

## 安装后的使用方式

- 后台服务地址：`http://127.0.0.1:8080`
- 桌面点击 `防碰撞系统可视化调试工具` 图标会打开这个地址

如果需要修改监听地址或端口，编辑：

```text
/etc/default/collision-guard-visual-debugger
```

如果需要修改可管理服务列表，编辑：

```text
/etc/collision-guard-visual-debugger/services.json
```

修改服务配置后执行：

```bash
sudo systemctl restart collision-guard-visual-debugger.service
```

## 权限说明

当前打包方案默认让 `collision-guard-visual-debugger.service` 以 `root` 运行，并仅监听本机地址。原因是这个程序需要：

- 调 `systemctl`
- 读 `journalctl`
- 写业务配置文件

如果后续要改成非 `root` 运行，需要同时补齐：

- `sudoers` 或 `polkit` 白名单
- `journald` 读取权限
- 对各业务配置文件路径的写权限设计

## 卸载

```bash
sudo apt remove collision-guard-visual-debugger
```

如需连配置一起清理：

```bash
sudo apt purge collision-guard-visual-debugger
```
