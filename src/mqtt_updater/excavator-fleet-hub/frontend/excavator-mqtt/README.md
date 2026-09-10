# Excavator Fleet Hub 前端文档

本项目是一个基于 Next.js App Router + React 的轻量 MQTT 控制台 Demo，主要用于演示机队遥测展示与控制指令下发。

## 关键文件说明

- `package.json`
  - 由 `next`, `react`, `mqtt` 组成。包含 `dev`, `build`, `start`, `lint` 命令。

- `next.config.ts`, `tsconfig.json`, `eslint.config.mjs`, `postcss.config.mjs`
  - Next.js 和 TypeScript 项目配置，通常无需频繁修改。

- `app/layout.tsx`
  - 全局布局文件，定义 HTML 基本结构与根样式范围。

- `app/globals.css`
  - 全局 CSS 样式，包含项目基础配色、字体等。

- `app/page.tsx`
  - 登录页（login）。模拟账号密码校验并在本地存储 `operator_name`，登录成功后跳转到 `/dashboard`。

- `app/dashboard/page.tsx`
  - Dashboard 主页面：
    - 使用 `useMQTT` 订阅遥测 Topic
    - 显示连接状态、遥测 JSON 数据、并提供“发送 POWER_ON 指令”按钮
    - 通过 `sessionStorage` 读取操作者名称显示在顶部
    - 主要业务逻辑都在此文件，适合后续扩展真实 API、topic 配置、指令 UI 等

- `hooks/useMQTT.ts`
  - MQTT 客户端 Hook：
    - 连接 Broker、订阅 topic、接收消息、发送 publish
    - 处理 reconnect、error、message 事件
    - 组件卸载时自动断开
    - 后续可继续优化连接配置、JSON 解析、重连策略、QoS 支持等

## 运行与调试

1. 安装依赖：

```bash
npm install
```

2. 启动开发环境：

```bash
npm run dev
```

3. 在浏览器访问：

```bash
http://localhost:3000
```

## 环境变量与 Broker 说明

- `app/dashboard/page.tsx` 中默认 Broker URL：`process.env.NEXT_PUBLIC_MQTT_BROKER_WS ?? "ws://10.106.241.152:8083/mqtt"`
- 生产或测试环境建议使用 `NEXT_PUBLIC_MQTT_BROKER_WS` 指定 EMQX WebSocket 地址。
- 设备 ID 使用常量 `excavator-001`，topic 模板为 `device/${id}/telemetry` 和 `device/${id}/cmd`。

## 交接要点

- **核心模块**：`hooks/useMQTT.ts`（MQTT 连接与消息） + `app/dashboard/page.tsx`（业务呈现与指令）
- **UI 路由**：`/` 为登录页，`/dashboard` 为监控控制页
- **演示账号**：当前登录逻辑为本地模拟，任意账号密码通过。
- **后续建议**：补充真实登录接口、状态管理、设备列表、图表展示、错误处理与可配置Broker

---

如需进一步增加新功能，可以直接从 `app/dashboard/page.tsx` 进行业务新增，从 `hooks/useMQTT.ts` 调整 MQTT 行为。

