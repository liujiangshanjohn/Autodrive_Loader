# MQTT Updater 项目说明

## 🔧 项目概览
这个仓库包含两个主要模块：

1. `excavator-fleet-hub`：前端的 MQTT 客户端（Web/Next.js），用于后台可视化与控制台展示。
2. `mqtt_updater`：挖掘机域控端的 MQTT 客户端（Python），用于与车辆硬件对接、上报状态、接收控制指令。

---

## 📁 目录结构
```
D:\MyCode\mqtt_updater
├── package.xml
├── README.md
├── setup.cfg
├── setup.py
├── excavator-fleet-hub/        # 前端 MQTT 客户端（Web前端）
│   ├── docs/
│   │   └── MQTT-对接说明-挖掘机端.md
│   ├── frontend/
│   │   └── excavator-mqtt/    # Next.js Web 应用
│   │       ├── app/
│   │       ├── hooks/useMQTT.ts
│   │       └── ...
│   └── mqtt-bridge/
└── mqtt_updater/               # 挖掘机域控端 MQTT 客户端（Python）
    ├── __init__.py
    ├── chassis_to_mqtt.py      # 主要逻辑
    └── ...
```

---

## 🚀 各模块职责

### 1) `excavator-fleet-hub`（前端 MQTT 客户端）
- 主要用于管理车辆列表、设备状态、远程下发、可视化仪表盘。
- 采用 Next.js + TypeScript 构建。
- 通过 `hooks/useMQTT.ts` 等封装与 MQTT Broker 的连接、订阅、发布。

### 2) `mqtt_updater`（挖掘机域控端 MQTT 客户端）
- 在挖掘机域控机上运行，负责与控制器 (chassis) 的数据转换与 MQTT 上报/命令接收。
- `chassis_to_mqtt.py` 负责读取硬件数据、构造 MQTT payload、发布到 Broker，同时订阅控制指令 Topic 并下发到域控设备。

---

## 🧭 运行方式

### 前端（excavator-fleet-hub）
1. 进入前端目录：
```bash
cd excavator-fleet-hub/frontend/excavator-mqtt
```
2. 安装依赖并启动：
```bash
npm install
npm run dev
```
3. 浏览器打开 `http://localhost:3000`（或 Next.js 指定端口）。

### 域控 Python 客户端（mqtt_updater）
1. 进入 Python 目录：
```bash
cd mqtt_updater
```
2. 安装依赖（如果有 `requirements.txt` 或 `setup.py`）：
```bash
pip install -e .
```
3. 运行主脚本：
```bash
python -m mqtt_updater.chassis_to_mqtt
```
（或者按你项目启动脚本实际方式）

---

## 📌 关键说明
- 前端 `excavator-fleet-hub` 与域控端 `mqtt_updater` 通过同一个 MQTT Broker 通信。
- 前端主要关注上层调度和展示；域控端主要处理车辆底层数据与命令桥接。
- 若要快速调试，请先确保 Broker 地址、Topic、认证配置在两端一致。

---

## 📚 参考文档
- `excavator-fleet-hub/docs/MQTT-对接说明-挖掘机端.md`
- 其他项目说明文档请查看各模块目录下 README。
