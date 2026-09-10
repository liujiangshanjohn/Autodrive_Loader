# 挖掘机端 MQTT 对接说明

本文档供**挖掘机端（Jetson / 嵌入式）**同事对接「机队管理枢纽」的 MQTT 面使用。控制台（Dashboard）在 Web 端已按以下约定订阅/发布，挖掘机端需与之保持一致。

---

## 一、连接信息

| 项目       | 值                          | 说明 |
|------------|-----------------------------|------|
| **Broker 地址** | `10.106.241.152`            | EMQX 所在机器局域网 IP（与前端约定一致） |
| **端口**       | **1883**（TCP）             | 挖掘机端走原生 MQTT TCP，不要用 8083 |
| **连接 URL**   | `mqtt://10.106.241.152:1883` 或 `tcp://10.106.241.152:1883` | 视所用 MQTT 库而定 |

注意：**不要使用 `localhost` / `127.0.0.1`**，否则连接的是挖掘机本机，无法连到枢纽。

若后续 EMQX 换机器或换 IP，会统一通知新地址。

---

## 二、设备 ID 与 Topic 约定

每台挖掘机一个**设备 ID**，例如首台为 `excavator-001`，后续可为 `excavator-002`、`excavator-003` 等，与前端 Dashboard 选择/配置的 ID 一致即可。

| Topic 规则 | 方向 | 说明 |
|-------------|------|------|
| `device/<设备ID>/telemetry` | **挖掘机 → 云端** | 挖掘机**发布**遥测数据，Dashboard 订阅并展示 |
| `device/<设备ID>/cmd`       | **云端 → 挖掘机** | Dashboard **发布**控制指令，挖掘机**订阅**并执行 |

示例（设备 ID = `excavator-001`）：

- 遥测 Topic：`device/excavator-001/telemetry`
- 指令 Topic：`device/excavator-001/cmd`

---

## 三、遥测上报（挖掘机 → 云端）

- **Topic**：`device/<设备ID>/telemetry`  
  例如：`device/excavator-001/telemetry`
- **方向**：挖掘机 **Publish**，频率建议 1～5 秒一条（按需调整）
- **Payload**：**JSON 字符串**，UTF-8 编码

推荐字段（可扩展，前端会原样展示）：

```json
{
  "status": "running",
  "battery": 85,
  "temperature": 42,
  "location": { "lat": 31.23, "lng": 121.47 },
  "timestamp": 1709876543210
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| status | string | 如 `idle` / `running` / `fault` |
| battery | number | 电量 0–100（可选） |
| temperature | number | 温度（可选） |
| location | { lat, lng } | 经纬度（可选） |
| timestamp | number | 毫秒时间戳（可选） |

---

## 四、指令接收（云端 → 挖掘机）

- **Topic**：`device/<设备ID>/cmd`  
  例如：`device/excavator-001/cmd`
- **方向**：挖掘机 **Subscribe**，接收 Dashboard 下发的控制指令
- **Payload**：**JSON 字符串**，UTF-8 编码

当前约定指令示例（上电）：

```json
{
  "cmd": "POWER_ON",
  "timestamp": 1709876543210,
  "operator": "quinn.ops"
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| cmd | string | 指令类型，如 `POWER_ON` |
| timestamp | number | 下发端毫秒时间戳，可用于幂等/去重 |
| operator | string | 操作员账号（谁发的） |

后续若增加指令类型（如 `POWER_OFF`、`REBOOT` 等），会在此文档或接口文档中补充。

---

## 五、简要对接清单（给同事自检）

- [ ] 使用 **TCP 1883** 端口，连接地址 `mqtt://10.106.241.152:1883`（或当前通知的最新 IP）
- [ ] 确认本机设备 ID（如 `excavator-001`），与前端配置一致
- [ ] **发布**遥测到 `device/<设备ID>/telemetry`，Payload 为 JSON 字符串
- [ ] **订阅** `device/<设备ID>/cmd`，解析 JSON 后根据 `cmd` 执行相应动作（如 `POWER_ON`）
- [ ] 不使用 `localhost` / `127.0.0.1` 作为 Broker 地址

---

## 六、联系方式与文档更新

- 若 EMQX 地址/端口或 Topic 规范有变更，以此文档更新版为准。
- 有问题可找前端/架构负责人确认。

文档版本：v1  
更新日期：按实际修订日填写即可。
