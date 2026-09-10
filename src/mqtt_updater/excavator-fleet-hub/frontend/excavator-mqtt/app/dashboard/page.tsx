"use client";

import { useEffect, useState } from "react";
import { useMQTT } from "../../hooks/useMQTT";

// 浏览器端必须用 EMQX 所在机器的局域网 IP，不能用 localhost（否则其他设备访问时会连到自己的电脑）
const BROKER_URL =
  process.env.NEXT_PUBLIC_MQTT_BROKER_WS ?? "ws://10.106.241.152:8083/mqtt";
const EXCAVATOR_ID = "excavator-001";
const TELEMETRY_TOPIC = `device/${EXCAVATOR_ID}/telemetry`;
const CMD_TOPIC = `device/${EXCAVATOR_ID}/cmd`;

interface TelemetryPayload {
  status?: string;
  battery?: number;
  temperature?: number;
  location?: {
    lat: number;
    lng: number;
  };
  [key: string]: unknown;
}

export default function DashboardPage() {
  const { isConnected, lastMessage, publish, error } = useMQTT(
    BROKER_URL,
    TELEMETRY_TOPIC
  );
  const [telemetry, setTelemetry] = useState<TelemetryPayload | null>(null);
  const [operatorName, setOperatorName] = useState<string>("Unknown");

  // 1. 在客户端挂载后，读取操作员名字
  useEffect(() => {
    const name = sessionStorage.getItem("operator_name");
    if (name) {
      setOperatorName(name);
    }
  }, []);

  // 2. 解析遥测数据
  useEffect(() => {
    if (!lastMessage) return;
    try {
      const parsed = JSON.parse(lastMessage) as TelemetryPayload;
      setTelemetry(parsed);
    } catch (e) {
      setTelemetry({ raw: lastMessage });
    }
  }, [lastMessage]);

  const handlePowerOn = () => {
    if (!isConnected) {
      alert("⚠️ MQTT 未连接，无法下发指令！");
      return;
    }

    const payload = {
      cmd: "POWER_ON",
      timestamp: Date.now(),
      // 3. 动态注入真实操作员账号
      operator: operatorName, 
    };

    // 注意：去你的 hooks/useMQTT.ts 里把 publish 的类型改成支持 options 的，
    // 例如: publish: (topic: string, message: string, options?: mqtt.IClientPublishOptions) => void
    publish(CMD_TOPIC, JSON.stringify(payload), { qos: 1 });
  };

  return (
    <div className="min-h-screen bg-slate-950 text-slate-50">
      <header className="border-b border-slate-800/80 bg-slate-950/70 backdrop-blur">
        <div className="mx-auto flex max-w-6xl items-center justify-between px-6 py-4">
          <div>
            <p className="text-xs uppercase tracking-[0.3em] text-emerald-400/80">
              Excavator Fleet Hub
            </p>
            <h1 className="mt-1 text-lg font-semibold">
              机队监控 Dashboard（MQTT 面）
            </h1>
          </div>
          <div className="flex items-center gap-6 text-sm">
            {/* 顶栏顺手展示一下当前操作员，让界面显得更专业 */}
            <div className="hidden sm:flex items-center text-slate-400 border-r border-slate-700 pr-6">
              <span className="font-mono bg-slate-800 px-2 py-1 rounded text-slate-300">
                OP: {operatorName}
              </span>
            </div>
            <div className="flex items-center gap-3">
              <div
                className={`h-2.5 w-2.5 rounded-full ${
                  isConnected ? "bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,0.8)]" : "bg-rose-500 animate-pulse"
                }`}
              />
              <span className="text-slate-300">
                EMQX：{isConnected ? "已连接" : "未连接 / 重连中"}
              </span>
            </div>
          </div>
        </div>
      </header>

      <main className="mx-auto flex max-w-6xl flex-col gap-6 px-6 py-6 md:flex-row">
        <section className="flex-1 rounded-2xl border border-slate-800 bg-slate-900/60 p-5 shadow-[0_0_40px_rgba(15,23,42,0.9)]">
          <h2 className="mb-3 text-sm font-semibold text-slate-100">
            实时遥测数据（Telemetry）
          </h2>
          <p className="mb-3 text-xs text-slate-400">
            订阅 Topic：<span className="font-mono text-slate-300">{TELEMETRY_TOPIC}</span>
          </p>

          {error && (
            <p className="mb-3 text-xs text-rose-400" role="alert">
              MQTT 错误：{error}
            </p>
          )}

          <div className="mt-2 h-72 overflow-auto rounded-xl border border-slate-800 bg-black/40 p-4 text-xs font-mono text-emerald-300">
            {telemetry ? (
              <pre className="leading-relaxed">{JSON.stringify(telemetry, null, 2)}</pre>
            ) : (
              <p className="text-slate-500 animate-pulse">
                等待挖掘机设备（{EXCAVATOR_ID}）上报 MQTT 遥测数据…
              </p>
            )}
          </div>
        </section>

        <section className="w-full max-w-sm rounded-2xl border border-slate-800 bg-slate-900/60 p-5 shadow-[0_0_40px_rgba(15,23,42,0.9)] flex flex-col">
          <h2 className="mb-3 text-sm font-semibold text-slate-100">
            设备控制（Command）
          </h2>
          <p className="mb-6 text-xs text-slate-400">
            下发 Topic：
            <span className="font-mono text-slate-300 block mt-1">{CMD_TOPIC}</span>
          </p>

          <button
            onClick={handlePowerOn}
            disabled={!isConnected}
            className="w-full rounded-xl bg-emerald-500 px-4 py-3 text-sm font-bold text-slate-950 shadow-[0_0_20px_rgba(16,185,129,0.4)] transition hover:bg-emerald-400 disabled:cursor-not-allowed disabled:opacity-50 active:scale-[0.98]"
          >
            ⚡ 发送上电指令（POWER_ON）
          </button>

          <div className="mt-auto pt-6">
            <p className="text-[11px] leading-relaxed text-slate-500 border-t border-slate-800 pt-4">
              当前 Demo 默认使用本地 EMQX（ws://localhost:8083/mqtt）作为
              MQTT Broker，后续可切换到线上集群并通过环境变量注入连接参数。
            </p>
          </div>
        </section>
      </main>
    </div>
  );
}