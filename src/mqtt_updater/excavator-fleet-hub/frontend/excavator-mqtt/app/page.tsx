 "use client";

import { useState, type FormEvent } from "react";
import { useRouter } from "next/navigation";

export default function LoginPage() {
  const router = useRouter();
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");

  const handleSubmit = (e: FormEvent) => {
    e.preventDefault();
    setError("");

    if (!username || !password) {
      setError("请输入账号和密码");
      return;
    }

    setLoading(true);

    setTimeout(() => {
      setLoading(false);
      sessionStorage.setItem("operator_name", username);
      router.push("/dashboard");
    }, 400);
  };

  return (
    <div className="flex min-h-screen items-center justify-center bg-gradient-to-br from-slate-900 via-slate-950 to-slate-900 font-sans px-4">
      <div className="w-full max-w-md rounded-3xl border border-white/10 bg-slate-900/70 p-8 shadow-[0_0_60px_rgba(15,23,42,0.9)] backdrop-blur-xl">
        <div className="mb-8 text-center">
          <p className="text-xs uppercase tracking-[0.3em] text-emerald-400/80">
            Excavator Fleet Hub
          </p>
          <h1 className="mt-3 text-2xl font-semibold tracking-tight text-slate-50">
            远程挖掘机监控控制台
          </h1>
          <p className="mt-2 text-sm text-slate-400">
            登录以管理机队设备、查看 MQTT 遥测数据并下发控制指令。
          </p>
        </div>

        <form onSubmit={handleSubmit} className="space-y-6">
          <div className="space-y-2">
            <label className="block text-sm font-medium text-slate-200">
              操作员账号
            </label>
            <input
              value={username}
              onChange={(e) => setUsername(e.target.value)}
              placeholder="例如：quinn.ops"
              className="w-full rounded-xl border border-white/10 bg-white/5 px-3 py-2.5 text-sm text-slate-50 outline-none ring-0 transition placeholder:text-slate-500 focus:border-emerald-400/50 focus:bg-white/10 focus:ring-2 focus:ring-emerald-500/40"
            />
          </div>

          <div className="space-y-2">
            <label className="block text-sm font-medium text-slate-200">
              密码
            </label>
            <input
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="输入任意密码（当前为本地演示环境）"
              className="w-full rounded-xl border border-white/10 bg-white/5 px-3 py-2.5 text-sm text-slate-50 outline-none ring-0 transition placeholder:text-slate-500 focus:border-emerald-400/50 focus:bg-white/10 focus:ring-2 focus:ring-emerald-500/40"
            />
          </div>

          {error && (
            <p className="text-sm text-rose-400" role="alert">
              {error}
            </p>
          )}

          <button
            type="submit"
            disabled={loading}
            className="flex w-full items-center justify-center gap-2 rounded-xl bg-emerald-500 px-4 py-2.5 text-sm font-semibold text-slate-950 shadow-[0_0_25px_rgba(16,185,129,0.55)] transition hover:bg-emerald-400 disabled:cursor-not-allowed disabled:opacity-60"
          >
            {loading ? "正在校验..." : "进入机队 Dashboard"}
          </button>

          <p className="mt-2 text-center text-xs text-slate-500">
            当前为前端本地 Demo：任意账号密码均可登录，后续可接入真实
            Supabase/Auth 服务。
          </p>
        </form>
      </div>
    </div>
  );
}
