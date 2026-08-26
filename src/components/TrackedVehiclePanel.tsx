import React, { useEffect, useState } from "react";
import { Compass, Route, Gauge, Cog } from "lucide-react";

// Live telemetry for the tracked BlueBoat-like vehicle (Tasks 5/10-13),
// polled from web_telemetry_bridge.py -> /api/telemetry the same way
// GtsamSlamVisualizer.tsx does. No demo/concept fallback numbers here --
// unlike the gtsam_slam system this vehicle has no separate mock version,
// so the panel just shows "offline" when the ROS2 stack isn't posting
// telemetry rather than fabricate anything.
interface TrackedVehicleTelemetry {
  tracked_vehicle_gt: { x: number; y: number; z: number; yaw: number } | null;
  frontier_count: number | null;
  ate_rmse: number | null;
  track_state: "deployed" | "retracted" | "moving" | null;
}

const TRACK_STATE_LABEL: Record<string, string> = {
  deployed: "Deployed (max traction)",
  retracted: "Retracted (transit)",
  moving: "Retracting/deploying...",
};

export const TrackedVehiclePanel: React.FC = () => {
  const [live, setLive] = useState<TrackedVehicleTelemetry | null>(null);

  useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      try {
        const res = await fetch("/api/telemetry");
        const json = await res.json();
        if (!cancelled) setLive(json.live ? json.data : null);
      } catch {
        if (!cancelled) setLive(null);
      }
    };
    poll();
    const id = setInterval(poll, 1000);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, []);

  const isLive = live?.tracked_vehicle_gt != null;
  const gt = live?.tracked_vehicle_gt;

  return (
    <div className="w-full bg-slate-900 border border-slate-800 rounded-xl p-4 shadow-2xl flex flex-col gap-3 text-slate-200 font-mono">
      <div className="flex items-center justify-between border-b border-slate-800 pb-3">
        <h3 className="text-sm font-bold text-slate-100 flex items-center gap-2">
          <Cog className="w-4 h-4 text-teal-400" />
          BlueBoat-like tracked vehicle (ArduPilot Rover SITL)
        </h3>
        {isLive ? (
          <span className="text-[10px] text-emerald-400 bg-emerald-950 px-1.5 py-0.5 rounded border border-emerald-800 font-bold">
            ● live from ROS2
          </span>
        ) : (
          <span className="text-[10px] text-slate-500 bg-slate-950 px-1.5 py-0.5 rounded border border-slate-800 font-bold">
            offline
          </span>
        )}
      </div>

      <div className="grid grid-cols-2 md:grid-cols-4 gap-2 text-xs">
        <div className="bg-slate-950 p-2.5 rounded border border-slate-800 flex flex-col gap-1">
          <span className="text-slate-400 flex items-center gap-1"><Compass className="w-3.5 h-3.5" /> Ground-truth pose</span>
          <strong className="text-cyan-300">
            {gt ? `x=${gt.x.toFixed(2)}, y=${gt.y.toFixed(2)}` : "--"}
          </strong>
        </div>

        <div className="bg-slate-950 p-2.5 rounded border border-slate-800 flex flex-col gap-1">
          <span className="text-slate-400 flex items-center gap-1"><Route className="w-3.5 h-3.5" /> Frontiers (explore_lite)</span>
          <strong className="text-amber-300">
            {live?.frontier_count ?? "--"}
          </strong>
        </div>

        <div className="bg-slate-950 p-2.5 rounded border border-slate-800 flex flex-col gap-1">
          <span className="text-slate-400 flex items-center gap-1"><Gauge className="w-3.5 h-3.5" /> ATE RMSE</span>
          <strong className="text-emerald-300">
            {live?.ate_rmse != null ? `${live.ate_rmse.toFixed(3)} m` : "--"}
          </strong>
        </div>

        <div className="bg-slate-950 p-2.5 rounded border border-slate-800 flex flex-col gap-1">
          <span className="text-slate-400 flex items-center gap-1"><Cog className="w-3.5 h-3.5" /> Tracks</span>
          <strong className="text-slate-200">
            {live?.track_state ? TRACK_STATE_LABEL[live.track_state] : "--"}
          </strong>
        </div>
      </div>
    </div>
  );
};
