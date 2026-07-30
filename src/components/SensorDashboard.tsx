import React, { useEffect, useRef, useState } from "react";
import { RobotState, SensorData } from "../types";
import {
  Camera,
  Radio,
  Waves,
  Activity,
  Scan,
  ShieldAlert,
  Battery,
  BatteryCharging,
  BatteryWarning,
  Zap,
  AlertTriangle,
  RefreshCw,
  Video,
  Circle,
  Sun,
  Maximize2,
} from "lucide-react";

interface SensorDashboardProps {
  robotState: RobotState;
  sensorData: SensorData;
  onSetBattery?: (val: number) => void;
}

export const SensorDashboard: React.FC<SensorDashboardProps> = ({
  robotState,
  sensorData,
  onSetBattery,
}) => {
  const lidarCanvasRef = useRef<HTMLCanvasElement>(null);
  const cameraCanvasRef = useRef<HTMLCanvasElement>(null);

  const isLowBattery = robotState.battery < 20;

  // Video Recording State
  const [isRecording, setIsRecording] = useState(false);
  const [recordSeconds, setRecordSeconds] = useState(0);

  // FPV Brightness / Gain state
  const [brightnessLevel, setBrightnessLevel] = useState<"standard" | "high" | "ultra">("high");

  // Timer for video recording
  useEffect(() => {
    let interval: any = null;
    if (isRecording) {
      interval = setInterval(() => {
        setRecordSeconds((s) => s + 1);
      }, 1000);
    } else {
      setRecordSeconds(0);
    }
    return () => clearInterval(interval);
  }, [isRecording]);

  const formatRecordTime = (totalSec: number) => {
    const mins = Math.floor(totalSec / 60)
      .toString()
      .padStart(2, "0");
    const secs = (totalSec % 60).toString().padStart(2, "0");
    return `${mins}:${secs}`;
  };

  // Render Simulated Visual Camera with ORB SLAM Features (Enhanced Brightness)
  useEffect(() => {
    const canvas = cameraCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;

    // Background cavern illumination based on position & mode (BRIGHTENED)
    const isWater = robotState.position.x >= -5 && robotState.position.x <= 12;
    const isAirPocket = robotState.position.x > 12;

    // Gain multipliers
    const gainFactor = brightnessLevel === "ultra" ? 1.8 : brightnessLevel === "high" ? 1.4 : 1.0;

    const grad = ctx.createLinearGradient(0, 0, 0, h);
    if (isWater && robotState.position.z < 0) {
      grad.addColorStop(0, brightnessLevel === "ultra" ? "#004080" : "#002b52");
      grad.addColorStop(1, brightnessLevel === "ultra" ? "#001a33" : "#001224");
    } else if (isAirPocket) {
      grad.addColorStop(0, brightnessLevel === "ultra" ? "#3b1e66" : "#28124a");
      grad.addColorStop(1, brightnessLevel === "ultra" ? "#1a0d33" : "#100724");
    } else {
      grad.addColorStop(0, brightnessLevel === "ultra" ? "#4a3c30" : "#33271f");
      grad.addColorStop(1, brightnessLevel === "ultra" ? "#241a14" : "#1a120d");
    }
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    // High Intensity Headlight / High-Gain Exposure Light
    const spotlightAlpha = Math.min(0.9, 0.55 * gainFactor);
    const lightGrad = ctx.createRadialGradient(w / 2, h / 2, 5, w / 2, h / 2, h * 0.9);
    lightGrad.addColorStop(0, `rgba(255, 245, 210, ${spotlightAlpha})`);
    lightGrad.addColorStop(0.5, `rgba(255, 215, 170, ${spotlightAlpha * 0.5})`);
    lightGrad.addColorStop(1, "rgba(255, 255, 255, 0.05)");
    ctx.fillStyle = lightGrad;
    ctx.fillRect(0, 0, w, h);

    // Draw cave horizon / tunnel walls (Brighter stroke)
    ctx.strokeStyle = `rgba(255, 255, 255, ${0.35 * gainFactor})`;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(w / 2, h / 2, h * 0.38, 0, Math.PI * 2);
    ctx.stroke();

    // Secondary bright tunnel outline
    ctx.strokeStyle = `rgba(56, 189, 248, ${0.25 * gainFactor})`;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(w / 2, h / 2, h * 0.2, 0, Math.PI * 2);
    ctx.stroke();

    // Draw ORB Feature Trackers (Vivid Lime points with ID tags)
    ctx.fillStyle = "#00ff88";
    ctx.font = "10px monospace";
    for (let i = 0; i < sensorData.featurePointsCount; i++) {
      const seed = i * 137.5;
      const fx = w / 2 + Math.cos(seed) * (h * 0.35 * Math.sin(seed * 0.1));
      const fy = h / 2 + Math.sin(seed) * (h * 0.35 * Math.sin(seed * 0.1));

      ctx.beginPath();
      ctx.arc(fx, fy, 3, 0, Math.PI * 2);
      ctx.fill();

      // Draw feature crosshair
      ctx.strokeStyle = "rgba(0, 255, 136, 0.8)";
      ctx.beginPath();
      ctx.moveTo(fx - 5, fy);
      ctx.lineTo(fx + 5, fy);
      ctx.moveTo(fx, fy - 5);
      ctx.lineTo(fx, fy + 5);
      ctx.stroke();
    }

    // Camera HUD Overlay Text
    ctx.fillStyle = "#38bdf8";
    ctx.font = "10px monospace";
    ctx.fillText(`CAM0: FRONT_RGB [${sensorData.cameraResolution}]`, 10, 18);
    ctx.fillText(`EXPOSURE: +${(gainFactor * 1.5).toFixed(1)} EV (${brightnessLevel.toUpperCase()})`, 10, 32);
    ctx.fillText(`ORB-SLAM3: ${sensorData.featurePointsCount} FEATS`, 10, 46);

    // Recording Badge on Canvas if active
    if (isRecording) {
      ctx.fillStyle = "#ef4444";
      ctx.beginPath();
      ctx.arc(w - 20, 20, 6, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = "#ffffff";
      ctx.font = "bold 11px monospace";
      ctx.fillText(`REC ${formatRecordTime(recordSeconds)}`, w - 90, 24);
    }
  }, [robotState, sensorData, brightnessLevel, isRecording, recordSeconds]);

  // Render 2D SLAM Occupancy Grid Map & LiDAR Scan Rays
  useEffect(() => {
    const canvas = lidarCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2;

    // Dark grid background
    ctx.fillStyle = "#090d16";
    ctx.fillRect(0, 0, w, h);

    // Grid lines
    ctx.strokeStyle = "#1e293b";
    ctx.lineWidth = 1;
    const gridSize = 20;
    for (let x = 0; x < w; x += gridSize) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let y = 0; y < h; y += gridSize) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // Radar distance rings
    ctx.strokeStyle = "rgba(56, 189, 248, 0.2)";
    [40, 80, 120].forEach((r) => {
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.stroke();
    });

    // Render 2D LiDAR Range Readings (Cyan/Lime dots & laser beams)
    ctx.strokeStyle = "rgba(0, 255, 180, 0.15)";
    ctx.fillStyle = "#00ffb4";

    const ranges = sensorData.lidarRanges;
    const step = (Math.PI * 2) / ranges.length;

    ctx.beginPath();
    ranges.forEach((range, i) => {
      const angle = i * step;
      const scale = 16; // 1 meter = 16px
      const lx = cx + Math.cos(angle) * (range * scale);
      const ly = cy + Math.sin(angle) * (range * scale);

      ctx.moveTo(cx, cy);
      ctx.lineTo(lx, ly);

      // Point at hit location
      ctx.fillRect(lx - 1.5, ly - 1.5, 3, 3);
    });
    ctx.stroke();

    // Robot Position Marker in 2D SLAM Map
    ctx.fillStyle = "#f59e0b";
    ctx.beginPath();
    ctx.arc(cx, cy, 5, 0, Math.PI * 2);
    ctx.fill();

    // Robot Heading Direction Arrow
    const yawRad = (robotState.orientation.z * Math.PI) / 180;
    ctx.strokeStyle = "#f59e0b";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(yawRad) * 14, cy + Math.sin(yawRad) * 14);
    ctx.stroke();

    // Map Legend
    ctx.fillStyle = "#94a3b8";
    ctx.font = "10px monospace";
    ctx.fillText(`SLAM Toolbox 2D Grid (10m x 10m)`, 8, 16);
    ctx.fillText(`Confidence: ${sensorData.slamConfidence.toFixed(0)}%`, 8, 30);
  }, [sensorData, robotState]);

  return (
    <div className="w-full bg-slate-900 border border-slate-800 rounded-xl p-4 shadow-xl flex flex-col gap-4 text-slate-200">
      <div className="flex items-center justify-between border-b border-slate-800 pb-2">
        <h3 className="font-semibold text-sm font-mono flex items-center gap-2 text-sky-400">
          <Activity className="w-4 h-4 text-sky-400" />
          Autonomous Multi-Sensor Perception & Power Suite
        </h3>
        <span className="text-xs font-mono text-emerald-400 bg-emerald-950/60 border border-emerald-800 px-2 py-0.5 rounded">
          ROS 2 Node: /sensor_fusion_slam (30Hz)
        </span>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
        {/* Sensor 1: Visual RGB Camera Feed */}
        <div className="bg-slate-950 rounded-lg p-2.5 border border-slate-800 flex flex-col justify-between gap-2">
          <div className="flex items-center justify-between text-xs font-mono text-slate-400">
            <span className="flex items-center gap-1.5 text-sky-300 font-semibold">
              <Camera className="w-3.5 h-3.5 text-sky-400" /> RGB Camera Feed
            </span>
            {isRecording ? (
              <span className="text-[10px] px-2 py-0.5 rounded bg-rose-500/20 text-rose-400 border border-rose-500/40 font-mono font-bold flex items-center gap-1 animate-pulse">
                <Circle className="w-2 h-2 fill-rose-500 text-rose-500" /> REC {formatRecordTime(recordSeconds)}
              </span>
            ) : (
              <span className="text-slate-500 text-[10px]">Topic: /camera/image_raw</span>
            )}
          </div>

          <div className="relative w-full aspect-video rounded overflow-hidden border border-slate-800 bg-black">
            <canvas ref={cameraCanvasRef} width={320} height={180} className="w-full h-full object-cover" />
            <div className="absolute top-2 left-2 flex items-center gap-1">
              <span className="bg-slate-900/80 backdrop-blur text-sky-300 border border-slate-700 text-[9px] px-1.5 py-0.5 rounded font-mono font-bold">
                FPV ISO: 3200
              </span>
              <span className="bg-emerald-500/20 text-emerald-400 border border-emerald-500/30 text-[9px] px-1.5 py-0.5 rounded font-mono font-bold">
                +{brightnessLevel === "ultra" ? "3.0" : brightnessLevel === "high" ? "1.5" : "0.0"} EV
              </span>
            </div>
            {robotState.headlightOn && (
              <span className="absolute bottom-2 right-2 bg-amber-500/20 text-amber-300 border border-amber-500/40 text-[9px] px-1.5 py-0.5 rounded font-mono font-bold">
                LIGHT ON
              </span>
            )}
          </div>

          {/* FPV Controls: Video Record Button & Brightness Boost */}
          <div className="flex items-center gap-1.5 pt-1 border-t border-slate-800/80 font-mono text-[10px]">
            {/* Record Video Button */}
            <button
              id="btn-record-video-dash"
              onClick={() => setIsRecording(!isRecording)}
              className={`flex-1 py-1 px-2 rounded font-bold transition flex items-center justify-center gap-1.5 ${
                isRecording
                  ? "bg-rose-600 hover:bg-rose-500 text-white shadow-[0_0_10px_rgba(239,68,68,0.5)] animate-pulse"
                  : "bg-slate-900 hover:bg-slate-800 text-rose-400 border border-rose-500/30 hover:border-rose-500"
              }`}
              title={isRecording ? "Stop Video Recording" : "Start Video Recording"}
            >
              <Video className="w-3.5 h-3.5" />
              {isRecording ? `REC (${formatRecordTime(recordSeconds)})` : "Record Video"}
            </button>

            {/* Brightness Boost Toggle */}
            <button
              id="btn-toggle-camera-brightness"
              onClick={() =>
                setBrightnessLevel((prev) =>
                  prev === "standard" ? "high" : prev === "high" ? "ultra" : "standard"
                )
              }
              className="py-1 px-2 rounded bg-slate-900 hover:bg-slate-800 text-amber-300 border border-slate-700 font-bold transition flex items-center gap-1"
              title="Boost FPV Camera Brightness / Exposure"
            >
              <Sun className="w-3.5 h-3.5 text-amber-400" />
              <span className="uppercase text-[9px]">{brightnessLevel}</span>
            </button>
          </div>
        </div>

        {/* Sensor 2: 2D/3D LiDAR Occupancy Grid */}
        <div className="bg-slate-950 rounded-lg p-2.5 border border-slate-800 flex flex-col gap-2">
          <div className="flex items-center justify-between text-xs font-mono text-slate-400">
            <span className="flex items-center gap-1.5 text-emerald-300 font-semibold">
              <Scan className="w-3.5 h-3.5 text-emerald-400" /> 360° LiDAR Scan Map
            </span>
            <span className="text-slate-500">Topic: /scan (20Hz)</span>
          </div>

          <div className="relative w-full aspect-video rounded overflow-hidden border border-slate-800 bg-slate-950 flex items-center justify-center">
            <canvas ref={lidarCanvasRef} width={320} height={180} className="w-full h-full object-contain" />
          </div>
        </div>

        {/* Sensor 3: Underwater Sonar Acoustic Bathymetry */}
        <div className="bg-slate-950 rounded-lg p-2.5 border border-slate-800 flex flex-col gap-2">
          <div className="flex items-center justify-between text-xs font-mono text-slate-400">
            <span className="flex items-center gap-1.5 text-rose-300 font-semibold">
              <Waves className="w-3.5 h-3.5 text-rose-400" /> Underwater Sonar Depth
            </span>
            <span className="text-slate-500">Topic: /sonar/echo</span>
          </div>

          <div className="w-full aspect-video rounded border border-slate-800 bg-slate-950 p-2.5 flex flex-col justify-between font-mono text-xs">
            <div className="flex items-center justify-between">
              <span className="text-slate-400 text-[11px]">Submerged Floor Depth:</span>
              <strong className="text-rose-400 text-xs">{sensorData.sonarDepth.toFixed(2)} meters</strong>
            </div>

            {/* Echo Signal Bar */}
            <div className="flex flex-col gap-1">
              <div className="flex justify-between text-[10px] text-slate-500">
                <span>Acoustic Echo Strength</span>
                <span>{sensorData.sonarEchoStrength.toFixed(0)} dB</span>
              </div>
              <div className="w-full bg-slate-800 h-2 rounded-full overflow-hidden">
                <div
                  className="bg-gradient-to-r from-rose-500 to-amber-400 h-full transition-all duration-300"
                  style={{ width: `${sensorData.sonarEchoStrength}%` }}
                />
              </div>
            </div>

            <div className="flex items-center justify-between text-[10px] text-slate-400 pt-1 border-t border-slate-800/80">
              <span>Waterline Surface: <strong className="text-cyan-400">z = 0.0m</strong></span>
              <span>Submersion: <strong className={robotState.waterSubmerged ? "text-cyan-400" : "text-slate-500"}>{robotState.waterSubmerged ? "ACTIVE" : "NONE"}</strong></span>
            </div>
          </div>
        </div>

        {/* Sensor 4 / BMS: Visual Battery Health & Power Monitor */}
        <div
          className={`bg-slate-950 rounded-lg p-2.5 border transition-all flex flex-col justify-between gap-2 font-mono text-xs ${
            isLowBattery
              ? "border-rose-500/80 bg-rose-950/20 shadow-[0_0_15px_rgba(244,63,94,0.25)]"
              : "border-slate-800"
          }`}
        >
          <div className="flex items-center justify-between text-xs font-mono">
            <span
              className={`flex items-center gap-1.5 font-semibold ${
                isLowBattery ? "text-rose-400" : "text-emerald-300"
              }`}
            >
              {isLowBattery ? (
                <BatteryWarning className="w-4 h-4 text-rose-500 animate-bounce" />
              ) : (
                <BatteryCharging className="w-4 h-4 text-emerald-400" />
              )}
              BMS Battery Health
            </span>
            <span
              className={`text-[10px] px-1.5 py-0.5 rounded border font-bold uppercase ${
                isLowBattery
                  ? "bg-rose-500/20 text-rose-300 border-rose-500/40 animate-pulse"
                  : "bg-emerald-950/80 text-emerald-400 border-emerald-800"
              }`}
            >
              {isLowBattery ? "CRITICAL LOW" : "NORMAL"}
            </span>
          </div>

          <div className="flex flex-col gap-1.5 my-0.5">
            <div className="flex items-baseline justify-between">
              <span className="text-slate-400 text-[11px]">Dynamic Charge Level:</span>
              <strong
                className={`text-base font-mono font-bold ${
                  isLowBattery
                    ? "text-rose-400"
                    : robotState.battery < 50
                    ? "text-amber-400"
                    : "text-emerald-400"
                }`}
              >
                {robotState.battery.toFixed(1)}%
              </strong>
            </div>

            {/* Visual Battery Health Progress Bar */}
            <div className="relative w-full bg-slate-900 h-3 rounded-full overflow-hidden border border-slate-800 p-0.5">
              <div
                className={`h-full rounded-full transition-all duration-300 ${
                  isLowBattery
                    ? "bg-gradient-to-r from-red-600 via-rose-500 to-red-500 animate-pulse shadow-[0_0_10px_rgba(244,63,94,0.8)]"
                    : robotState.battery < 50
                    ? "bg-gradient-to-r from-amber-500 to-yellow-400"
                    : "bg-gradient-to-r from-emerald-500 via-teal-400 to-emerald-400"
                }`}
                style={{ width: `${Math.max(0, Math.min(100, robotState.battery))}%` }}
              />
            </div>

            {/* Warning Banner when < 20% */}
            {isLowBattery && (
              <div className="p-1.5 rounded bg-rose-500/10 border border-rose-500/30 text-rose-300 text-[10px] flex items-center gap-1.5 font-sans animate-pulse">
                <AlertTriangle className="w-3.5 h-3.5 shrink-0 text-rose-400" />
                <span>
                  <strong>LOW BATTERY WARNING:</strong> Battery &lt; 20%! Triggering RTL protocol.
                </span>
              </div>
            )}
          </div>

          {/* BMS Power Specs */}
          <div className="grid grid-cols-2 gap-1 text-[10px] pt-1 border-t border-slate-800/80 text-slate-400">
            <div>
              Bus: <strong className="text-slate-200">{(22.2 * (robotState.battery / 100)).toFixed(1)}V</strong>
            </div>
            <div>
              Draw:{" "}
              <strong className="text-slate-200">
                {robotState.mode === "FLYING"
                  ? "38.2A"
                  : robotState.mode === "SAILING"
                  ? "8.4A"
                  : "12.1A"}
              </strong>
            </div>
          </div>

          {/* Quick Test Actions */}
          {onSetBattery && (
            <div className="flex items-center gap-1.5 pt-1 border-t border-slate-800/60">
              <button
                id="btn-test-low-battery"
                onClick={() => onSetBattery(15)}
                className="flex-1 py-1 rounded bg-rose-950/60 hover:bg-rose-900/80 text-rose-300 border border-rose-800/60 text-[10px] font-bold transition flex items-center justify-center gap-1"
                title="Simulate low battery (<20%)"
              >
                <Zap className="w-3 h-3 text-rose-400" /> Test 15%
              </button>
              <button
                id="btn-recharge-battery"
                onClick={() => onSetBattery(100)}
                className="flex-1 py-1 rounded bg-slate-900 hover:bg-slate-800 text-slate-300 border border-slate-700 text-[10px] font-medium transition flex items-center justify-center gap-1"
              >
                <RefreshCw className="w-3 h-3 text-emerald-400" /> Charge 100%
              </button>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

