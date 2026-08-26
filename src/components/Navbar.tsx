import React from "react";
import { LocomotionMode } from "../types";
import { Compass, Sparkles, Terminal, RotateCcw, ShieldCheck, Cpu, Radio } from "lucide-react";
import { downloadFullWorkspaceAsShellScript } from "../utils/workspaceExport";
import { ROS2_JAZZY_WORKSPACE_FILES } from "../data/ros2WorkspaceData";

interface NavbarProps {
  currentMode: LocomotionMode;
  onResetSim: () => void;
  onOpenAICopilot: () => void;
  activeView: "simulation" | "workspace" | "urdf" | "sicslam";
  setActiveView: (view: "simulation" | "workspace" | "urdf" | "sicslam") => void;
}

export const Navbar: React.FC<NavbarProps> = ({
  currentMode,
  onResetSim,
  onOpenAICopilot,
  activeView,
  setActiveView,
}) => {
  return (
    <header className="w-full bg-slate-900 border-b border-slate-800 px-6 py-3 flex flex-wrap items-center justify-between gap-4 font-mono text-slate-200 shadow-xl sticky top-0 z-40 shrink-0">
      {/* Brand & Workspace Info */}
      <div className="flex items-center gap-4">
        <div className="w-8 h-8 bg-emerald-500 rounded-lg flex items-center justify-center font-bold text-slate-950 shadow-[0_0_12px_rgba(52,211,153,0.4)]">
          X
        </div>
        <div>
          <h1 className="text-base font-semibold tracking-tight leading-none text-slate-100 flex items-center gap-2">
            CaveX Explorer Pro
            <span className="text-[10px] px-1.5 py-0.5 rounded bg-slate-800 text-emerald-400 border border-slate-700 font-mono">
              v2.4
            </span>
          </h1>
          <p className="text-[10px] text-slate-500 uppercase tracking-widest font-bold mt-0.5">
            ROS2 Jazzy • Gazebo Harmonic
          </p>
        </div>
      </div>

      {/* Telemetry Quick Badges */}
      <div className="hidden lg:flex items-center gap-4 text-xs">
        <div className="flex flex-col items-start">
          <span className="text-[9px] text-slate-500 uppercase font-bold tracking-wider">Workspace</span>
          <span className="text-xs font-mono text-emerald-400">cave_exploration_ws</span>
        </div>
        <div className="h-6 w-px bg-slate-800"></div>
        <div className="flex items-center gap-2">
          <div className="px-2.5 py-1 bg-slate-800/90 rounded text-[11px] text-slate-300 border border-slate-700 font-mono">
            Node: /chimera_controller
          </div>
          <div className="px-2.5 py-1 bg-emerald-500/10 text-emerald-400 rounded text-[11px] border border-emerald-500/20 font-mono flex items-center gap-1.5 shadow-[0_0_8px_rgba(52,211,153,0.15)]">
            <span className="w-1.5 h-1.5 rounded-full bg-emerald-400 animate-pulse"></span>
            Sim Time: 00:42:15.8
          </div>
        </div>
      </div>

      {/* Center View Selector Tabs */}
      <div className="flex items-center gap-1 bg-slate-950 p-1 rounded-lg border border-slate-800 text-xs">
        <button
          id="nav-tab-sim"
          onClick={() => setActiveView("simulation")}
          className={`px-3 py-1.5 rounded transition flex items-center gap-1.5 ${
            activeView === "simulation"
              ? "bg-emerald-500/20 text-emerald-400 border border-emerald-500/40 font-semibold shadow-[0_0_10px_rgba(52,211,153,0.2)]"
              : "text-slate-400 hover:text-slate-200"
          }`}
        >
          <Compass className="w-3.5 h-3.5" /> 3D Cave Visualizer
        </button>
        <button
          id="nav-tab-sicslam"
          onClick={() => setActiveView("sicslam")}
          className={`px-3 py-1.5 rounded transition flex items-center gap-1.5 ${
            activeView === "sicslam"
              ? "bg-cyan-500/20 text-cyan-300 border border-cyan-500/40 font-semibold shadow-[0_0_10px_rgba(6,182,212,0.25)]"
              : "text-slate-400 hover:text-slate-200"
          }`}
        >
          <Radio className="w-3.5 h-3.5 text-cyan-400 animate-pulse" /> GTSAM-SLAM Engine
        </button>
        <button
          id="nav-tab-urdf"
          onClick={() => setActiveView("urdf")}
          className={`px-3 py-1.5 rounded transition flex items-center gap-1.5 ${
            activeView === "urdf"
              ? "bg-emerald-500/20 text-emerald-400 border border-emerald-500/40 font-semibold shadow-[0_0_10px_rgba(52,211,153,0.2)]"
              : "text-slate-400 hover:text-slate-200"
          }`}
        >
          <Cpu className="w-3.5 h-3.5" /> URDF Kinematics
        </button>
        <button
          id="nav-tab-workspace"
          onClick={() => setActiveView("workspace")}
          className={`px-3 py-1.5 rounded transition flex items-center gap-1.5 ${
            activeView === "workspace"
              ? "bg-emerald-500/20 text-emerald-400 border border-emerald-500/40 font-semibold shadow-[0_0_10px_rgba(52,211,153,0.2)]"
              : "text-slate-400 hover:text-slate-200"
          }`}
        >
          <Terminal className="w-3.5 h-3.5" /> ROS 2 Workspace
        </button>
      </div>

      {/* Right Quick Actions */}
      <div className="flex items-center gap-2 text-xs">
        <button
          id="btn-reset-sim"
          onClick={onResetSim}
          className="px-2.5 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 flex items-center gap-1 transition"
          title="Reset Robot Pose to Start of Cave"
        >
          <RotateCcw className="w-3.5 h-3.5 text-amber-400" /> Reset Sim
        </button>

        <button
          id="btn-nav-ai"
          onClick={onOpenAICopilot}
          className="px-3 py-1.5 rounded bg-purple-950/80 hover:bg-purple-900 text-purple-300 border border-purple-800/80 font-bold flex items-center gap-1.5 transition shadow"
        >
          <Sparkles className="w-3.5 h-3.5 text-purple-400" /> AI Copilot
        </button>

        <button
          id="btn-nav-export"
          onClick={() => downloadFullWorkspaceAsShellScript(ROS2_JAZZY_WORKSPACE_FILES)}
          className="px-3 py-1.5 rounded bg-emerald-500 hover:bg-emerald-400 text-slate-950 font-bold flex items-center gap-1.5 transition shadow-lg shadow-emerald-500/20"
        >
          <Terminal className="w-3.5 h-3.5 text-slate-950" /> Export ROS 2
        </button>
      </div>
    </header>
  );
};
