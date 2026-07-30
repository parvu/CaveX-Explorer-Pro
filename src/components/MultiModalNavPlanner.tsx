import React from "react";
import { RobotState, LocomotionMode, MissionStatus, Waypoint, SensorData } from "../types";
import { Play, Pause, RotateCcw, ArrowRight, CheckCircle2, Navigation, Compass, Shield, Zap, Footprints, Flame } from "lucide-react";

interface MultiModalNavPlannerProps {
  robotState: RobotState;
  setRobotState: React.Dispatch<React.SetStateAction<RobotState>>;
  missionStatus: MissionStatus;
  setMissionStatus: React.Dispatch<React.SetStateAction<MissionStatus>>;
  sensorData: SensorData;
  setSensorData: React.Dispatch<React.SetStateAction<SensorData>>;
}

export const MultiModalNavPlanner: React.FC<MultiModalNavPlannerProps> = ({
  robotState,
  setRobotState,
  missionStatus,
  setMissionStatus,
  sensorData,
  setSensorData,
}) => {
  // Manual Kinematic Mode Switch Handler
  const handleModeSwitch = (newMode: LocomotionMode) => {
    let newZ = robotState.position.z;
    let newPropellerRpm = 0;
    let buoyancy = 0;

    if (newMode === "WALKING") {
      newZ = 0.6; // Dry land height
      newPropellerRpm = 0;
      buoyancy = 0;
    } else if (newMode === "SAILING") {
      newZ = -0.05; // Water surface height
      newPropellerRpm = 300;
      buoyancy = 41.2; // Buoyancy balances robot weight
    } else if (newMode === "FLYING") {
      newZ = 1.8; // Elevated aerial altitude
      newPropellerRpm = 4200; // High RPM for flight
      buoyancy = 0;
    }

    setRobotState((prev) => ({
      ...prev,
      mode: newMode,
      position: { ...prev.position, z: newZ },
      propellerRpm: newPropellerRpm,
      buoyancyForce: buoyancy,
      waterSubmerged: newMode === "SAILING",
      inAirPocket: newMode === "FLYING",
    }));

    // Log FSM transition
    setMissionStatus((prev) => ({
      ...prev,
      logs: [`[FSM] Manual Kinematic Chain Switch executed -> ${newMode}`, ...prev.logs.slice(0, 15)],
    }));
  };

  // Autonomous Mission Step Handler
  const handleNextWaypoint = () => {
    const nextIdx = (missionStatus.currentWaypointIndex + 1) % missionStatus.waypoints.length;
    const targetWp = missionStatus.waypoints[nextIdx];

    handleModeSwitch(targetWp.targetMode);

    setRobotState((prev) => ({
      ...prev,
      position: targetWp.position,
    }));

    const updatedWaypoints = missionStatus.waypoints.map((wp, idx) => ({
      ...wp,
      completed: idx <= nextIdx,
    }));

    setMissionStatus((prev) => ({
      ...prev,
      currentWaypointIndex: nextIdx,
      waypoints: updatedWaypoints,
      logs: [
        `[Nav2] Reached Waypoint: ${targetWp.name} (${targetWp.section})`,
        `[FSM] Environment Medium: ${targetWp.section} -> Kinematic Chain: ${targetWp.targetMode}`,
        ...prev.logs.slice(0, 15),
      ],
    }));
  };

  // Toggle Autonomous Mission Auto-pilot Loop
  const toggleAutoMission = () => {
    const active = !missionStatus.active;
    setMissionStatus((prev) => ({
      ...prev,
      active,
      logs: [active ? "[Nav2] Autonomous Mission Started." : "[Nav2] Autonomous Mission Paused.", ...prev.logs.slice(0, 15)],
    }));
  };

  // Manual Nudge Position (WASD / Movement)
  const nudgePosition = (dx: number, dy: number, dz: number) => {
    setRobotState((prev) => {
      const nx = Math.max(-20, Math.min(22, prev.position.x + dx));
      const ny = Math.max(-5, Math.min(5, prev.position.y + dy));
      let nz = prev.position.z + dz;

      // Auto detect mode switch based on environment x and z bounds
      let detectedMode: LocomotionMode = prev.mode;

      if (nx >= -5 && nx <= 12 && nz <= 0.2) {
        // Flooded water section
        detectedMode = "SAILING";
        nz = Math.min(0, nz);
      } else if (nx > 12 && nz > 0.8) {
        // Air pocket shaft
        detectedMode = "FLYING";
      } else if (nx < -5 || (nx > 12 && nz <= 0.8)) {
        // Dry ground
        detectedMode = "WALKING";
        nz = 0.6;
      }

      return {
        ...prev,
        position: { x: nx, y: ny, z: nz },
        mode: detectedMode,
      };
    });
  };

  return (
    <div className="w-full bg-slate-900 border border-slate-800 rounded-xl p-4 shadow-xl flex flex-col gap-4 text-slate-200">
      <div className="flex flex-wrap items-center justify-between border-b border-slate-800 pb-3 gap-2">
        <div>
          <h3 className="font-semibold text-base font-mono flex items-center gap-2 text-amber-400">
            <Navigation className="w-5 h-5 text-amber-400" />
            Multi-Modal Autonomous Navigation & FSM Transition Planner
          </h3>
          <p className="text-xs text-slate-400 font-sans mt-0.5">
            Manages dynamic URDF kinematic switches across Dry Cave, Flooded Water, and Air Pocket sections.
          </p>
        </div>

        {/* Action Buttons */}
        <div className="flex items-center gap-2">
          <button
            id="btn-auto-mission"
            onClick={toggleAutoMission}
            className={`px-3 py-1.5 rounded-lg text-xs font-mono font-bold flex items-center gap-1.5 transition ${
              missionStatus.active
                ? "bg-amber-500 text-slate-950 hover:bg-amber-400 shadow-lg shadow-amber-500/20"
                : "bg-emerald-600 text-white hover:bg-emerald-500 shadow-lg shadow-emerald-600/20"
            }`}
          >
            {missionStatus.active ? <Pause className="w-4 h-4" /> : <Play className="w-4 h-4" />}
            {missionStatus.active ? "Pause Mission" : "Start Autonomous Mission"}
          </button>

          <button
            id="btn-next-wp"
            onClick={handleNextWaypoint}
            className="px-3 py-1.5 rounded-lg bg-sky-600 hover:bg-sky-500 text-white text-xs font-mono font-semibold flex items-center gap-1.5 transition"
          >
            Step Waypoint <ArrowRight className="w-3.5 h-3.5" />
          </button>
        </div>
      </div>

      {/* Mode Selector Cards */}
      <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
        {/* Mode 1: WALKING */}
        <div
          id="mode-card-walking"
          onClick={() => handleModeSwitch("WALKING")}
          className={`cursor-pointer rounded-lg p-3 border transition flex flex-col gap-1.5 ${
            robotState.mode === "WALKING"
              ? "bg-emerald-500/20 border-emerald-500/50 text-emerald-400 shadow-[0_0_12px_rgba(52,211,153,0.15)]"
              : "bg-slate-950/60 border-slate-800 hover:border-slate-700 text-slate-300"
          }`}
        >
          <div className="flex items-center justify-between">
            <span className="font-mono text-xs font-bold flex items-center gap-1.5 text-emerald-400">
              <Footprints className="w-4 h-4" /> WALKING (Quadruped)
            </span>
            <span
              className={`w-2 h-2 rounded-full ${
                robotState.mode === "WALKING"
                  ? "bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,0.8)] animate-pulse"
                  : "bg-slate-600"
              }`}
            ></span>
          </div>
          <p className="text-[11px] text-slate-400 leading-snug">
            4 Leg chains active. High ground friction, zero water buoyancy. Navigates dry stalagmites ($z &gt; 0$).
          </p>
          <div className="text-[10px] font-mono text-emerald-300/80 bg-slate-900/90 px-2 py-0.5 rounded border border-emerald-500/20 w-fit">
            Joints: Leg FL, FR, BL, BR
          </div>
        </div>

        {/* Mode 2: SAILING */}
        <div
          id="mode-card-sailing"
          onClick={() => handleModeSwitch("SAILING")}
          className={`cursor-pointer rounded-lg p-3 border transition flex flex-col gap-1.5 ${
            robotState.mode === "SAILING"
              ? "bg-emerald-500/20 border-emerald-500/50 text-emerald-400 shadow-[0_0_12px_rgba(52,211,153,0.15)]"
              : "bg-slate-950/60 border-slate-800 hover:border-slate-700 text-slate-300"
          }`}
        >
          <div className="flex items-center justify-between">
            <span className="font-mono text-xs font-bold flex items-center gap-1.5 text-cyan-400">
              <Compass className="w-4 h-4" /> SAILING (Hydrofoil)
            </span>
            <span
              className={`w-2 h-2 rounded-full ${
                robotState.mode === "SAILING"
                  ? "bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,0.8)] animate-pulse"
                  : "bg-slate-600"
              }`}
            ></span>
          </div>
          <p className="text-[11px] text-slate-400 leading-snug">
            Pontoons deployed, legs float. Hydrodynamic buoyancy active ($F_b=41.2N$). Water surface ($z=0$).
          </p>
          <div className="text-[10px] font-mono text-cyan-300/80 bg-slate-900/90 px-2 py-0.5 rounded border border-cyan-500/20 w-fit">
            Joints: Pontoon Left/Right
          </div>
        </div>

        {/* Mode 3: FLYING */}
        <div
          id="mode-card-flying"
          onClick={() => handleModeSwitch("FLYING")}
          className={`cursor-pointer rounded-lg p-3 border transition flex flex-col gap-1.5 ${
            robotState.mode === "FLYING"
              ? "bg-emerald-500/20 border-emerald-500/50 text-emerald-400 shadow-[0_0_12px_rgba(52,211,153,0.15)]"
              : "bg-slate-950/60 border-slate-800 hover:border-slate-700 text-slate-300"
          }`}
        >
          <div className="flex items-center justify-between">
            <span className="font-mono text-xs font-bold flex items-center gap-1.5 text-purple-400">
              <Zap className="w-4 h-4" /> FLYING (Quadrotor)
            </span>
            <span
              className={`w-2 h-2 rounded-full ${
                robotState.mode === "FLYING"
                  ? "bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,0.8)] animate-pulse"
                  : "bg-slate-600"
              }`}
            ></span>
          </div>
          <p className="text-[11px] text-slate-400 leading-snug">
            High-RPM thrusters (4200 RPM). Folds legs into skids. Ascends vertical air shafts ($z &gt; 0.8$).
          </p>
          <div className="text-[10px] font-mono text-purple-300/80 bg-slate-900/90 px-2 py-0.5 rounded border border-purple-500/20 w-fit">
            Joints: Rotors FL, FR, BL, BR
          </div>
        </div>
      </div>

      {/* Waypoints Sequence Timeline & Controls */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        {/* Left: Waypoints Sequence */}
        <div className="md:col-span-2 bg-slate-950 p-3 rounded-lg border border-slate-800 flex flex-col gap-2">
          <span className="text-xs font-mono text-slate-400 font-semibold">
            Autonomous Exploration Mission Waypoints
          </span>
          <div className="flex flex-col gap-1.5">
            {missionStatus.waypoints.map((wp, idx) => (
              <div
                key={wp.id}
                className={`flex items-center justify-between p-2 rounded text-xs font-mono border transition ${
                  idx === missionStatus.currentWaypointIndex
                    ? "bg-sky-950/80 border-sky-500 text-sky-200"
                    : wp.completed
                    ? "bg-slate-900 border-emerald-900/60 text-slate-400"
                    : "bg-slate-900/40 border-slate-800/80 text-slate-500"
                }`}
              >
                <div className="flex items-center gap-2">
                  <span
                    className={`w-5 h-5 rounded-full flex items-center justify-center text-[10px] font-bold ${
                      idx === missionStatus.currentWaypointIndex
                        ? "bg-sky-500 text-slate-950"
                        : wp.completed
                        ? "bg-emerald-600 text-white"
                        : "bg-slate-800 text-slate-400"
                    }`}
                  >
                    {idx + 1}
                  </span>
                  <div>
                    <strong className="text-slate-200">{wp.name}</strong>{" "}
                    <span className="text-[11px] text-slate-400">({wp.description})</span>
                  </div>
                </div>

                <div className="flex items-center gap-2">
                  <span className="px-2 py-0.5 rounded text-[10px] bg-slate-800 text-slate-300 border border-slate-700">
                    {wp.targetMode}
                  </span>
                  <span className="text-slate-400 font-mono text-[10px]">
                    x:{wp.position.x} m
                  </span>
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Right: Manual Teleop Joystick Controls */}
        <div className="bg-slate-950 p-3 rounded-lg border border-slate-800 flex flex-col justify-between gap-3 font-mono text-xs">
          <span className="text-slate-400 font-semibold">Manual Drone Teleop Control</span>

          <div className="flex flex-col items-center gap-1.5">
            {/* Forward */}
            <button
              id="teleop-forward"
              onClick={() => nudgePosition(1.0, 0, 0)}
              className="px-3 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-sky-400 font-bold border border-slate-700 w-24 text-center"
            >
              ▲ Forward (W)
            </button>
            <div className="flex gap-2">
              {/* Left */}
              <button
                id="teleop-left"
                onClick={() => nudgePosition(0, -0.8, 0)}
                className="px-3 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-sky-400 font-bold border border-slate-700 text-center"
              >
                ◄ Left (A)
              </button>
              {/* Backward */}
              <button
                id="teleop-backward"
                onClick={() => nudgePosition(-1.0, 0, 0)}
                className="px-3 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-sky-400 font-bold border border-slate-700 text-center"
              >
                ▼ Back (S)
              </button>
              {/* Right */}
              <button
                id="teleop-right"
                onClick={() => nudgePosition(0, 0.8, 0)}
                className="px-3 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-sky-400 font-bold border border-slate-700 text-center"
              >
                Right (D) ►
              </button>
            </div>
            {/* Altitude Up / Down */}
            <div className="flex gap-2 mt-1">
              <button
                id="teleop-up"
                onClick={() => nudgePosition(0, 0, 0.5)}
                className="px-2.5 py-1 rounded bg-purple-950 hover:bg-purple-900 text-purple-300 font-bold border border-purple-800 text-[11px]"
              >
                ▲ Ascend (Q)
              </button>
              <button
                id="teleop-down"
                onClick={() => nudgePosition(0, 0, -0.5)}
                className="px-2.5 py-1 rounded bg-purple-950 hover:bg-purple-900 text-purple-300 font-bold border border-purple-800 text-[11px]"
              >
                ▼ Descend (E)
              </button>
            </div>
          </div>

          <div className="text-[10px] text-slate-500 text-center">
            Position auto-triggers kinematic state transitions when crossing z / x cave boundaries.
          </div>
        </div>
      </div>

      {/* FSM Log Console */}
      <div className="bg-slate-950 rounded-lg p-2.5 border border-slate-800 flex flex-col gap-1 font-mono text-xs">
        <span className="text-[11px] text-slate-400 font-semibold">ROS 2 Navigation FSM Event Log</span>
        <div className="h-20 overflow-y-auto space-y-1 pr-1 text-[11px]">
          {missionStatus.logs.map((log, i) => (
            <div key={i} className="text-slate-300">
              <span className="text-slate-500">[{new Date().toLocaleTimeString()}]</span> {log}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};
