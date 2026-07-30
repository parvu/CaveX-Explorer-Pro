import React, { useState, useEffect } from "react";
import { RobotState, SensorData, MissionStatus, LocomotionMode, CameraMode } from "./types";
import { Navbar } from "./components/Navbar";
import { GazeboSimViewport } from "./components/GazeboSimViewport";
import { SensorDashboard } from "./components/SensorDashboard";
import { MultiModalNavPlanner } from "./components/MultiModalNavPlanner";
import { KinematicChainEditor } from "./components/KinematicChainEditor";
import { ROS2WorkspaceExplorer } from "./components/ROS2WorkspaceExplorer";
import { AICopilotModal } from "./components/AICopilotModal";

export default function App() {
  const [activeView, setActiveView] = useState<"simulation" | "workspace" | "urdf">("simulation");
  const [activeCameraMode, setActiveCameraMode] = useState<CameraMode>("orbit");
  const [showLaserBeams, setShowLaserBeams] = useState(true);
  const [showSonarPulse, setShowSonarPulse] = useState(true);
  const [headlight, setHeadlight] = useState(true);
  const [antiCollisionEnabled, setAntiCollisionEnabled] = useState(true);
  const [evasionAlert, setEvasionAlert] = useState<{ active: boolean; obstacle: string }>({
    active: false,
    obstacle: "",
  });

  // Copilot Modal State
  const [isCopilotOpen, setIsCopilotOpen] = useState(false);
  const [copilotFile, setCopilotFile] = useState<any>(undefined);

  // Robot State
  const [robotState, setRobotState] = useState<RobotState>({
    position: { x: -15, y: 0, z: 0.6 },
    orientation: { x: 0, y: 0, z: 0 },
    velocity: { x: 0, y: 0, z: 0 },
    mode: "WALKING",
    jointStates: {},
    propellerRpm: 0,
    headlightOn: true,
    battery: 98,
    buoyancyForce: 0,
    groundContact: true,
    waterSubmerged: false,
    waterDepth: 0,
    inAirPocket: false,
  });

  // Sensor Data
  const [sensorData, setSensorData] = useState<SensorData>({
    cameraActive: true,
    cameraResolution: "1280x720",
    featurePointsCount: 42,
    lidarRanges: Array.from({ length: 36 }, () => 3 + Math.random() * 2.5),
    lidarMinDist: 0.2,
    lidarMaxDist: 12.0,
    sonarDepth: 3.5,
    sonarEchoStrength: 88,
    imuAccel: { x: 0, y: 0, z: 9.81 },
    imuGyro: { x: 0, y: 0, z: 0 },
    slamPose: { x: -15, y: 0, z: 0.6 },
    slamConfidence: 96,
  });

  // Autonomous Mission Status
  const [missionStatus, setMissionStatus] = useState<MissionStatus>({
    active: false,
    currentWaypointIndex: 0,
    waypoints: [
      {
        id: "wp1",
        name: "Dry Cave Entry Corridor",
        targetMode: "WALKING",
        position: { x: -15, y: 0, z: 0.6 },
        section: "DRY_CAVE",
        completed: true,
        description: "Rugged stalagmite terrain inspection",
      },
      {
        id: "wp2",
        name: "Flooded Cave Channel",
        targetMode: "SAILING",
        position: { x: 2, y: 0, z: -0.05 },
        section: "FLOODED_WATER",
        completed: false,
        description: "Hydrofoil water surface navigation (z=0)",
      },
      {
        id: "wp3",
        name: "Air Pocket Vertical Shaft",
        targetMode: "FLYING",
        position: { x: 16, y: 0, z: 2.2 },
        section: "AIR_POCKET",
        completed: false,
        description: "Quadrotor vertical VTOL ascent",
      },
      {
        id: "wp4",
        name: "Elevated Cavern Ledge",
        targetMode: "WALKING",
        position: { x: 18.5, y: 0, z: 2.5 },
        section: "AIR_POCKET",
        completed: false,
        description: "High altitude ledge sample collection",
      },
    ],
    autoTransitions: true,
    fsmState: "IDLE",
    logs: [
      "[System] ROS 2 Jazzy Gazebo Simulator Initialized.",
      "[URDF] Tri-Modal Kinematic Chains Loaded: WALKING, SAILING, FLYING.",
      "[FSM] Default Kinematic Chain: WALKING (Dry Cave Zone 1).",
    ],
  });

  // Keep headlight synced
  useEffect(() => {
    setRobotState((prev) => ({ ...prev, headlightOn: headlight }));
  }, [headlight]);

  // Periodic sensor updates & autonomous mission loop
  useEffect(() => {
    const interval = setInterval(() => {
      // Dynamic sensor readings jitter & subtle battery consumption
      setSensorData((prev) => ({
        ...prev,
        featurePointsCount: 35 + Math.floor(Math.random() * 15),
        lidarRanges: prev.lidarRanges.map((r) => Math.max(0.5, r + (Math.random() - 0.5) * 0.15)),
        sonarEchoStrength: Math.max(40, Math.min(100, prev.sonarEchoStrength + (Math.random() - 0.5) * 4)),
        slamConfidence: Math.max(90, Math.min(99, prev.slamConfidence + (Math.random() - 0.5) * 1)),
      }));

      // Dynamic battery drain & Anti-Collision Avoidance Guard
      setRobotState((prev) => {
        const drainRate = prev.mode === "FLYING" ? 0.08 : prev.mode === "SAILING" ? 0.03 : 0.02;
        const newBattery = Math.max(0, prev.battery - drainRate);
        let newY = prev.position.y;
        let newZ = prev.position.z;

        if (antiCollisionEnabled) {
          // Detect Y wall bounds [-3.0, 3.0]
          if (newY > 2.3) {
            newY -= 0.15; // Auto evasion nudge left
            setEvasionAlert({ active: true, obstacle: "Right Cave Wall Proximity (0.7m)" });
          } else if (newY < -2.3) {
            newY += 0.15; // Auto evasion nudge right
            setEvasionAlert({ active: true, obstacle: "Left Cave Wall Proximity (0.7m)" });
          } else if (prev.mode === "FLYING" && newZ > 4.5) {
            newZ -= 0.15; // Auto evasion nudge down
            setEvasionAlert({ active: true, obstacle: "Shaft Ceiling Obstacle (0.8m)" });
          } else {
            setEvasionAlert({ active: false, obstacle: "" });
          }
        } else {
          setEvasionAlert({ active: false, obstacle: "" });
        }

        return {
          ...prev,
          battery: newBattery,
          position: { ...prev.position, y: newY, z: newZ },
        };
      });

      // If autonomous mission is active, slowly drive towards current waypoint target
      if (missionStatus.active && missionStatus.waypoints && missionStatus.waypoints.length > 0) {
        const currentWp = missionStatus.waypoints[missionStatus.currentWaypointIndex];
        if (currentWp && currentWp.position) {
          setRobotState((prev) => {
            const dx = currentWp.position.x - prev.position.x;
            const dy = currentWp.position.y - prev.position.y;
            const dz = currentWp.position.z - prev.position.z;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < 0.3 || isNaN(dist)) {
              // Reached waypoint!
              const nextIdx = (missionStatus.currentWaypointIndex + 1) % missionStatus.waypoints.length;
              const nextWp = missionStatus.waypoints[nextIdx];
              if (!nextWp) return prev;

              setMissionStatus((mPrev) => ({
                ...mPrev,
                currentWaypointIndex: nextIdx,
                logs: [
                  `[FSM] Reached Waypoint: ${currentWp.name}`,
                  `[FSM] Transitioning Kinematic Chain -> ${nextWp.targetMode}`,
                  ...(mPrev.logs || []).slice(0, 15),
                ],
              }));

              let newZ = nextWp.position?.z ?? 0.6;
              let rpm = prev.propellerRpm;
              let buoyancy = prev.buoyancyForce;

              if (nextWp.targetMode === "WALKING") {
                newZ = 0.6;
                rpm = 0;
                buoyancy = 0;
              } else if (nextWp.targetMode === "SAILING") {
                newZ = -0.05;
                rpm = 300;
                buoyancy = 41.2;
              } else if (nextWp.targetMode === "FLYING") {
                newZ = 2.0;
                rpm = 4200;
                buoyancy = 0;
              }

              return {
                ...prev,
                mode: nextWp.targetMode,
                position: { ...prev.position, z: newZ },
                propellerRpm: rpm,
                buoyancyForce: buoyancy,
                waterSubmerged: nextWp.targetMode === "SAILING",
                inAirPocket: nextWp.targetMode === "FLYING",
              };
            }

            const step = 0.12;
            const safeDist = dist > 0 ? dist : 1;
            return {
              ...prev,
              position: {
                x: prev.position.x + (dx / safeDist) * step,
                y: prev.position.y + (dy / safeDist) * step,
                z: prev.position.z + (dz / safeDist) * step,
              },
            };
          });
        }
      }
    }, 150);

    return () => clearInterval(interval);
  }, [missionStatus.active, missionStatus.currentWaypointIndex, missionStatus.waypoints]);

  // Reset Simulation
  const handleResetSim = () => {
    setRobotState({
      position: { x: -15, y: 0, z: 0.6 },
      orientation: { x: 0, y: 0, z: 0 },
      velocity: { x: 0, y: 0, z: 0 },
      mode: "WALKING",
      jointStates: {},
      propellerRpm: 0,
      headlightOn: headlight,
      battery: 100,
      buoyancyForce: 0,
      groundContact: true,
      waterSubmerged: false,
      waterDepth: 0,
      inAirPocket: false,
    });

    setMissionStatus((prev) => ({
      ...prev,
      active: false,
      currentWaypointIndex: 0,
      logs: ["[System] Simulation Pose Reset to Dry Cave Start (-15m, 0m, 0.6m).", ...prev.logs.slice(0, 15)],
    }));
  };

  const handleOpenAICopilot = (file?: any) => {
    setCopilotFile(file);
    setIsCopilotOpen(true);
  };

  return (
    <div className="min-h-screen w-full bg-slate-950 text-slate-100 flex flex-col font-sans">
      {/* Header Navigation */}
      <Navbar
        currentMode={robotState.mode}
        onResetSim={handleResetSim}
        onOpenAICopilot={() => handleOpenAICopilot()}
        activeView={activeView}
        setActiveView={setActiveView}
      />

      {/* Main Container */}
      <main className="flex-1 w-full max-w-7xl mx-auto p-4 flex flex-col gap-5">
        {activeView === "simulation" && (
          <>
            {/* Top 3D Gazebo Simulation Canvas */}
            <div className="w-full h-[460px]">
              <GazeboSimViewport
                robotState={robotState}
                setRobotState={setRobotState}
                sensorData={sensorData}
                setSensorData={setSensorData}
                activeCameraMode={activeCameraMode}
                setActiveCameraMode={setActiveCameraMode}
                showLaserBeams={showLaserBeams}
                setShowLaserBeams={setShowLaserBeams}
                showSonarPulse={showSonarPulse}
                setShowSonarPulse={setShowSonarPulse}
                headlight={headlight}
                setHeadlight={setHeadlight}
                antiCollisionEnabled={antiCollisionEnabled}
                setAntiCollisionEnabled={setAntiCollisionEnabled}
                evasionAlert={evasionAlert}
              />
            </div>

            {/* Middle: Multi-Modal FSM & Nav2 Controller */}
            <MultiModalNavPlanner
              robotState={robotState}
              setRobotState={setRobotState}
              missionStatus={missionStatus}
              setMissionStatus={setMissionStatus}
              sensorData={sensorData}
              setSensorData={setSensorData}
            />

            {/* Bottom: Multi-Sensor Perception Dashboard */}
            <SensorDashboard
              robotState={robotState}
              sensorData={sensorData}
              onSetBattery={(val) => setRobotState((prev) => ({ ...prev, battery: val }))}
            />
          </>
        )}

        {activeView === "urdf" && (
          <KinematicChainEditor robotState={robotState} setRobotState={setRobotState} />
        )}

        {activeView === "workspace" && (
          <ROS2WorkspaceExplorer onOpenAICopilot={handleOpenAICopilot} />
        )}
      </main>

      {/* Gemini AI Copilot Modal */}
      <AICopilotModal
        isOpen={isCopilotOpen}
        onClose={() => setIsCopilotOpen(false)}
        activeFile={copilotFile}
      />

      {/* Sleek Telemetry Footer Log Stream */}
      <footer className="h-10 bg-slate-950 border-t border-slate-800/80 px-4 flex items-center text-xs font-mono overflow-hidden whitespace-nowrap gap-4 shrink-0 mt-auto">
        <div className="text-emerald-400 font-bold shrink-0 flex items-center gap-1.5 text-[11px]">
          <span className="w-2 h-2 rounded-full bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,0.8)] animate-pulse"></span>
          [LOG]
        </div>
        <div className="text-slate-400 text-[11px] font-mono truncate flex-1">
          {(missionStatus?.logs || []).slice(0, 3).join("   •   ")}
        </div>
        <div className="hidden sm:flex items-center gap-3 text-[10px] text-slate-500 font-mono shrink-0">
          <span>Hz: 60.0</span>
          <span>Latency: 1.2ms</span>
          <span className="text-emerald-400">STATUS: OK</span>
        </div>
      </footer>
    </div>
  );
}
