import React, { useState, useEffect } from "react";
import { RobotState, SensorData, MissionStatus, LocomotionMode, CameraMode } from "./types";
import { Navbar } from "./components/Navbar";
import { GazeboSimViewport } from "./components/GazeboSimViewport";
import { SensorDashboard } from "./components/SensorDashboard";
import { MultiModalNavPlanner } from "./components/MultiModalNavPlanner";
import { KinematicChainEditor } from "./components/KinematicChainEditor";
import { ROS2WorkspaceExplorer } from "./components/ROS2WorkspaceExplorer";
import { AICopilotModal } from "./components/AICopilotModal";

// Ground Collision Floor Height Calculator & Safety Clearance
export const getMinGroundHeight = (x: number, y: number, mode: LocomotionMode, currentZ?: number): number => {
  if (x < -5) {
    // Section 1: Dry Cave Corridor (Rock surface ~0.4m-0.6m; quadruped origin Z = 1.35m)
    if (mode === "WALKING") return 1.35;
    if (mode === "SAILING") return 0.85;
    return 1.35; // Flight clearance over rocks
  } else if (x <= 12) {
    // Section 2: Flooded Cave (Raised water surface at 0.6m matching dry section end, seabed at -2.9m)
    if (mode === "SAILING") return 0.55;  // Floating on raised water surface at 0.6m
    if (mode === "WALKING") return -2.15; // Quadruped walking on underwater seabed (-2.9m + 0.75m leg = -2.15m)
    return 1.40;                          // Quadrotor flight clearance above raised water surface
  } else {
    // Section 3: Air Pocket & Vertical Ascent Shaft (x > 12)
    // Extended 25m shaft chimney allows flight up to z = 24.0m
    if (mode === "WALKING") {
      // 16m high balcony platform top surface is at Z = 16.0m -> quadruped body Z = 16.75m
      if (x >= 17 && (currentZ === undefined || currentZ > 10.0)) {
        return 16.75;
      }
      return 1.35;   // Base ground clearance
    }
    if (mode === "SAILING") return 0.55;
    return 1.0;      // Flight clearance in open vertical shaft
  }
};

// Helper for dynamic environment mode detection
export const getAutoSectionMode = (x: number, y: number, z: number, currentMode: LocomotionMode): LocomotionMode => {
  if (x < -5) {
    // Section 1: Dry Cave -> WALKING default
    return currentMode === "FLYING" && z > 2.0 ? "FLYING" : "WALKING";
  } else if (x <= 12) {
    // Section 2: Flooded Water Channel -> SAILING default (Raised water surface z = 0.6m)
    if (z < -1.0) return "WALKING"; // Underwater seabed walk
    if (z > 1.8) return "FLYING";   // Aerial flight above water surface
    return "SAILING";               // Water surface hydrofoil sailing
  } else {
    // Section 3: Air Pocket & Vertical Shaft -> FLYING default for VTOL ascent up to 25m shaft ceiling
    if (x >= 17 && z >= 15.0 && currentMode === "WALKING") {
      return "WALKING"; // Standing & walking on top of 16m balcony platform
    }
    return "FLYING"; // Ascending vertical tube chimney
  }
};

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
    position: { x: -15, y: 0, z: 1.35 },
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
    slamPose: { x: -15, y: 0, z: 1.35 },
    slamConfidence: 96,
  });

  // Autonomous Mission Status
  const [missionStatus, setMissionStatus] = useState<MissionStatus>({
    active: false,
    currentWaypointIndex: 0,
    waypoints: [
      {
        id: "wp1",
        name: "Dry Cave Patrol Origin",
        targetMode: "WALKING",
        position: { x: -15, y: 0, z: 1.35 },
        section: "DRY_CAVE",
        completed: true,
        description: "Rugged stalagmite quadruped ground start (-15m)",
      },
      {
        id: "wp2",
        name: "Rugged Dry Corridor Mid-Point",
        targetMode: "WALKING",
        position: { x: -10, y: 0.3, z: 1.35 },
        section: "DRY_CAVE",
        completed: false,
        description: "Navigating through narrow dry stalagmite passage",
      },
      {
        id: "wp3",
        name: "Water Edge Transition Readiness",
        targetMode: "WALKING",
        position: { x: -6, y: 0, z: 1.35 },
        section: "DRY_CAVE",
        completed: false,
        description: "Approaching dry section end / flooded water shore",
      },
      {
        id: "wp4",
        name: "Water Launch & Pontoon Deployment",
        targetMode: "SAILING",
        position: { x: -4, y: 0, z: 0.55 },
        section: "FLOODED_WATER",
        completed: false,
        description: "Hydrofoil launch matching raised water surface (z=0.6m)",
      },
      {
        id: "wp5",
        name: "Flooded Lake Cruise West",
        targetMode: "SAILING",
        position: { x: 0, y: -0.4, z: 0.55 },
        section: "FLOODED_WATER",
        completed: false,
        description: "Surface sailing on raised flooded channel (z=0.6m)",
      },
      {
        id: "wp6",
        name: "Submerged Rock Passage Center",
        targetMode: "SAILING",
        position: { x: 4, y: 0.3, z: 0.55 },
        section: "FLOODED_WATER",
        completed: false,
        description: "Hydrofoil surface navigation over submerged rock formations",
      },
      {
        id: "wp7",
        name: "Flooded Lake East Shore Approach",
        targetMode: "SAILING",
        position: { x: 8, y: 0, z: 0.55 },
        section: "FLOODED_WATER",
        completed: false,
        description: "Approaching east shore transition of flooded channel",
      },
      {
        id: "wp8",
        name: "Air Pocket Beach Transition",
        targetMode: "SAILING",
        position: { x: 11, y: 0, z: 0.8 },
        section: "AIR_POCKET",
        completed: false,
        description: "Entering air pocket cave zone at base of vertical shaft",
      },
      {
        id: "wp9",
        name: "Shaft Base Portal Approach",
        targetMode: "FLYING",
        position: { x: 13.2, y: 0, z: 2.0 },
        section: "AIR_POCKET",
        completed: false,
        description: "VTOL transition into wide open 10.4m diameter shaft portal",
      },
      {
        id: "wp10",
        name: "VTOL Takeoff & Shaft Tube Entry",
        targetMode: "FLYING",
        position: { x: 15.5, y: 0, z: 3.8 },
        section: "AIR_POCKET",
        completed: false,
        description: "Ascending into vertical tube chimney (z=3.8m)",
      },
      {
        id: "wp11",
        name: "Lower Shaft Spire Evasion (North)",
        targetMode: "FLYING",
        position: { x: 17.5, y: 0.8, z: 6.2 },
        section: "AIR_POCKET",
        completed: false,
        description: "Bypassing compact lower rock spire with safe clearance (z=6.2m)",
      },
      {
        id: "wp12",
        name: "Mid-Shaft Arch Pass (South)",
        targetMode: "FLYING",
        position: { x: 18.8, y: -0.8, z: 9.8 },
        section: "AIR_POCKET",
        completed: false,
        description: "Navigating open central flight channel past mid arch (z=9.8m)",
      },
      {
        id: "wp13",
        name: "Upper Shaft Stalactite Clearance",
        targetMode: "FLYING",
        position: { x: 17.6, y: 0.6, z: 13.5 },
        section: "AIR_POCKET",
        completed: false,
        description: "Smooth evasion of upper compact stalactite overhang (z=13.5m)",
      },
      {
        id: "wp14",
        name: "Upper Shaft Rock Shelf Bypass",
        targetMode: "FLYING",
        position: { x: 18.8, y: -0.5, z: 16.5 },
        section: "AIR_POCKET",
        completed: false,
        description: "Ascending past upper rock shelf obstacle (z=16.5m)",
      },
      {
        id: "wp15",
        name: "Shaft High Ceiling Apex Sweep",
        targetMode: "FLYING",
        position: { x: 18.5, y: 0, z: 21.0 },
        section: "AIR_POCKET",
        completed: false,
        description: "High altitude exploration near 25m shaft ceiling (z=21.0m)",
      },
      {
        id: "wp16",
        name: "16m Balcony Landing Descent Approach",
        targetMode: "FLYING",
        position: { x: 18.8, y: 0, z: 17.5 },
        section: "AIR_POCKET",
        completed: false,
        description: "Aligning VTOL descent vector over 16m high balcony platform",
      },
      {
        id: "wp17",
        name: "16m Balcony Touchdown & Leg Lock",
        targetMode: "WALKING",
        position: { x: 19.2, y: 0, z: 16.75 },
        section: "AIR_POCKET",
        completed: false,
        description: "Landing and walking on 16m high balcony summit platform (z=16.75m)",
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
          } else if (prev.mode === "FLYING" && prev.position.x < 12 && newZ > 4.5) {
            newZ -= 0.15; // Auto evasion nudge down in low ceiling cave
            setEvasionAlert({ active: true, obstacle: "Cave Ceiling Obstacle (0.8m)" });
          } else {
            setEvasionAlert({ active: false, obstacle: "" });
          }
        } else {
          setEvasionAlert({ active: false, obstacle: "" });
        }

        // Ground Collision Anti-penetration floor check
        const minZFloor = getMinGroundHeight(prev.position.x, newY, prev.mode, newZ);
        if (newZ < minZFloor) {
          newZ = minZFloor;
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

              let newZ = nextWp.position?.z ?? 1.35;
              let rpm = prev.propellerRpm;
              let buoyancy = prev.buoyancyForce;

              if (nextWp.targetMode === "WALKING") {
                newZ = nextWp.position?.z ?? 1.35;
                rpm = 0;
                buoyancy = 0;
              } else if (nextWp.targetMode === "SAILING") {
                newZ = 0.55;
                rpm = 300;
                buoyancy = 41.2;
              } else if (nextWp.targetMode === "FLYING") {
                newZ = nextWp.position?.z ?? 2.0;
                rpm = 4200;
                buoyancy = 0;
              }

              const safeMinZ = getMinGroundHeight(nextWp.position.x, nextWp.position.y, nextWp.targetMode, newZ);
              if (newZ < safeMinZ) {
                newZ = safeMinZ;
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

            const step = 0.15;
            const safeDist = dist > 0 ? dist : 1;
            const nextX = prev.position.x + (dx / safeDist) * step;
            const nextY = prev.position.y + (dy / safeDist) * step;
            let nextZ = prev.position.z + (dz / safeDist) * step;

            const dynamicMode = getAutoSectionMode(nextX, nextY, nextZ, prev.mode);
            const floorZ = getMinGroundHeight(nextX, nextY, dynamicMode);
            if (nextZ < floorZ) {
              nextZ = floorZ;
            }

            return {
              ...prev,
              mode: dynamicMode,
              position: {
                x: nextX,
                y: nextY,
                z: nextZ,
              },
              propellerRpm: dynamicMode === "FLYING" ? 4200 : dynamicMode === "SAILING" ? 300 : 0,
              waterSubmerged: dynamicMode === "SAILING",
              inAirPocket: dynamicMode === "FLYING",
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
