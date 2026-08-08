import React, { useState, useEffect } from "react";
import { RobotState, SensorData, MissionStatus, LocomotionMode, CameraMode } from "./types";
import { Navbar } from "./components/Navbar";
import { GazeboSimViewport } from "./components/GazeboSimViewport";
import { SensorDashboard } from "./components/SensorDashboard";
import { MultiModalNavPlanner } from "./components/MultiModalNavPlanner";
import { KinematicChainEditor } from "./components/KinematicChainEditor";
import { ROS2WorkspaceExplorer } from "./components/ROS2WorkspaceExplorer";
import { SICSlamVisualizer } from "./components/SICSlamVisualizer";
import { TrackedVehiclePanel } from "./components/TrackedVehiclePanel";
import { AICopilotModal } from "./components/AICopilotModal";

// Ground Collision Floor Height Calculator & Safety Clearance
export const getMinGroundHeight = (x: number, y: number, mode: LocomotionMode, currentZ?: number): number => {
  if (x < -5) {
    // Section 1: Dry Cave Corridor (Rock surface ~0.4m-0.6m; quadruped origin Z = 1.35m)
    if (mode === "WALKING") return 1.35;
    if (mode === "SAILING") return 0.85;
    return 1.35; // Flight clearance over rocks
  } else if (x <= 29) {
    // Section 2: Flooded Cave (Raised water surface at 0.6m matching dry section end, seabed at -2.9m)
    if (mode === "SAILING") return 0.55;  // Floating on raised water surface at 0.6m
    if (mode === "WALKING") return -2.15; // Quadruped walking on underwater seabed (-2.9m + 0.75m leg = -2.15m)
    return 1.40;                          // Quadrotor flight clearance above raised water surface
  } else {
    // Section 3: Air Pocket & Vertical Ascent Shaft (x > 29)
    // Extended 25m shaft chimney allows flight up to z = 24.0m
    if (mode === "WALKING") return 1.35;   // Base ground clearance for Spot companion computer base
    if (mode === "SAILING") return 0.55;
    return 1.0;      // Flight clearance in open vertical shaft
  }
};

// Helper for dynamic environment mode detection
export const getAutoSectionMode = (x: number, y: number, z: number, currentMode: LocomotionMode): LocomotionMode => {
  if (x < -5) {
    // Section 1: Dry Cave -> WALKING default
    return currentMode === "FLYING" && z > 2.0 ? "FLYING" : "WALKING";
  } else if (x <= 29) {
    // Section 2: Flooded Water Channel -> SAILING default (Raised water surface z = 0.6m)
    if (z < -1.0) return "WALKING"; // Underwater seabed walk
    if (z > 1.8) return "FLYING";   // Aerial flight above water surface
    return "SAILING";               // Water surface hydrofoil sailing
  } else {
    // Section 3: Air Pocket & Vertical Shaft -> FLYING default for VTOL ascent up to 25m shaft ceiling
    return "FLYING"; // Ascending vertical tube chimney for full 3D mapping
  }
};

export default function App() {
  const [activeView, setActiveView] = useState<"simulation" | "workspace" | "urdf" | "sicslam">("simulation");
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
    position: { x: -34.5, y: 0, z: 1.35 },
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
    wifiStreamingActive: true,
    wifiSignalDbm: -42,
    wifiBitrateMbps: 48.5,
    wifiLatencyMs: 12,
    lidarRanges: Array.from({ length: 36 }, () => 3 + Math.random() * 2.5),
    lidarMinDist: 0.2,
    lidarMaxDist: 12.0,
    sonarActive: true,
    sonarDepth: 3.5,
    sonarEchoStrength: 88,
    imuAccel: { x: 0, y: 0, z: 9.81 },
    imuGyro: { x: 0, y: 0, z: 0 },
    slamPose: { x: -17.5, y: 0, z: 1.35 },
    slamConfidence: 96,
  });

  // Autonomous Mission Status
  const [missionStatus, setMissionStatus] = useState<MissionStatus>({
    active: false,
    currentWaypointIndex: 0,
    waypoints: [
      {
        id: "wp1",
        name: "Dry Cave Entrance",
        targetMode: "WALKING",
        position: { x: -37.0, y: 0, z: 1.35 },
        section: "DRY_CAVE",
        completed: true,
        description: "Starting position in dry cave (-37m)",
      },
      {
        id: "wp2",
        name: "Dry Cave Mid-Point",
        targetMode: "WALKING",
        position: { x: -22.0, y: 1.0, z: 1.35 },
        section: "DRY_CAVE",
        completed: false,
        description: "Navigating stalagmites",
      },
      {
        id: "wp3",
        name: "Approaching Water Edge",
        targetMode: "WALKING",
        position: { x: -7.0, y: -1.0, z: 1.35 },
        section: "DRY_CAVE",
        completed: false,
        description: "Reaching the end of the dry floor",
      },
      {
        id: "wp4",
        name: "Water Entry & Sailing Mode",
        targetMode: "SAILING",
        position: { x: -3.0, y: 0.0, z: 0.6 },
        section: "FLOODED_WATER",
        completed: false,
        description: "Transition to sailing on the flooded cave water",
      },
      {
        id: "wp5",
        name: "Flooded Cave Navigation",
        targetMode: "SAILING",
        position: { x: 12.0, y: 1.5, z: 0.6 },
        section: "FLOODED_WATER",
        completed: false,
        description: "Avoiding submerged rocks",
      },
      {
        id: "wp6",
        name: "Approaching East Shore",
        targetMode: "SAILING",
        position: { x: 27.0, y: -1.0, z: 0.6 },
        section: "FLOODED_WATER",
        completed: false,
        description: "Reaching the base of the air pocket",
      },
      {
        id: "wp7",
        name: "Shore Arrival & Takeoff Prep",
        targetMode: "WALKING",
        position: { x: 30.0, y: 0.0, z: 1.35 },
        section: "AIR_POCKET",
        completed: false,
        description: "Transitioning back to walking on shore",
      },
      {
        id: "wp8",
        name: "VTOL Takeoff",
        targetMode: "FLYING",
        position: { x: 30.0, y: 0.0, z: 4.0 },
        section: "AIR_POCKET",
        completed: false,
        description: "Initiating vertical takeoff in the air pocket",
      },
      {
        id: "wp9",
        name: "Shaft Entry & Stabilization",
        targetMode: "FLYING",
        position: { x: 32.5, y: 0.0, z: 6.0 },
        section: "AIR_POCKET",
        completed: false,
        description: "Ascending into the lower shaft",
      },
      {
        id: "wp10",
        name: "Helical Mapping - Lower East",
        targetMode: "FLYING",
        position: { x: 34.5, y: 2.5, z: 8.5 },
        section: "AIR_POCKET",
        completed: false,
        description: "Scanning lower east wall",
      },
      {
        id: "wp11",
        name: "Helical Mapping - Mid North",
        targetMode: "FLYING",
        position: { x: 36.5, y: 0.0, z: 11.5 },
        section: "AIR_POCKET",
        completed: false,
        description: "Navigating past mid-shaft outcrops",
      },
      {
        id: "wp12",
        name: "Helical Mapping - Upper West",
        targetMode: "FLYING",
        position: { x: 34.5, y: -3.0, z: 15.0 },
        section: "AIR_POCKET",
        completed: false,
        description: "Scanning upper stalactites",
      },
      {
        id: "wp13",
        name: "Helical Mapping - Upper South",
        targetMode: "FLYING",
        position: { x: 33.5, y: 0.0, z: 18.5 },
        section: "AIR_POCKET",
        completed: false,
        description: "Approaching apex dome",
      },
      {
        id: "wp14",
        name: "Shaft Apex Dome 360 Scan",
        targetMode: "FLYING",
        position: { x: 35.5, y: 0.0, z: 23.0 },
        section: "AIR_POCKET",
        completed: false,
        description: "Full resolution ceiling mapping",
      },
      {
        id: "wp15",
        name: "Vertical Descent - Center",
        targetMode: "FLYING",
        position: { x: 35.0, y: 0.0, z: 12.0 },
        section: "AIR_POCKET",
        completed: false,
        description: "Returning down the shaft center",
      },
      {
        id: "wp16",
        name: "Landing Approach",
        targetMode: "FLYING",
        position: { x: 31.0, y: 0.0, z: 4.0 },
        section: "AIR_POCKET",
        completed: false,
        description: "Aligning for touchdown",
      },
      {
        id: "wp17",
        name: "Spot Base Docking",
        targetMode: "FLYING",
        position: { x: 30.0, y: 0.0, z: 1.5 },
        section: "AIR_POCKET",
        completed: false,
        description: "Precision landing on Spot cradle",
      }
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

  // Live ROS2 telemetry (see web_telemetry_bridge.py -> /api/telemetry).
  // Real robot pose (ground_truth) and real lidar scan (lidar_ranges) are
  // used to drive the 3D view + sensor panels below in place of the
  // client-fabricated physics when the ROS2 stack is actually running.
  // Falls back to the mock simulation (unchanged) when it isn't -- same
  // honest live/demo split already used in SICSlamVisualizer.tsx.
  const [liveTelemetry, setLiveTelemetry] = useState<{
    ground_truth: { x: number; y: number; z: number; yaw: number } | null;
    sic_slam_pose: { x: number; y: number; z: number; yaw: number } | null;
    ate_rmse: number | null;
    lidar_ranges: number[] | null;
    nav_goal: { x: number; y: number; distance_remaining: number } | null;
  } | null>(null);
  const isLive = liveTelemetry !== null && liveTelemetry.ground_truth !== null;

  useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      try {
        const res = await fetch("/api/telemetry");
        const json = await res.json();
        if (!cancelled) setLiveTelemetry(json.live ? json.data : null);
      } catch {
        if (!cancelled) setLiveTelemetry(null);
      }
    };
    poll();
    const id = setInterval(poll, 500);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, []);

  useEffect(() => {
    if (!liveTelemetry?.ground_truth) return;
    const gt = liveTelemetry.ground_truth;
    setRobotState((prev) => ({
      ...prev,
      position: { x: gt.x, y: gt.y, z: prev.position.z },
      orientation: { ...prev.orientation, z: (gt.yaw * 180) / Math.PI },
    }));
    if (liveTelemetry.lidar_ranges) {
      setSensorData((prev) => ({ ...prev, lidarRanges: liveTelemetry.lidar_ranges! }));
    }
  }, [liveTelemetry]);

  // Real (if minimal -- straight-line, no obstacle avoidance) waypoint
  // navigation: POSTs a ground (x, y) goal for waypoint_follower.py to
  // drive to. There is no flight/dive capability in this sim's actual
  // robot, so z/mode from the mission's waypoint list is not honored here
  // -- only the ground position is real.
  const sendWaypointGoal = async (x: number, y: number) => {
    try {
      await fetch("/api/waypoint-goal", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ x, y }),
      });
    } catch {
      // best-effort -- the ROS2 bridge just won't see a new goal this tick
    }
  };

  // Periodic sensor updates & autonomous mission loop
  useEffect(() => {
    const interval = setInterval(() => {
      // Dynamic sensor readings jitter & subtle battery consumption
      setSensorData((prev) => ({
        ...prev,
        featurePointsCount: 35 + Math.floor(Math.random() * 15),
        // Real lidar_ranges is set from telemetry above when live -- don't
        // jitter over it here, that would fight the real data every tick.
        lidarRanges: isLive
          ? prev.lidarRanges
          : prev.lidarRanges.map((r) => Math.max(0.5, r + (Math.random() - 0.5) * 0.15)),
        sonarEchoStrength: Math.max(40, Math.min(100, prev.sonarEchoStrength + (Math.random() - 0.5) * 4)),
        slamConfidence: Math.max(90, Math.min(99, prev.slamConfidence + (Math.random() - 0.5) * 1)),
      }));

      // Dynamic battery drain & Anti-Collision Avoidance Guard -- position
      // comes from real telemetry when live (see the effect above), so
      // this mock physics step is skipped then (it would otherwise fight
      // the real pose every 150ms).
      if (isLive) return;
      setRobotState((prev) => {
        const drainRate = prev.mode === "FLYING" ? 0.08 : prev.mode === "SAILING" ? 0.03 : 0.02;
        const newBattery = Math.max(0, prev.battery - drainRate);
        let newX = prev.position.x;
        let newY = prev.position.y;
        let newZ = prev.position.z;

        if (antiCollisionEnabled) {
          // Detect Y wall bounds [-3.0, 3.0] inside the narrow sections
          if (newX < 30) {
            if (newY > 2.6) {
              newY -= 0.15;
              setEvasionAlert({ active: true, obstacle: "Right Cave Wall Proximity (0.4m)" });
            } else if (newY < -2.6) {
              newY += 0.15;
              setEvasionAlert({ active: true, obstacle: "Left Cave Wall Proximity (0.4m)" });
            } else if (prev.mode === "FLYING" && newX < 12 && newZ > 4.5) {
              newZ -= 0.15;
              setEvasionAlert({ active: true, obstacle: "Cave Ceiling Obstacle (0.8m)" });
            } else {
              setEvasionAlert({ active: false, obstacle: "" });
            }
          }
          
          // Shaft Obstacles Spherical Evasion
          if (newX > 30) {
            const obstacles = [
              { id: "O1", x: 33.5, y: -4.2, z: 5.5, radius: 1.5, name: "Lower Shaft Spire" },
              { id: "O2", x: 37.2, y: 4.2, z: 9.2, radius: 1.5, name: "Mid-Shaft Rock Outcrop" },
              { id: "O3", x: 34.2, y: -4.2, z: 14.5, radius: 1.5, name: "Upper Shaft Stalactite" },
              { id: "O4", x: 36.8, y: 4.2, z: 16.8, radius: 1.5, name: "Upper Shaft Rock Shelf" },
            ];
            
            let evaded = false;
            for (const obs of obstacles) {
              const dx = newX - obs.x;
              const dy = newY - obs.y;
              const dz = newZ - obs.z;
              const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
              
              if (dist < obs.radius) {
                // Avoidance push
                const pushStr = (obs.radius - dist) + 0.1;
                newX += (dx / dist) * pushStr;
                newY += (dy / dist) * pushStr;
                newZ += (dz / dist) * pushStr;
                setEvasionAlert({ active: true, obstacle: `Collision Avoidance: ${obs.name}` });
                evaded = true;
                break;
              }
            }
            if (!evaded) {
              setEvasionAlert({ active: false, obstacle: "" });
            }
          }
        } else {
          setEvasionAlert({ active: false, obstacle: "" });
        }

        // Ground Collision Anti-penetration floor check
        const minZFloor = getMinGroundHeight(newX, newY, prev.mode, newZ);
        if (newZ < minZFloor) {
          newZ = minZFloor;
        }

        return {
          ...prev,
          battery: newBattery,
          position: { ...prev.position, x: newX, y: newY, z: newZ },
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

              let rpm = prev.propellerRpm;
              let buoyancy = prev.buoyancyForce;

              if (nextWp.targetMode === "WALKING") {
                rpm = 0;
                buoyancy = 0;
              } else if (nextWp.targetMode === "SAILING") {
                rpm = 300;
                buoyancy = 41.2;
              } else if (nextWp.targetMode === "FLYING") {
                rpm = 4200;
                buoyancy = 0;
              }

              return {
                ...prev,
                mode: nextWp.targetMode,
                propellerRpm: rpm,
                buoyancyForce: buoyancy,
                waterSubmerged: nextWp.targetMode === "SAILING",
                inAirPocket: nextWp.targetMode === "FLYING",
              };
            }

            const safeDist = dist > 0 ? dist : 1;
            let nextX = prev.position.x;
            let nextY = prev.position.y;
            let nextZ = prev.position.z;
            
            if (prev.mode === "FLYING") {
              // Smooth flight path with subtle hover noise
              const time = Date.now() / 1000;
              const hoverX = Math.sin(time * 1.5) * 0.03;
              const hoverY = Math.cos(time * 1.2) * 0.03;
              const hoverZ = Math.sin(time * 2.0) * 0.02;
              
              const speed = 0.25;
              nextX += (dx / safeDist) * speed + hoverX;
              nextY += (dy / safeDist) * speed + hoverY;
              nextZ += (dz / safeDist) * speed + hoverZ;
            } else {
              // Walking / Sailing mode linear step
              const step = 0.15;
              nextX += (dx / safeDist) * step;
              nextY += (dy / safeDist) * step;
              nextZ += (dz / safeDist) * step;
            }

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
  }, [missionStatus.active, missionStatus.currentWaypointIndex, missionStatus.waypoints, isLive]);

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
                isLive={isLive}
                liveTelemetry={liveTelemetry}
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
              isLive={isLive}
              sendWaypointGoal={sendWaypointGoal}
            />

            {/* Bottom: Multi-Sensor Perception Dashboard */}
            <SensorDashboard
              robotState={robotState}
              sensorData={sensorData}
              onSetBattery={(val) => setRobotState((prev) => ({ ...prev, battery: val }))}
              onToggleSonar={() => {
                const nextVal = !showSonarPulse;
                setShowSonarPulse(nextVal);
                setSensorData((prev) => ({ ...prev, sonarActive: nextVal }));
              }}
              onToggleWifi={() => {
                setSensorData((prev) => ({
                  ...prev,
                  wifiStreamingActive: !prev.wifiStreamingActive,
                }));
              }}
            />

            {/* Tracked BlueBoat-like vehicle live status (Tasks 5/10-13) */}
            <TrackedVehiclePanel />
          </>
        )}

        {activeView === "urdf" && (
          <KinematicChainEditor robotState={robotState} setRobotState={setRobotState} />
        )}

        {activeView === "sicslam" && (
          <SICSlamVisualizer robotState={robotState} sensorData={sensorData} />
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
