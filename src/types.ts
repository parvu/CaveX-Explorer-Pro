/**
 * ROS 2 Jazzy Hybrid Drone Simulation Types
 */

export type CameraMode = "orbit" | "fpv" | "follow" | "topdown";

export type LocomotionMode = "WALKING" | "SAILING" | "FLYING";

export type EnvironmentSection = "DRY_CAVE" | "FLOODED_WATER" | "AIR_POCKET";

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

export interface JointState {
  name: string;
  angle: number; // in radians or degrees
  min: number;
  max: number;
  type: "revolute" | "continuous" | "prismatic" | "fixed";
  chain: "leg_fl" | "leg_fr" | "leg_bl" | "leg_br" | "hydrofoil" | "rotor";
}

export interface RobotState {
  position: Vector3D;
  orientation: Vector3D; // Roll, Pitch, Yaw in degrees
  velocity: Vector3D;
  mode: LocomotionMode;
  jointStates: Record<string, number>;
  propellerRpm: number;
  headlightOn: boolean;
  battery: number; // 0-100%
  buoyancyForce: number;
  groundContact: boolean;
  waterSubmerged: boolean;
  waterDepth: number; // depth below z=0
  inAirPocket: boolean;
}

export interface SensorData {
  cameraActive: boolean;
  cameraResolution: string;
  featurePointsCount: number;
  lidarRanges: number[]; // 360 degree range readings
  lidarMinDist: number;
  lidarMaxDist: number;
  sonarDepth: number; // Distance to submerged seabed
  sonarEchoStrength: number;
  imuAccel: Vector3D;
  imuGyro: Vector3D;
  slamPose: Vector3D;
  slamConfidence: number; // 0-100%
}

export interface Waypoint {
  id: string;
  name: string;
  targetMode: LocomotionMode;
  position: Vector3D;
  section: EnvironmentSection;
  completed: boolean;
  description: string;
}

export interface MissionStatus {
  active: boolean;
  currentWaypointIndex: number;
  waypoints: Waypoint[];
  autoTransitions: boolean;
  fsmState: string;
  logs: string[];
}

export interface ROS2File {
  path: string;
  name: string;
  language: "python" | "cpp" | "xml" | "yaml" | "cmake" | "markdown";
  content: string;
}

export interface OccupancyGridCell {
  x: number;
  y: number;
  state: "free" | "occupied" | "unknown";
}
