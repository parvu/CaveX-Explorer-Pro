import React, { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { RobotState, LocomotionMode, EnvironmentSection, SensorData, CameraMode } from "../types";
import { getMinGroundHeight } from "../App";
import { Camera, Eye, Layers, Maximize2, RotateCcw, Zap, Compass, Navigation, Video, Circle, Sun, ShieldCheck, ShieldAlert, Folder, Download, Radio, BatteryCharging, BatteryWarning, Waves, AlertTriangle, Edit3, Plus, Trash2, Box, Move, Sliders, EyeOff, Sparkles, Check, Crosshair, Target, Lightbulb, MapPin, Copy, Settings, X, GripVertical, ArrowUp, ArrowDown, ArrowLeft, ArrowRight } from "lucide-react";

interface GazeboSimViewportProps {
  robotState: RobotState;
  setRobotState: React.Dispatch<React.SetStateAction<RobotState>>;
  sensorData: SensorData;
  setSensorData: React.Dispatch<React.SetStateAction<SensorData>>;
  activeCameraMode: CameraMode;
  setActiveCameraMode: (mode: CameraMode) => void;
  showLaserBeams: boolean;
  setShowLaserBeams: (val: boolean) => void;
  showSonarPulse: boolean;
  setShowSonarPulse: (val: boolean) => void;
  headlight: boolean;
  setHeadlight: (val: boolean) => void;
  antiCollisionEnabled?: boolean;
  setAntiCollisionEnabled?: (val: boolean) => void;
  evasionAlert?: { active: boolean; obstacle: string };
}

export interface EditableSimObject {
  id: string;
  name: string;
  type: "boulder" | "stalagmite" | "buoy" | "target" | "spotlight" | "waypoint";
  position: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number };
  scale: { x: number; y: number; z: number };
  color: string;
  intensity?: number;
  visible: boolean;
}

export const GazeboSimViewport: React.FC<GazeboSimViewportProps> = ({
  robotState,
  setRobotState,
  sensorData,
  setSensorData,
  activeCameraMode,
  setActiveCameraMode,
  showLaserBeams,
  setShowLaserBeams,
  showSonarPulse,
  setShowSonarPulse,
  headlight,
  setHeadlight,
  antiCollisionEnabled = true,
  setAntiCollisionEnabled,
  evasionAlert = { active: false, obstacle: "" },
}) => {
  const mountRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);

  // 3D Objects references
  const robotGroupRef = useRef<THREE.Group | null>(null);
  const spotGroupRef = useRef<THREE.Group | null>(null);
  const droneGroupRef = useRef<THREE.Group | null>(null);
  const legsRef = useRef<THREE.Group[]>([]);
  const pontoonsRef = useRef<THREE.Group[]>([]);
  const propellersRef = useRef<THREE.Mesh[]>([]);
  const lidarMeshRef = useRef<THREE.Group | null>(null);
  const laserRaysRef = useRef<THREE.LineSegments | null>(null);
  const sonarPulseMeshRef = useRef<THREE.Mesh | null>(null);
  const waterMeshRef = useRef<THREE.Mesh | null>(null);
  const slamVoxelMeshRef = useRef<THREE.InstancedMesh | null>(null);
  const exploredVoxelsRef = useRef<Set<string>>(new Set());
  const voxelCountRef = useRef<number>(0);
  const tempMatrix = new THREE.Matrix4();
  const spotlightRef = useRef<THREE.SpotLight | null>(null);
  const ambientLightRef = useRef<THREE.AmbientLight | null>(null);
  const dirLightRef = useRef<THREE.DirectionalLight | null>(null);
  const shieldMeshRef = useRef<THREE.Mesh | null>(null);

  const isMouseDragging = useRef(false);
  const previousMousePos = useRef({ x: 0, y: 0 });
  const cameraAngle = useRef({ theta: Math.PI / 4, phi: Math.PI / 5, radius: 14 });

  // State refs to keep animation loop up to date without destroying WebGL context on prop changes
  const activeCameraModeRef = useRef(activeCameraMode);
  activeCameraModeRef.current = activeCameraMode;

  const headlightRef = useRef(headlight);
  headlightRef.current = headlight;

  const showLaserBeamsRef = useRef(showLaserBeams);
  showLaserBeamsRef.current = showLaserBeams;

  const showSonarPulseRef = useRef(showSonarPulse);
  showSonarPulseRef.current = showSonarPulse;

  const robotStateRef = useRef(robotState);
  robotStateRef.current = robotState;

  const antiCollisionEnabledRef = useRef(antiCollisionEnabled);
  antiCollisionEnabledRef.current = antiCollisionEnabled;

  const evasionAlertRef = useRef(evasionAlert);
  evasionAlertRef.current = evasionAlert;

  // Gazebo Viewport 3D World Scene Editor State
  const [isEditorMode, setIsEditorMode] = useState(false);
  const [editorTab, setEditorTab] = useState<"objects" | "spawn" | "environment">("objects");
  const [spawnedObjects, setSpawnedObjects] = useState<EditableSimObject[]>([
    {
      id: "obj-initial-boulder-1",
      name: "Subsea Rock Ridge A",
      type: "boulder",
      position: { x: 5, y: -1.2, z: -1.5 },
      rotation: { x: 0, y: 45, z: 0 },
      scale: { x: 1.2, y: 1.0, z: 1.2 },
      color: "#475569",
      visible: true,
    },
    {
      id: "obj-initial-buoy-1",
      name: "Sonar Acoustic Buoy 1",
      type: "buoy",
      position: { x: 12, y: 0.6, z: 2.0 },
      rotation: { x: 0, y: 0, z: 0 },
      scale: { x: 1.0, y: 1.0, z: 1.0 },
      color: "#f97316",
      intensity: 3.0,
      visible: true,
    },
    {
      id: "obj-initial-target-1",
      name: "ARUCO Calibration Grid",
      type: "target",
      position: { x: 20, y: -0.5, z: -2.0 },
      rotation: { x: 0, y: 90, z: 0 },
      scale: { x: 1.0, y: 1.0, z: 1.0 },
      color: "#06b6d4",
      visible: true,
    },
  ]);

  const [selectedObjectId, setSelectedObjectId] = useState<string | null>("obj-initial-boulder-1");
  const [clickToPlaceType, setClickToPlaceType] = useState<EditableSimObject["type"] | null>(null);

  // Environment & Physics Parameters
  const [envLightIntensity, setEnvLightIntensity] = useState<number>(3.5);
  const [waterFogDensity, setWaterFogDensity] = useState<number>(0.04);
  const [waterColor, setWaterColor] = useState<string>("#0f2b48");
  const [waterWaveHeight, setWaterWaveHeight] = useState<number>(0.4);
  const [antiCollisionRadius, setAntiCollisionRadius] = useState<number>(2.2);

  // Editor Refs
  const editorObjectsGroupRef = useRef<THREE.Group | null>(null);
  const clickToPlaceTypeRef = useRef(clickToPlaceType);
  clickToPlaceTypeRef.current = clickToPlaceType;

  const spawnedObjectsRef = useRef(spawnedObjects);
  spawnedObjectsRef.current = spawnedObjects;

  const selectedObjectIdRef = useRef(selectedObjectId);
  selectedObjectIdRef.current = selectedObjectId;

  // Helper Functions for Object Spawning, Transforming, and Editing
  const spawnNewObject = (
    type: EditableSimObject["type"],
    customPos?: { x: number; y: number; z: number }
  ) => {
    const rx = robotStateRef.current.position.x;
    const ry = robotStateRef.current.position.y;
    const rz = robotStateRef.current.position.z;

    const spawnX = customPos ? customPos.x : Number((rx + 2.5).toFixed(2));
    const spawnY = customPos ? customPos.y : Number((ry).toFixed(2));
    const spawnZ = customPos ? customPos.z : Number((rz + (Math.random() * 2 - 1)).toFixed(2));

    const typeNames: Record<EditableSimObject["type"], string> = {
      boulder: "Heavy Boulder",
      stalagmite: "Cave Column",
      buoy: "Beacon Buoy",
      target: "Inspection Target",
      spotlight: "Lighting Beacon",
      waypoint: "Subsea Waypoint",
    };

    const defaultColors: Record<EditableSimObject["type"], string> = {
      boulder: "#475569",
      stalagmite: "#334155",
      buoy: "#f97316",
      target: "#06b6d4",
      spotlight: "#fbbf24",
      waypoint: "#10b981",
    };

    const newObj: EditableSimObject = {
      id: `obj-${Date.now()}-${Math.floor(Math.random() * 1000)}`,
      name: `${typeNames[type]} ${spawnedObjects.length + 1}`,
      type,
      position: { x: spawnX, y: spawnY, z: spawnZ },
      rotation: { x: 0, y: Math.floor(Math.random() * 360), z: 0 },
      scale: { x: 1, y: 1, z: 1 },
      color: defaultColors[type],
      intensity: type === "spotlight" ? 4.0 : type === "buoy" ? 2.5 : 1.0,
      visible: true,
    };

    setSpawnedObjects((prev) => [...prev, newObj]);
    setSelectedObjectId(newObj.id);
  };

  const updateSelectedObject = (updater: (obj: EditableSimObject) => EditableSimObject) => {
    if (!selectedObjectId) return;
    setSpawnedObjects((prev) =>
      prev.map((o) => (o.id === selectedObjectId ? updater(o) : o))
    );
  };

  const deleteObject = (id: string) => {
    setSpawnedObjects((prev) => prev.filter((o) => o.id !== id));
    if (selectedObjectId === id) {
      const remaining = spawnedObjects.filter((o) => o.id !== id);
      setSelectedObjectId(remaining.length > 0 ? remaining[remaining.length - 1].id : null);
    }
  };

  const duplicateObject = (id: string) => {
    const target = spawnedObjects.find((o) => o.id === id);
    if (!target) return;
    const clone: EditableSimObject = {
      ...target,
      id: `obj-${Date.now()}-${Math.floor(Math.random() * 1000)}`,
      name: `${target.name} (Copy)`,
      position: {
        x: Number((target.position.x + 0.8).toFixed(2)),
        y: target.position.y,
        z: Number((target.position.z + 0.8).toFixed(2)),
      },
    };
    setSpawnedObjects((prev) => [...prev, clone]);
    setSelectedObjectId(clone.id);
  };

  const snapToRobot = (id: string) => {
    const rx = Number((robotStateRef.current.position.x + 2.0).toFixed(2));
    const ry = Number((robotStateRef.current.position.y).toFixed(2));
    const rz = Number((robotStateRef.current.position.z).toFixed(2));
    setSpawnedObjects((prev) =>
      prev.map((o) => (o.id === id ? { ...o, position: { x: rx, y: ry, z: rz } } : o))
    );
  };

  // Video Recording State & Custom Download Directory
  const [isRecording, setIsRecording] = useState(false);
  const [recordSeconds, setRecordSeconds] = useState(0);
  const [hasWebGLError, setHasWebGLError] = useState(false);
  const [downloadDir, setDownloadDir] = useState("~/Downloads/CaveX_Recordings/");
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const recordedChunksRef = useRef<Blob[]>([]);

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

  // Directory Selection & Recording logic
  const handleRecordToggle = async () => {
    if (isRecording) {
      // Stop recording
      if (mediaRecorderRef.current && mediaRecorderRef.current.state !== "inactive") {
        mediaRecorderRef.current.stop();
      } else {
        triggerVideoDownload();
      }
      setIsRecording(false);
    } else {
      // Prompt user for custom directory when recording starts!
      try {
        if ("showDirectoryPicker" in window) {
          try {
            const dirHandle = await (window as any).showDirectoryPicker({ mode: "readwrite" });
            if (dirHandle && dirHandle.name) {
              setDownloadDir(`/${dirHandle.name}/`);
            }
          } catch (e) {
            // Cancelled or unsupported in sandbox
          }
        } else {
          const userPrompt = window.prompt("Target Video Recording Download Folder / Directory:", downloadDir);
          if (userPrompt !== null && userPrompt.trim().length > 0) {
            setDownloadDir(userPrompt.trim());
          }
        }
      } catch (err) {
        console.warn("Directory selector error:", err);
      }

      // Initialize media recorder on viewport canvas stream
      try {
        const canvasElem = mountRef.current?.querySelector("canvas");
        if (canvasElem && typeof (canvasElem as any).captureStream === "function") {
          const stream = (canvasElem as any).captureStream(30);
          const recorder = new MediaRecorder(stream, { mimeType: "video/webm" });
          recordedChunksRef.current = [];
          recorder.ondataavailable = (e) => {
            if (e.data.size > 0) recordedChunksRef.current.push(e.data);
          };
          recorder.onstop = () => {
            triggerVideoDownload();
          };
          recorder.start();
          mediaRecorderRef.current = recorder;
        }
      } catch (e) {
        console.warn("Canvas stream recording fallback:", e);
      }

      setIsRecording(true);
    }
  };

  const triggerVideoDownload = () => {
    const hasData = recordedChunksRef.current.length > 0;
    const blob = hasData
      ? new Blob(recordedChunksRef.current, { type: "video/webm" })
      : new Blob([`[CaveX Explorer Pro] Telemetry & Video Log - Stream Recorded`], { type: "text/plain" });

    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    const dateStr = new Date().toISOString().replace(/[:.]/g, "-");
    const filename = `CaveX_Video_${dateStr}.${hasData ? "webm" : "txt"}`;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    alert(`[CaveX Explorer Pro] Video Stream Export Complete!\n\nTarget Directory: ${downloadDir}\nFile Name: ${filename}`);
  };

  // Animation & Physics Loop
  useEffect(() => {
    if (!mountRef.current) return;

    const width = mountRef.current.clientWidth || 800;
    const height = mountRef.current.clientHeight || 500;
    const aspect = height > 0 ? width / height : 1.6;

    // 1. Scene
    const scene = new THREE.Scene();
    sceneRef.current = scene;
    scene.background = new THREE.Color(0x0a0c10);
    scene.fog = new THREE.FogExp2(0x0a0c10, 0.04);

    // 2. Camera
    const camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 100);
    cameraRef.current = camera;
    camera.position.set(-10, 8, 12);
    camera.lookAt(0, 0, 0);

    // 3. Renderer with safe WebGL context creation and fallback
    let renderer: THREE.WebGLRenderer | null = null;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "default", failIfMajorPerformanceCaveat: false });
      renderer.setSize(width, height);
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
      renderer.shadowMap.enabled = true;
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    } catch (e1) {
      console.warn("Primary WebGL initialization failed, trying fallback WebGL parameters...", e1);
      try {
        renderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: "low-power" });
        renderer.setSize(width, height);
      } catch (e2) {
        console.error("WebGL context creation error:", e2);
        setHasWebGLError(true);
        return;
      }
    }

    if (!renderer) {
      setHasWebGLError(true);
      return;
    }

    rendererRef.current = renderer;

    const handleContextLost = (e: Event) => {
      e.preventDefault();
      console.warn("WebGL Context Lost");
      setHasWebGLError(true);
    };
    renderer.domElement.addEventListener("webglcontextlost", handleContextLost, false);

    mountRef.current.innerHTML = "";
    mountRef.current.appendChild(renderer.domElement);

    // 4. Lights (BRIGHTENED FOR CLEARER CAVE & FPV ILLUMINATION)
    const ambientLight = new THREE.AmbientLight(0x4a5d7c, 3.5);
    ambientLightRef.current = ambientLight;
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xb3d9ff, 3.8);
    dirLight.position.set(5, 18, 10);
    dirLight.castShadow = true;
    dirLightRef.current = dirLight;
    scene.add(dirLight);

    // Cave Ambient Pointlights
    const dryLight = new THREE.PointLight(0xffb366, 2.2, 18);
    dryLight.position.set(-12, 4, 2);
    scene.add(dryLight);

    const waterLight = new THREE.PointLight(0x00f0ff, 2.5, 20);
    waterLight.position.set(3, -1, 0);
    scene.add(waterLight);

    const airLight = new THREE.PointLight(0xc880ff, 2.5, 20);
    airLight.position.set(18, 5, 0);
    scene.add(airLight);

    // ==========================================
    // 5. CAVE ENVIRONMENT GENERATION (3 SECTIONS)
    // ==========================================
    const caveGroup = new THREE.Group();

    // SECTION 1: DRY CAVE (x: -39 to -5, z > 0)
    const dryGroundGeo = new THREE.PlaneGeometry(34, 12, 64, 24);
    const dryGroundMat = new THREE.MeshStandardMaterial({
      color: 0x2d2a26,
      roughness: 0.9,
      metalness: 0.1,
      bumpScale: 0.3,
    });
    // Add roughness elevation to dry ground
    const posAttr = dryGroundGeo.attributes.position;
    for (let i = 0; i < posAttr.count; i++) {
      const x = posAttr.getX(i);
      const y = posAttr.getY(i);
      const noise = Math.sin(x * 0.8) * Math.cos(y * 0.8) * 0.35 + Math.random() * 0.15;
      posAttr.setZ(i, noise + 0.4); // z > 0
    }
    dryGroundGeo.computeVertexNormals();
    const dryGround = new THREE.Mesh(dryGroundGeo, dryGroundMat);
    dryGround.rotation.x = -Math.PI / 2;
    dryGround.position.set(-22, 0, 0);
    dryGround.receiveShadow = true;
    caveGroup.add(dryGround);

    // Stalagmites on dry section
    const stalagmiteMat = new THREE.MeshStandardMaterial({ color: 0x3d3935, roughness: 0.8 });
    for (let i = 0; i < 24; i++) {
      const height = 0.8 + Math.random() * 1.8;
      const coneGeo = new THREE.ConeGeometry(0.3 + Math.random() * 0.3, height, 6);
      const cone = new THREE.Mesh(coneGeo, stalagmiteMat);
      cone.position.set(-35 + Math.random() * 25, height / 2, -5 + Math.random() * 10);
      caveGroup.add(cone);
    }

    // SECTION 2: FLOODED WATER SECTION (x: -5 to 29, z=0.6 water surface matching dry section end, seabed at z=-2.9)
    // Water Surface Plane (z = 0.6m matching dry section ground level)
    const waterGeo = new THREE.PlaneGeometry(34, 12, 64, 24);
    const waterMat = new THREE.MeshPhysicalMaterial({
      color: 0x0066aa,
      transparent: true,
      opacity: 0.75,
      roughness: 0.1,
      metalness: 0.1,
      transmission: 0.6,
      ior: 1.33,
    });
    const waterMesh = new THREE.Mesh(waterGeo, waterMat);
    waterMesh.rotation.x = -Math.PI / 2;
    waterMesh.position.set(12, 0.6, 0); // z = 0.6 water plane matching dry section end
    waterMeshRef.current = waterMesh;
    caveGroup.add(waterMesh);

    // Underwater Seabed Floor (z = -2.9)
    const seabedGeo = new THREE.PlaneGeometry(34, 12, 40, 15);
    const seabedMat = new THREE.MeshStandardMaterial({ color: 0x121c24, roughness: 0.9 });
    const seabedMesh = new THREE.Mesh(seabedGeo, seabedMat);
    seabedMesh.rotation.x = -Math.PI / 2;
    seabedMesh.position.set(12, -2.9, 0);
    caveGroup.add(seabedMesh);

    // Submerged rock formations
    const rockMat = new THREE.MeshStandardMaterial({ color: 0x1a2630, roughness: 0.9 });
    for (let i = 0; i < 16; i++) {
      const rockGeo = new THREE.DodecahedronGeometry(0.5 + Math.random() * 0.7);
      const rock = new THREE.Mesh(rockGeo, rockMat);
      rock.position.set(-3 + Math.random() * 26, -2.2, -4 + Math.random() * 8);
      caveGroup.add(rock);
    }

    // SECTION 3: AIR POCKET CAVE & ENLARGED SHADED VERTICAL SHAFT (x: 29 to 41)
    
    // Air Pocket Beach (Ground for Base Station at x=29 to 41)
    const beachGeo = new THREE.PlaneGeometry(12, 12, 32, 12);
    const beachMat = new THREE.MeshStandardMaterial({
      color: 0x3d3a36,
      roughness: 0.95,
      metalness: 0.05,
      bumpScale: 0.2,
    });
    // Add roughness elevation to beach
    const beachPosAttr = beachGeo.attributes.position;
    for (let i = 0; i < beachPosAttr.count; i++) {
      const x = beachPosAttr.getX(i);
      const y = beachPosAttr.getY(i);
      const noise = Math.sin(x * 1.5) * Math.cos(y * 1.5) * 0.2 + Math.random() * 0.1;
      beachPosAttr.setZ(i, noise);
    }
    beachGeo.computeVertexNormals();
    const beach = new THREE.Mesh(beachGeo, beachMat);
    beach.rotation.x = -Math.PI / 2;
    beach.position.set(35, 0.8, 0); // z=0.8 for the beach
    beach.receiveShadow = true;
    caveGroup.add(beach);

    // Dedicated Enlarged Shaded Vertical Ascent Shaft Chimney (Radius 5.2m, Height 25.0m)
    // Smooth shaded surface material with transparency = 0.7 (NO wireframe)
    const shaftGeo = new THREE.CylinderGeometry(5.2, 5.2, 25.0, 32, 25, true, Math.PI * 0.25, Math.PI * 1.5);
    const shaftMat = new THREE.MeshPhysicalMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.7, // Transparency set to 0.7 as requested
      roughness: 0.2,
      metalness: 0.15,
      transmission: 0.65,
      ior: 1.2,
      flatShading: false, // Smooth shaded cylinder surface
      side: THREE.DoubleSide,
    });
    const shaftMesh = new THREE.Mesh(shaftGeo, shaftMat);
    shaftMesh.position.set(35.5, 13.1, 0);
    caveGroup.add(shaftMesh);

    // Solid Shaded Structural Bezel Collar Rings for Vertical Shaft (Shaded, non-wireframe accents)
    [0.6, 25.6].forEach((collarY) => {
      const collarGeo = new THREE.TorusGeometry(5.22, 0.08, 12, 36);
      const collarMat = new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.3, metalness: 0.5 });
      const collar = new THREE.Mesh(collarGeo, collarMat);
      collar.rotation.x = Math.PI / 2;
      collar.position.set(35.5, collarY, 0);
      caveGroup.add(collar);
    });

    // Shaft Entrance Portal Archway (Spacious entry portal for drone at base)
    const portalArchGeo = new THREE.TorusGeometry(5.25, 0.12, 12, 32, Math.PI);
    const portalArchMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.85 });
    const portalArch = new THREE.Mesh(portalArchGeo, portalArchMat);
    portalArch.rotation.y = Math.PI / 2;
    portalArch.position.set(30.3, 3.5, 0);
    caveGroup.add(portalArch);

    // Glowing Guidance Rings Inside Vertical Shaft (Spanning up to 23.0m altitude)
    [3.2, 6.5, 9.8, 13.0, 16.0, 19.5, 23.0].forEach((ringY, idx) => {
      const ringGeo = new THREE.TorusGeometry(5.15, 0.05, 8, 36);
      const ringMat = new THREE.MeshBasicMaterial({ color: idx === 6 ? 0x10b981 : 0x06b6d4, transparent: true, opacity: 0.75 });
      const ringMesh = new THREE.Mesh(ringGeo, ringMat);
      ringMesh.rotation.x = Math.PI / 2;
      ringMesh.position.set(35.5, ringY, 0);
      caveGroup.add(ringMesh);
    });

    // =========================================================================
    // COMPACT OBSTACLES INSIDE VERTICAL SHAFT (Easily navigable flight channel)
    // =========================================================================
    const obsMat = new THREE.MeshStandardMaterial({ color: 0x4a3f35, roughness: 0.9 });
    const obsHazardMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b }); // Amber hazard beacon

    // Obstacle 1: Compact Lower Shaft Spire (North Wall at Y = 5.5m)
    const obs1Geo = new THREE.ConeGeometry(0.45, 1.2, 8);
    const obs1 = new THREE.Mesh(obs1Geo, obsMat);
    obs1.rotation.z = -Math.PI / 3;
    obs1.position.set(33.5, 5.5, -4.2);
    caveGroup.add(obs1);

    const obs1Light = new THREE.Mesh(new THREE.SphereGeometry(0.12, 8, 8), obsHazardMat);
    obs1Light.position.set(33.8, 5.5, -3.5);
    caveGroup.add(obs1Light);

    // Obstacle 2: Compact Mid-Shaft Rock Outcrop (South Wall at Y = 9.2m)
    const obs2Geo = new THREE.BoxGeometry(0.8, 0.4, 0.8);
    const obs2 = new THREE.Mesh(obs2Geo, obsMat);
    obs2.position.set(37.2, 9.2, 4.2);
    caveGroup.add(obs2);

    const obs2Light = new THREE.Mesh(new THREE.SphereGeometry(0.12, 8, 8), new THREE.MeshBasicMaterial({ color: 0xef4444 }));
    obs2Light.position.set(36.6, 9.2, 3.5);
    caveGroup.add(obs2Light);

    // Obstacle 3: Compact Upper Shaft Stalactite (North Wall at Y = 14.5m)
    const obs3Geo = new THREE.ConeGeometry(0.45, 1.2, 8);
    const obs3 = new THREE.Mesh(obs3Geo, obsMat);
    obs3.rotation.x = Math.PI;
    obs3.position.set(34.2, 14.5, -4.2);
    caveGroup.add(obs3);

    const obs3Light = new THREE.Mesh(new THREE.SphereGeometry(0.12, 8, 8), obsHazardMat);
    obs3Light.position.set(34.2, 13.8, -3.5);
    caveGroup.add(obs3Light);

    // Obstacle 4: Compact Upper Shaft Rock Shelf (South Wall at Y = 16.8m)
    const obs4Geo = new THREE.BoxGeometry(0.7, 0.3, 0.7);
    const obs4 = new THREE.Mesh(obs4Geo, obsMat);
    obs4.position.set(36.8, 16.8, 4.2);
    caveGroup.add(obs4);

    const obs4Light = new THREE.Mesh(new THREE.SphereGeometry(0.12, 8, 8), obsHazardMat);
    obs4Light.position.set(36.3, 16.8, 3.5);
    caveGroup.add(obs4Light);

    // Cave Walls & High Cavern Ceiling (Semi-transparent for interior visibility)
    const wallMat = new THREE.MeshStandardMaterial({
      color: 0x1c1a17,
      roughness: 0.95,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.45,
    });

    // Expanded High Cavern Ceiling (Height extended up to 26.5m over 25m vertical shaft)
    const ceilingGeo = new THREE.PlaneGeometry(80, 16, 20, 10);
    const ceiling = new THREE.Mesh(ceilingGeo, wallMat);
    ceiling.rotation.x = Math.PI / 2;
    ceiling.position.set(1, 26.5, 0);
    caveGroup.add(ceiling);

    // Stalactites hanging down
    for (let i = 0; i < 40; i++) {
      const len = 0.8 + Math.random() * 2.2;
      const coneGeo = new THREE.ConeGeometry(0.25, len, 6);
      const stalactite = new THREE.Mesh(coneGeo, stalagmiteMat);
      stalactite.rotation.x = Math.PI;
      stalactite.position.set(-39 + Math.random() * 80, 8.5 - len / 2, -5 + Math.random() * 10);
      caveGroup.add(stalactite);
    }

    // Back Cave Wall (Expanded height)
    const backWallGeo = new THREE.PlaneGeometry(80, 26.5);
    const backWall = new THREE.Mesh(backWallGeo, wallMat);
    backWall.position.set(1, 13.25, -6);
    caveGroup.add(backWall);

    // Front Cave Wall
    const frontWallGeo = new THREE.PlaneGeometry(80, 26.5);
    const frontWall = new THREE.Mesh(frontWallGeo, wallMat);
    frontWall.position.set(1, 13.25, 6);
    caveGroup.add(frontWall);

    scene.add(caveGroup);

    // 5.8 Editor Group for Custom User Spawned 3D Props
    const editorGroup = new THREE.Group();
    editorObjectsGroupRef.current = editorGroup;
    scene.add(editorGroup);

    // ==========================================
    // 5.5 SLAM 3D MESH RECONSTRUCTION (1cm Voxel Grid)
    // ==========================================
    const maxVoxels = 50000;
    const voxelGeo = new THREE.BoxGeometry(0.015, 0.015, 0.015); // 1cm high-resolution voxel units
    const voxelMat = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      roughness: 0.7,
      metalness: 0.3,
      transparent: true,
      opacity: 0.92,
    });
    const slamVoxelMesh = new THREE.InstancedMesh(voxelGeo, voxelMat, maxVoxels);
    slamVoxelMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    if (slamVoxelMesh.instanceColor) slamVoxelMesh.instanceColor.setUsage(THREE.DynamicDrawUsage);
    slamVoxelMesh.count = 0;
    slamVoxelMesh.castShadow = true;
    slamVoxelMesh.receiveShadow = true;
    scene.add(slamVoxelMesh);
    slamVoxelMeshRef.current = slamVoxelMesh;

    // ==========================================
    // 6. BOSTON DYNAMICS SPOT QUADRUPED & DETACHABLE FLYING DRONE
    // ==========================================
    const robotGroup = new THREE.Group();
    robotGroupRef.current = robotGroup;

    // --- A. BOSTON DYNAMICS "SPOT" QUADRUPED CHASSIS ---
    const spotGroup = new THREE.Group();
    spotGroupRef.current = spotGroup;
    robotGroup.add(spotGroup);

    // Spot Colors & Materials
    const spotYellowMat = new THREE.MeshStandardMaterial({
      color: 0xfacc15, // Spot Vibrant Yellow
      metalness: 0.25,
      roughness: 0.35,
    });
    const spotDarkMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b, // Dark Carbon Frame
      metalness: 0.8,
      roughness: 0.4,
    });
    const spotHeadMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a, // Front Sensor Head Matte Black
      metalness: 0.9,
      roughness: 0.3,
    });
    const jointMat = new THREE.MeshStandardMaterial({
      color: 0xf97316, // Orange/Amber Joint Accents
      metalness: 0.6,
      roughness: 0.4,
    });

    // Spot Main Body Top Shell (Yellow Protective Fairing)
    const spotTopGeo = new THREE.BoxGeometry(2.30, 0.22, 0.48);
    const spotTopMesh = new THREE.Mesh(spotTopGeo, spotYellowMat);
    spotTopMesh.position.set(0, 0.08, 0);
    spotTopMesh.castShadow = true;
    spotGroup.add(spotTopMesh);

    // Spot Lower Frame Chassis (Dark Slate Carbon)
    const spotBottomGeo = new THREE.BoxGeometry(2.36, 0.20, 0.46);
    const spotBottomMesh = new THREE.Mesh(spotBottomGeo, spotDarkMat);
    spotBottomMesh.position.set(0, -0.06, 0);
    spotBottomMesh.castShadow = true;
    spotGroup.add(spotBottomMesh);

    // Spot Side Accent Plates (Yellow side panels)
    [-0.245, 0.245].forEach((zSide) => {
      const sidePlateGeo = new THREE.BoxGeometry(1.6, 0.14, 0.02);
      const sidePlate = new THREE.Mesh(sidePlateGeo, spotYellowMat);
      sidePlate.position.set(0, 0.02, zSide);
      spotGroup.add(sidePlate);
    });

    // Spot Front Sensor Head Module (Stereo Camera & Depth Sensor Face)
    const spotHeadGeo = new THREE.BoxGeometry(0.22, 0.24, 0.40);
    const spotHeadMesh = new THREE.Mesh(spotHeadGeo, spotHeadMat);
    spotHeadMesh.position.set(1.15, 0.02, 0);
    spotGroup.add(spotHeadMesh);

    // Front Stereo Camera Lenses
    [-0.12, 0.12].forEach((zCam) => {
      const lensGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.02, 16);
      const lensMat = new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.1, metalness: 0.9 });
      const lens = new THREE.Mesh(lensGeo, lensMat);
      lens.rotation.z = Math.PI / 2;
      lens.position.set(1.26, 0.04, zCam);
      spotGroup.add(lens);
    });

    // Spot Top Carry Handles / Roll Rails
    [-0.21, 0.21].forEach((zRail) => {
      const railGeo = new THREE.CylinderGeometry(0.015, 0.015, 1.8, 8);
      const railMesh = new THREE.Mesh(railGeo, spotYellowMat);
      railMesh.rotation.z = Math.PI / 2;
      railMesh.position.set(-0.05, 0.20, zRail);
      spotGroup.add(railMesh);
    });

    // Spot Top Drone Docking Bay Cradle
    const dockPlateGeo = new THREE.BoxGeometry(0.52, 0.04, 0.42);
    const dockPlateMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.5, metalness: 0.8 });
    const dockPlate = new THREE.Mesh(dockPlateGeo, dockPlateMat);
    dockPlate.position.set(-0.1, 0.19, 0);
    spotGroup.add(dockPlate);

    // Docking Guide Latch Pins & Status LEDs
    [
      { x: 0.12, z: 0.16 },
      { x: 0.12, z: -0.16 },
      { x: -0.32, z: 0.16 },
      { x: -0.32, z: -0.16 },
    ].forEach((pinPos) => {
      const pinMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.06, 12), jointMat);
      pinMesh.position.set(pinPos.x, 0.22, pinPos.z);
      spotGroup.add(pinMesh);
    });

    // Docking Bay Latch Status Light Bar
    const latchLightGeo = new THREE.BoxGeometry(0.2, 0.02, 0.04);
    const latchLightMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const latchLight = new THREE.Mesh(latchLightGeo, latchLightMat);
    latchLight.position.set(-0.1, 0.22, 0);
    spotGroup.add(latchLight);

    // --- Spot Articulated Quadruped Legs (FL, FR, BL, BR) ---
    const legs: THREE.Group[] = [];
    const legPositions = [
      { x: 1.05, z: 0.32, name: "FL" },
      { x: 1.05, z: -0.32, name: "FR" },
      { x: -1.05, z: 0.32, name: "BL" },
      { x: -1.05, z: -0.32, name: "BR" },
    ];

    legPositions.forEach((pos) => {
      const legGroup = new THREE.Group();
      legGroup.position.set(pos.x, -0.05, pos.z);

      // 1. Hip Knuckle Module
      const hipMesh = new THREE.Mesh(new THREE.SphereGeometry(0.075, 12, 12), spotDarkMat);
      legGroup.add(hipMesh);

      const hipCap = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.04, 12), spotYellowMat);
      hipCap.rotation.x = Math.PI / 2;
      hipCap.position.set(0, 0, pos.z > 0 ? 0.05 : -0.05);
      legGroup.add(hipCap);

      // 2. Thigh / Upper Leg Link (Angled Carbon Arm + Yellow Armor Plate)
      const thighGroup = new THREE.Group();
      thighGroup.position.set(0, 0, 0);

      const thighMesh = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.42, 0.08), spotDarkMat);
      thighMesh.position.set(0, -0.21, 0);
      thighMesh.castShadow = true;
      thighGroup.add(thighMesh);

      // Spot Yellow Upper Leg Armor Shield
      const armorMesh = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.36, 0.02), spotYellowMat);
      armorMesh.position.set(0, -0.20, pos.z > 0 ? 0.045 : -0.045);
      thighGroup.add(armorMesh);

      // Knee Joint Pivot
      const kneeMesh = new THREE.Mesh(new THREE.SphereGeometry(0.05, 10, 10), jointMat);
      kneeMesh.position.set(0, -0.42, 0);
      thighGroup.add(kneeMesh);

      // 3. Shank / Lower Leg Link (Tapered Carbon Shaft)
      const shankMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.04, 0.48, 12), spotDarkMat);
      shankMesh.position.set(0, -0.66, 0);
      shankMesh.rotation.z = -0.12; // Realistic Spot backward knee bend
      thighGroup.add(shankMesh);

      // Foot Pad (Soft Rubber Dome)
      const footMesh = new THREE.Mesh(new THREE.SphereGeometry(0.055, 12, 12), spotHeadMat);
      footMesh.position.set(-0.05, -0.90, 0);
      thighGroup.add(footMesh);

      legGroup.add(thighGroup);
      spotGroup.add(legGroup);
      legs.push(legGroup);
    });
    legsRef.current = legs;

    // --- Spot Hydrofoil Pontoons (Sailing Mode) ---
    const pontoons: THREE.Group[] = [];
    [-0.52, 0.52].forEach((zPos) => {
      const pGroup = new THREE.Group();
      pGroup.position.set(0, -0.05, zPos);

      // Yellow + Dark Hydrodynamic Float Hull
      const pontoonMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 2.70, 16), spotYellowMat);
      pontoonMesh.rotation.z = Math.PI / 2;

      // Dark Hydrodynamic Nose Cones
      const noseGeo = new THREE.ConeGeometry(0.11, 0.3, 16);
      const noseFront = new THREE.Mesh(noseGeo, spotDarkMat);
      noseFront.rotation.z = -Math.PI / 2;
      noseFront.position.set(1.5, 0, 0);
      pGroup.add(noseFront);

      const noseBack = new THREE.Mesh(noseGeo, spotDarkMat);
      noseBack.rotation.z = Math.PI / 2;
      noseBack.position.set(-1.5, 0, 0);
      pGroup.add(noseBack);

      pGroup.add(pontoonMesh);

      // Deployment Link Arm
      const armGeo = new THREE.CylinderGeometry(0.02, 0.02, 0.35, 8);
      const armMesh = new THREE.Mesh(armGeo, spotDarkMat);
      armMesh.position.set(0, 0.18, 0);
      pGroup.add(armMesh);

      spotGroup.add(pGroup);
      pontoons.push(pGroup);
    });
    pontoonsRef.current = pontoons;

    // --- B. DETACHABLE AERIAL FLYING DRONE ---
    const droneGroup = new THREE.Group();
    droneGroupRef.current = droneGroup;

    // Drone Avionics Hub Core (Dark Graphite + Yellow Trim)
    const droneCoreGeo = new THREE.CylinderGeometry(0.22, 0.24, 0.12, 16);
    const droneCoreMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.3, metalness: 0.9 });
    const droneCore = new THREE.Mesh(droneCoreGeo, droneCoreMat);
    droneGroup.add(droneCore);

    // Drone Top Shell Fairing (Spot Yellow Accent)
    const droneCapGeo = new THREE.CylinderGeometry(0.14, 0.20, 0.06, 16);
    const droneCap = new THREE.Mesh(droneCapGeo, spotYellowMat);
    droneCap.position.y = 0.08;
    droneGroup.add(droneCap);

    // Glowing LED Ring around Drone Core
    const droneRingGeo = new THREE.TorusGeometry(0.23, 0.018, 8, 24);
    const droneRingMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff });
    const droneRing = new THREE.Mesh(droneRingGeo, droneRingMat);
    droneRing.rotation.x = Math.PI / 2;
    droneGroup.add(droneRing);

    // Front HD FPV Camera Gimbal
    const gimbalGeo = new THREE.SphereGeometry(0.075, 12, 12);
    const gimbalMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.2 });
    const gimbal = new THREE.Mesh(gimbalGeo, gimbalMat);
    gimbal.position.set(0.24, 0, 0);
    droneGroup.add(gimbal);

    // FPV Optical Camera Glass Lens
    const fpvLensGeo = new THREE.CylinderGeometry(0.035, 0.035, 0.02, 16);
    const fpvLensMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, roughness: 0.1, metalness: 0.9 });
    const fpvLens = new THREE.Mesh(fpvLensGeo, fpvLensMat);
    fpvLens.rotation.z = Math.PI / 2;
    fpvLens.position.set(0.31, 0, 0);
    droneGroup.add(fpvLens);

    // 4 Carbon Rotor Arms + Shrouded Ducted Propeller Guards
    const props: THREE.Mesh[] = [];
    const propMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.8 });
    const armPositions = [
      { x: 0.38, z: 0.38 },
      { x: 0.38, z: -0.38 },
      { x: -0.38, z: 0.38 },
      { x: -0.38, z: -0.38 },
    ];

    armPositions.forEach((pPos) => {
      // Diagonal Carbon Arm
      const armGeo = new THREE.CylinderGeometry(0.02, 0.02, 0.52, 8);
      const armMesh = new THREE.Mesh(armGeo, spotDarkMat);
      armMesh.rotation.x = Math.PI / 2;
      armMesh.rotation.z = Math.atan2(pPos.z, pPos.x);
      armMesh.position.set(pPos.x / 2, 0, pPos.z / 2);
      droneGroup.add(armMesh);

      // Ducted Propeller Guard Shroud (Ring around prop)
      const shroudGeo = new THREE.TorusGeometry(0.22, 0.018, 12, 24);
      const shroudMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.8, roughness: 0.4 });
      const shroud = new THREE.Mesh(shroudGeo, shroudMat);
      shroud.rotation.x = Math.PI / 2;
      shroud.position.set(pPos.x, 0.02, pPos.z);
      droneGroup.add(shroud);

      // Spinning Propeller Disc
      const propDisc = new THREE.Mesh(new THREE.CylinderGeometry(0.20, 0.20, 0.008, 16), propMat);
      propDisc.position.set(pPos.x, 0.03, pPos.z);
      droneGroup.add(propDisc);
      props.push(propDisc);
    });
    propellersRef.current = props;

    // Underside Docking Latch Pins (Fit onto Spot's back)
    [
      { x: 0.12, z: 0.16 },
      { x: 0.12, z: -0.16 },
      { x: -0.32, z: 0.16 },
      { x: -0.32, z: -0.16 },
    ].forEach((pinPos) => {
      const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.05, 8), spotDarkMat);
      pin.position.set(pinPos.x, -0.08, pinPos.z);
      droneGroup.add(pin);
    });

    // Attach Sensor Suite onto Detachable Drone
    // LiDAR Dome
    const lidarGroup = new THREE.Group();
    lidarGroup.position.set(0, 0.13, 0);
    const lidarBase = new THREE.Mesh(
      new THREE.CylinderGeometry(0.08, 0.08, 0.06, 16),
      new THREE.MeshStandardMaterial({ color: 0x0f172a })
    );
    const lidarDome = new THREE.Mesh(
      new THREE.CylinderGeometry(0.07, 0.07, 0.06, 16),
      new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.2 })
    );
    lidarDome.position.y = 0.06;
    lidarGroup.add(lidarBase);
    lidarGroup.add(lidarDome);
    droneGroup.add(lidarGroup);
    lidarMeshRef.current = lidarGroup;

    // Laser Ray Visualizer Lines
    const rayCount = 180;
    const linePositions = new Float32Array(rayCount * 6);
    const rayGeo = new THREE.BufferGeometry();
    rayGeo.setAttribute("position", new THREE.BufferAttribute(linePositions, 3));
    const rayMat = new THREE.LineBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.6 });
    const laserRays = new THREE.LineSegments(rayGeo, rayMat);
    laserRays.position.set(0, 0.20, 0);
    droneGroup.add(laserRays);
    laserRaysRef.current = laserRays;

    // Underwater Sonar Acoustic Pulse Rings
    const sonarGeo = new THREE.RingGeometry(0.1, 0.15, 24);
    const sonarMat = new THREE.MeshBasicMaterial({
      color: 0xff3344,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.8,
    });
    const sonarMesh = new THREE.Mesh(sonarGeo, sonarMat);
    sonarMesh.rotation.x = Math.PI / 2;
    sonarMesh.position.set(0.3, -0.1, 0);
    droneGroup.add(sonarMesh);
    sonarPulseMeshRef.current = sonarMesh;

    // Spotlight Headlight on Drone Nose
    const spotlight = new THREE.SpotLight(0xffffff, 8.0, 30, Math.PI / 4, 0.3, 1);
    spotlight.position.set(0.3, 0, 0);
    spotlight.target.position.set(3, 0, 0);
    spotlightRef.current = spotlight;
    droneGroup.add(spotlight);
    droneGroup.add(spotlight.target);

    // Initial Docked Position on top of Spot's back
    droneGroup.position.set(-0.1, 0.28, 0);
    robotGroup.add(droneGroup);

    // 3D Anti-Collision Shield Wireframe Sphere
    const shieldGeo = new THREE.SphereGeometry(1.6, 18, 18);
    const shieldMat = new THREE.MeshBasicMaterial({
      color: 0x10b981,
      wireframe: true,
      transparent: true,
      opacity: 0.18,
    });
    const shieldMesh = new THREE.Mesh(shieldGeo, shieldMat);
    shieldMeshRef.current = shieldMesh;
    robotGroup.add(shieldMesh);

    // Initial position on dry cave ground
    robotGroup.position.set(-15, 0.6, 0);
    scene.add(robotGroup);

    // ==========================================
    // 7. MOUSE ORBIT CONTROLS & RESIZE
    // ==========================================
    const handleMouseDown = (e: MouseEvent) => {
      isMouseDragging.current = true;
      previousMousePos.current = { x: e.clientX, y: e.clientY };
    };

    const handleMouseMove = (e: MouseEvent) => {
      if (!isMouseDragging.current) return;
      const deltaX = e.clientX - previousMousePos.current.x;
      const deltaY = e.clientY - previousMousePos.current.y;

      cameraAngle.current.theta -= deltaX * 0.008;
      cameraAngle.current.phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, cameraAngle.current.phi + deltaY * 0.008));

      previousMousePos.current = { x: e.clientX, y: e.clientY };
    };

    const handleMouseUp = () => {
      isMouseDragging.current = false;
    };

    const handleWheel = (e: WheelEvent) => {
      cameraAngle.current.radius = Math.max(4, Math.min(30, cameraAngle.current.radius + e.deltaY * 0.015));
    };

    const handleCanvasClick = (e: MouseEvent) => {
      const deltaX = Math.abs(e.clientX - previousMousePos.current.x);
      const deltaY = Math.abs(e.clientY - previousMousePos.current.y);
      if (deltaX > 6 || deltaY > 6) return;

      if (!mountRef.current || !rendererRef.current || !cameraRef.current || !sceneRef.current) return;

      const rect = mountRef.current.getBoundingClientRect();
      const mouse = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1
      );

      const raycaster = new THREE.Raycaster();
      raycaster.setFromCamera(mouse, cameraRef.current);

      if (clickToPlaceTypeRef.current) {
        const intersects = raycaster.intersectObjects(sceneRef.current.children, true);
        if (intersects.length > 0) {
          const pt = intersects[0].point;
          spawnNewObject(clickToPlaceTypeRef.current, {
            x: Number(pt.x.toFixed(2)),
            y: Number(pt.y.toFixed(2)),
            z: Number(pt.z.toFixed(2)),
          });
          setClickToPlaceType(null);
        }
        return;
      }

      if (editorObjectsGroupRef.current) {
        const hits = raycaster.intersectObjects(editorObjectsGroupRef.current.children, true);
        if (hits.length > 0) {
          let topObj: THREE.Object3D | null = hits[0].object;
          while (topObj && topObj.parent && topObj.parent !== editorObjectsGroupRef.current) {
            topObj = topObj.parent;
          }
          if (topObj && topObj.userData?.objectId) {
            setSelectedObjectId(topObj.userData.objectId);
            setIsEditorMode(true);
          }
        }
      }
    };

    const domElem = mountRef.current;
    domElem.addEventListener("mousedown", handleMouseDown);
    domElem.addEventListener("click", handleCanvasClick);
    window.addEventListener("mousemove", handleMouseMove);
    window.addEventListener("mouseup", handleMouseUp);
    domElem.addEventListener("wheel", handleWheel);

    const handleResize = () => {
      if (!mountRef.current || !rendererRef.current || !cameraRef.current) return;
      const w = mountRef.current.clientWidth || 800;
      const h = mountRef.current.clientHeight || 500;
      if (w === 0 || h === 0) return;
      cameraRef.current.aspect = w / h;
      cameraRef.current.updateProjectionMatrix();
      rendererRef.current.setSize(w, h);
    };
    window.addEventListener("resize", handleResize);

    // ==========================================
    // 8. MAIN ANIMATION & PHYSICS TICK LOOP
    // ==========================================
    let animationFrameId: number;
    let clock = new THREE.Clock();

    const animate = () => {
      animationFrameId = requestAnimationFrame(animate);
      const elapsedTime = clock.getElapsedTime();

      // Update Water Wave Animation
      if (waterMeshRef.current) {
        const waterPos = waterMeshRef.current.geometry.attributes.position;
        for (let i = 0; i < waterPos.count; i++) {
          const u = waterPos.getX(i);
          const v = waterPos.getY(i);
          const wave = Math.sin(u * 1.5 + elapsedTime * 3) * Math.cos(v * 1.5 + elapsedTime * 2) * 0.08;
          waterPos.setZ(i, wave);
        }
        waterMeshRef.current.geometry.attributes.position.needsUpdate = true;
      }

      // SLAM 3D MESH RECONSTRUCTION UPDATE
      const currentCamMode = activeCameraModeRef.current;
      if (robotGroupRef.current && slamVoxelMeshRef.current) {
        slamVoxelMeshRef.current.visible = true;

        const rx = robotGroupRef.current.position.x;
        const ry = robotGroupRef.current.position.y;
        const rz = robotGroupRef.current.position.z;

        // Reset SLAM Map on Simulation Reset
        if (Math.abs(rx - -15) < 0.1 && Math.abs(ry - 0.6) < 0.1 && Math.abs(rz - 0) < 0.1) {
           exploredVoxelsRef.current.clear();
           voxelCountRef.current = 0;
           slamVoxelMeshRef.current.count = 0;
        }

        // Generate Voxels around the robot at 1cm (0.01m) resolution grid
        const maxVoxels = 50000;
        const voxelRes = 0.01; // 1cm resolution
        let vCount = voxelCountRef.current;
        let needsUpdate = false;

        const scanRadius = 4.0;
        const scanStep = 0.12; // Sample step for fluid frame rate
        for (let dx = -scanRadius; dx <= scanRadius; dx += scanStep) {
          for (let dy = -scanRadius; dy <= scanRadius; dy += scanStep) {
            for (let dz = -scanRadius; dz <= scanRadius; dz += scanStep) {
              if (dx*dx + dy*dy + dz*dz <= scanRadius * scanRadius) {
                 const wx = rx + dx;
                 const wy = ry + dy; // altitude
                 const wz = rz + dz; // lateral

                 let isSolid = false;
                 // Cave bounds estimation
                 if (wx < 30) {
                    if (wy < (wx >= -5 && wx <= 29 ? -1 : 0)) isSolid = true; // floor
                    if (wz > 3.2 || wz < -3.2) isSolid = true; // walls
                 } else {
                    if (wy < 0) isSolid = true; // floor under shaft
                    if (wz*wz + (wx-35.5)*(wx-35.5) > 25) isSolid = true; // cylinder shaft wall
                 }

                 if (isSolid) {
                    // Quantize strictly to 1cm (0.01m) grid
                    const gx = Math.round(wx / voxelRes) * voxelRes;
                    const gy = Math.round(wy / voxelRes) * voxelRes;
                    const gz = Math.round(wz / voxelRes) * voxelRes;

                    const key = `${gx.toFixed(2)},${gy.toFixed(2)},${gz.toFixed(2)}`;
                    if (!exploredVoxelsRef.current.has(key) && vCount < maxVoxels) {
                       exploredVoxelsRef.current.add(key);
                       tempMatrix.setPosition(gx, gy, gz);
                       slamVoxelMeshRef.current.setMatrixAt(vCount, tempMatrix);

                       const color = new THREE.Color();
                       // Use altitude (gy) for color shading
                       color.setHSL((gy + 10) / 40, 0.85, 0.6);
                       slamVoxelMeshRef.current.setColorAt(vCount, color);

                       vCount++;
                       needsUpdate = true;
                    }
                 }
              }
            }
          }
        }
        
        if (needsUpdate) {
           slamVoxelMeshRef.current.count = vCount;
           slamVoxelMeshRef.current.instanceMatrix.needsUpdate = true;
           if (slamVoxelMeshRef.current.instanceColor) {
               slamVoxelMeshRef.current.instanceColor.needsUpdate = true;
           }
           voxelCountRef.current = vCount;
        }
      }

      // Update Robot Kinematics according to Mode & Position
      if (robotGroupRef.current) {
        const rPos = robotGroupRef.current.position;

        // Animate LiDAR Spinning Dome
        if (lidarMeshRef.current) {
          lidarMeshRef.current.rotation.y += 0.45;
        }

        // Animate LiDAR Laser Beams
        if (laserRaysRef.current && showLaserBeamsRef.current) {
          const pos = laserRaysRef.current.geometry.attributes.position;
          for (let i = 0; i < rayCount; i++) {
            const angle = (i / rayCount) * Math.PI * 2 + elapsedTime * 12;
            const dist = 3 + Math.sin(angle * 4 + elapsedTime * 2) * 1.5;
            pos.setXYZ(i * 2, 0, 0, 0);
            pos.setXYZ(i * 2 + 1, Math.cos(angle) * dist, (Math.random() - 0.5) * 1.0, Math.sin(angle) * dist);
          }
          laserRaysRef.current.geometry.attributes.position.needsUpdate = true;
          laserRaysRef.current.visible = true;
        } else if (laserRaysRef.current) {
          laserRaysRef.current.visible = false;
        }

        // Animate Sonar Pulse
        if (sonarPulseMeshRef.current && showSonarPulseRef.current) {
          const scale = 1 + ((elapsedTime * 3) % 4);
          sonarPulseMeshRef.current.scale.set(scale, scale, scale);
          (sonarPulseMeshRef.current.material as THREE.MeshBasicMaterial).opacity = Math.max(0, 1 - scale / 4);
          sonarPulseMeshRef.current.visible = true;
        } else if (sonarPulseMeshRef.current) {
          sonarPulseMeshRef.current.visible = false;
        }

        // Mode Specific Kinematic Adaptations & Detachable Drone VTOL Separation
        const currentMode = robotStateRef.current.mode;
        if (currentMode === "WALKING") {
          // Spot Quadruped Trotting Gait Motion
          const gaitSpeed = 8;
          legsRef.current.forEach((leg, idx) => {
            const offset = idx % 2 === 0 ? 0 : Math.PI;
            leg.rotation.z = Math.sin(elapsedTime * gaitSpeed + offset) * 0.35;
            leg.position.y = -0.05 + Math.max(0, Math.sin(elapsedTime * gaitSpeed + offset) * 0.08);
          });
          // Retract Pontoons
          pontoonsRef.current.forEach((p) => {
            p.position.y = 0.1;
          });
          // Drone Docked on Spot's Back Cradle
          if (droneGroupRef.current) {
            droneGroupRef.current.position.set(-0.1, 0.28, 0);
            droneGroupRef.current.rotation.set(0, 0, 0);
          }
          if (spotGroupRef.current) {
            spotGroupRef.current.position.set(0, 0, 0);
          }
          // Idle Propellers
          propellersRef.current.forEach((pr) => {
            pr.rotation.y += 0.02;
          });
        } else if (currentMode === "SAILING") {
          // Spot Lock Legs into Floating Hydrofoil Stance
          legsRef.current.forEach((leg) => {
            leg.rotation.z = -0.5;
            leg.position.y = 0.05;
          });
          // Lower Hydrofoil Pontoons to Waterline
          pontoonsRef.current.forEach((p) => {
            p.position.y = -0.18;
          });
          // Drone Docked on Spot's Back Cradle
          if (droneGroupRef.current) {
            droneGroupRef.current.position.set(-0.1, 0.28, 0);
            droneGroupRef.current.rotation.set(0, 0, 0);
          }
          if (spotGroupRef.current) {
            spotGroupRef.current.position.set(0, 0, 0);
          }
          // Slow Propeller Rotation
          propellersRef.current.forEach((pr) => {
            pr.rotation.y += 0.05;
          });
          // Bobbing Water Surface Motion (only when in flooded water section)
          if (rPos.x >= -5 && rPos.x <= 29) {
            rPos.y = -0.05 + Math.sin(elapsedTime * 2) * 0.04;
          }
        } else if (currentMode === "FLYING") {
          // Spot Quadruped Stays Grounded / Parked in Stable Stance
          legsRef.current.forEach((leg) => {
            leg.rotation.z = 0;
            leg.position.y = -0.05;
          });
          pontoonsRef.current.forEach((p) => {
            p.position.y = 0.1;
          });

          // DETACHABLE FLYING DRONE TAKES OFF / ASCENDS!
          if (droneGroupRef.current && spotGroupRef.current) {
            // Spot carrier base companion computer rests grounded at end of Flooded Zone Base Station (X=29.0, Y=0, Z=0.6m)
            // Three.js coordinates: rPos.x = ROS X, rPos.y = ROS Z altitude, rPos.z = ROS Y lateral
            const targetSpotWorldX = 29.0;
            const targetSpotWorldY = 0.6;
            const targetSpotWorldZ = 0.0;

            spotGroupRef.current.position.set(
              targetSpotWorldX - rPos.x,
              targetSpotWorldY - rPos.y,
              targetSpotWorldZ - rPos.z
            );
            spotGroupRef.current.rotation.set(0, 0, 0);

            // Detachable Drone hovers relative to flying target coordinates
            droneGroupRef.current.position.set(0, Math.sin(elapsedTime * 3) * 0.05, 0);
            droneGroupRef.current.rotation.z = Math.sin(elapsedTime * 2) * 0.04;
          }

          // High RPM Spinning Propellers for VTOL Flight
          propellersRef.current.forEach((pr) => {
            pr.rotation.y += 0.85;
          });
        }

        // Strict Anti-penetration ground floor collision guard
        const minH = getMinGroundHeight(rPos.x, rPos.z, currentMode);
        if (rPos.y < minH) {
          rPos.y = minH;
        }

        // Spotlight Headlight Toggle & FPV Exposure Boost
        const currentCamMode = activeCameraModeRef.current;
        if (spotlightRef.current) {
          spotlightRef.current.visible = headlightRef.current;
          spotlightRef.current.intensity = currentCamMode === "fpv" ? 14.0 : 8.0;
        }

        // Anti-Collision 3D Shield Mesh Animation
        if (shieldMeshRef.current) {
          const isAntiCollisionOn = antiCollisionEnabledRef.current;
          shieldMeshRef.current.visible = isAntiCollisionOn;
          if (isAntiCollisionOn) {
            shieldMeshRef.current.rotation.y += 0.015;
            shieldMeshRef.current.rotation.x += 0.008;
            if (evasionAlertRef.current?.active) {
              (shieldMeshRef.current.material as THREE.MeshBasicMaterial).color.setHex(0xef4444);
              (shieldMeshRef.current.material as THREE.MeshBasicMaterial).opacity = 0.45 + Math.sin(elapsedTime * 14) * 0.3;
            } else {
              (shieldMeshRef.current.material as THREE.MeshBasicMaterial).color.setHex(0x10b981);
              (shieldMeshRef.current.material as THREE.MeshBasicMaterial).opacity = 0.2;
            }
          }
        }

        // Mode Specific Lighting & Fog Brightness Adjustments
        if (sceneRef.current) {
          if (currentCamMode === "topdown") {
            // Top Down SLAM Map: Super bright, ultra-crisp fog-free visibility
            sceneRef.current.fog = new THREE.FogExp2(0x0f172a, 0.001);
            if (ambientLightRef.current) ambientLightRef.current.intensity = 5.2;
            if (dirLightRef.current) dirLightRef.current.intensity = 4.8;
          } else if (currentCamMode === "orbit") {
            // 3D Orbit View: High ambient illumination & low fog for crystal clear cave exploration
            sceneRef.current.fog = new THREE.FogExp2(0x0a0c10, 0.012);
            if (ambientLightRef.current) ambientLightRef.current.intensity = 4.2;
            if (dirLightRef.current) dirLightRef.current.intensity = 3.9;
          } else {
            // FPV / Follow: Atmospheric lighting with active headlight
            sceneRef.current.fog = new THREE.FogExp2(0x0a0c10, 0.032);
            if (ambientLightRef.current) ambientLightRef.current.intensity = 2.8;
            if (dirLightRef.current) dirLightRef.current.intensity = 2.8;
          }
        }

        // Camera Modes
        if (cameraRef.current) {
          if (currentCamMode === "orbit") {
            const { theta, phi, radius } = cameraAngle.current;
            cameraRef.current.position.set(
              rPos.x + radius * Math.sin(phi) * Math.cos(theta),
              rPos.y + radius * Math.cos(phi),
              rPos.z + radius * Math.sin(phi) * Math.sin(theta)
            );
            cameraRef.current.lookAt(rPos.x, rPos.y + 0.3, rPos.z);
          } else if (currentCamMode === "fpv") {
            // First Person Drone / Spot Nose Camera View (Oriented along vehicle heading)
            const headingRad = (robotStateRef.current.orientation?.z || 0) * (Math.PI / 180);
            const pitchRad = (robotStateRef.current.orientation?.y || 0) * (Math.PI / 180);
            const currentMode = robotStateRef.current.mode;

            // Position camera right at front optical camera lens
            const camHeight = currentMode === "FLYING" ? 0.22 : 0.45;
            const forwardDist = 0.45;
            const fpvX = rPos.x + forwardDist * Math.cos(headingRad);
            const fpvY = rPos.y + camHeight;
            const fpvZ = rPos.z + forwardDist * Math.sin(headingRad);

            cameraRef.current.position.set(fpvX, fpvY, fpvZ);

            const lookDist = 12;
            const lookX = fpvX + lookDist * Math.cos(headingRad) * Math.cos(pitchRad);
            const lookY = fpvY + lookDist * Math.sin(pitchRad);
            const lookZ = fpvZ + lookDist * Math.sin(headingRad) * Math.cos(pitchRad);

            cameraRef.current.lookAt(lookX, lookY, lookZ);
          } else if (currentCamMode === "follow") {
            // Third Person Drone Follow View (Smooth Chase Cam trailing robot)
            const headingRad = (robotStateRef.current.orientation?.z || 0) * (Math.PI / 180);
            const distBehind = 3.8;
            const heightAbove = 1.6;
            cameraRef.current.position.set(
              rPos.x - distBehind * Math.cos(headingRad),
              rPos.y + heightAbove,
              rPos.z - distBehind * Math.sin(headingRad)
            );
            cameraRef.current.lookAt(rPos.x + 2 * Math.cos(headingRad), rPos.y + 0.4, rPos.z + 2 * Math.sin(headingRad));
          } else if (currentCamMode === "topdown") {
            // Top Down SLAM View
            cameraRef.current.position.set(rPos.x, rPos.y + 16, rPos.z);
            cameraRef.current.lookAt(rPos.x, rPos.y, rPos.z);
          }
        }
      }

      if (rendererRef.current && cameraRef.current && sceneRef.current) {
        rendererRef.current.render(sceneRef.current, cameraRef.current);
      }
    };

    animate();

    return () => {
      cancelAnimationFrame(animationFrameId);
      if (domElem) {
        domElem.removeEventListener("mousedown", handleMouseDown);
        domElem.removeEventListener("click", handleCanvasClick);
        domElem.removeEventListener("wheel", handleWheel);
      }
      window.removeEventListener("mousemove", handleMouseMove);
      window.removeEventListener("mouseup", handleMouseUp);
      window.removeEventListener("resize", handleResize);

      if (rendererRef.current) {
        try {
          rendererRef.current.dispose();
        } catch (e) {
          // ignore cleanup errors
        }
        rendererRef.current = null;
      }
    };
  }, [hasWebGLError]);

  // Synchronize Custom Editable 3D Objects & Environment in Three.js Scene
  useEffect(() => {
    if (!editorObjectsGroupRef.current) return;
    const group = editorObjectsGroupRef.current;

    // Clear previous object children
    while (group.children.length > 0) {
      const child = group.children[0];
      group.remove(child);
    }

    // Live update Ambient Lighting & Fog & Shield Radius
    if (ambientLightRef.current) {
      ambientLightRef.current.intensity = envLightIntensity;
    }
    if (sceneRef.current?.fog) {
      (sceneRef.current.fog as THREE.FogExp2).density = waterFogDensity;
    }
    if (waterMeshRef.current) {
      (waterMeshRef.current.material as THREE.MeshStandardMaterial).color.set(waterColor);
    }
    if (shieldMeshRef.current) {
      shieldMeshRef.current.scale.setScalar(antiCollisionRadius / 1.6);
    }

    // Build 3D Meshes for Spawned Objects
    spawnedObjects.forEach((obj) => {
      if (!obj.visible) return;

      const objGroup = new THREE.Group();
      objGroup.userData = { objectId: obj.id };

      const objColor = new THREE.Color(obj.color);
      const mainMat = new THREE.MeshStandardMaterial({
        color: objColor,
        roughness: obj.type === "boulder" || obj.type === "stalagmite" ? 0.85 : 0.3,
        metalness: obj.type === "target" || obj.type === "buoy" ? 0.7 : 0.2,
      });

      if (obj.type === "boulder") {
        const rockGeo = new THREE.DodecahedronGeometry(0.8, 1);
        const rockMesh = new THREE.Mesh(rockGeo, mainMat);
        rockMesh.castShadow = true;
        rockMesh.receiveShadow = true;
        objGroup.add(rockMesh);
      } else if (obj.type === "stalagmite") {
        const coneGeo = new THREE.ConeGeometry(0.55, 2.4, 8);
        const coneMesh = new THREE.Mesh(coneGeo, mainMat);
        coneMesh.position.y = 1.2;
        coneMesh.castShadow = true;
        objGroup.add(coneMesh);
      } else if (obj.type === "buoy") {
        const baseGeo = new THREE.CylinderGeometry(0.35, 0.45, 0.6, 16);
        const baseMesh = new THREE.Mesh(baseGeo, mainMat);

        const floatDome = new THREE.Mesh(
          new THREE.SphereGeometry(0.4, 16, 12),
          new THREE.MeshStandardMaterial({ color: objColor, metalness: 0.8, roughness: 0.2 })
        );
        floatDome.position.y = 0.3;

        const mast = new THREE.Mesh(
          new THREE.CylinderGeometry(0.02, 0.02, 1.2, 8),
          new THREE.MeshStandardMaterial({ color: 0x1e293b })
        );
        mast.position.y = 0.9;

        const lightDome = new THREE.Mesh(
          new THREE.SphereGeometry(0.12, 12, 12),
          new THREE.MeshBasicMaterial({ color: 0x00f0ff })
        );
        lightDome.position.y = 1.5;

        const beaconLight = new THREE.PointLight(0x00f0ff, obj.intensity || 2.5, 12);
        beaconLight.position.y = 1.5;

        objGroup.add(baseMesh);
        objGroup.add(floatDome);
        objGroup.add(mast);
        objGroup.add(lightDome);
        objGroup.add(beaconLight);
      } else if (obj.type === "target") {
        const pole = new THREE.Mesh(
          new THREE.CylinderGeometry(0.04, 0.04, 1.8, 12),
          new THREE.MeshStandardMaterial({ color: 0x334155 })
        );
        pole.position.y = 0.9;

        const frame = new THREE.Mesh(
          new THREE.BoxGeometry(1.2, 1.2, 0.08),
          new THREE.MeshStandardMaterial({ color: 0x0f172a })
        );
        frame.position.set(0, 1.5, 0);

        const gridBoard = new THREE.Mesh(
          new THREE.PlaneGeometry(1.0, 1.0),
          new THREE.MeshBasicMaterial({ color: 0xffffff })
        );
        gridBoard.position.set(0, 1.5, 0.05);

        const crosshairRing = new THREE.Mesh(
          new THREE.RingGeometry(0.2, 0.25, 24),
          new THREE.MeshBasicMaterial({ color: objColor, side: THREE.DoubleSide })
        );
        crosshairRing.position.set(0, 1.5, 0.06);

        objGroup.add(pole);
        objGroup.add(frame);
        objGroup.add(gridBoard);
        objGroup.add(crosshairRing);
      } else if (obj.type === "spotlight") {
        const tripodBase = new THREE.Mesh(
          new THREE.CylinderGeometry(0.5, 0.7, 0.4, 6),
          new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.3 })
        );
        tripodBase.position.y = 0.2;

        const pole = new THREE.Mesh(
          new THREE.CylinderGeometry(0.06, 0.06, 2.2, 12),
          new THREE.MeshStandardMaterial({ color: 0x475569 })
        );
        pole.position.y = 1.3;

        const spotHousing = new THREE.Mesh(
          new THREE.CylinderGeometry(0.25, 0.35, 0.5, 12),
          new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.2 })
        );
        spotHousing.rotation.x = Math.PI / 4;
        spotHousing.position.set(0, 2.3, 0.2);

        const spotLight = new THREE.SpotLight(
          new THREE.Color(obj.color),
          obj.intensity || 4.0,
          25,
          Math.PI / 3,
          0.4
        );
        spotLight.position.set(0, 2.3, 0.2);
        spotLight.target.position.set(0, 0, 4);

        objGroup.add(tripodBase);
        objGroup.add(pole);
        objGroup.add(spotHousing);
        objGroup.add(spotLight);
        objGroup.add(spotLight.target);
      } else if (obj.type === "waypoint") {
        const outerRing = new THREE.Mesh(
          new THREE.TorusGeometry(0.8, 0.04, 16, 32),
          new THREE.MeshBasicMaterial({ color: objColor })
        );
        outerRing.rotation.x = Math.PI / 2;

        const innerRing = new THREE.Mesh(
          new THREE.TorusGeometry(0.4, 0.03, 16, 24),
          new THREE.MeshBasicMaterial({ color: 0xffffff })
        );
        innerRing.rotation.x = Math.PI / 2;

        const verticalShaft = new THREE.Mesh(
          new THREE.CylinderGeometry(0.015, 0.015, 4.0, 8),
          new THREE.MeshBasicMaterial({ color: objColor, transparent: true, opacity: 0.5 })
        );
        verticalShaft.position.y = 2.0;

        objGroup.add(outerRing);
        objGroup.add(innerRing);
        objGroup.add(verticalShaft);
      }

      objGroup.position.set(obj.position.x, obj.position.y, obj.position.z);
      objGroup.rotation.set(
        THREE.MathUtils.degToRad(obj.rotation.x),
        THREE.MathUtils.degToRad(obj.rotation.y),
        THREE.MathUtils.degToRad(obj.rotation.z)
      );
      objGroup.scale.set(obj.scale.x, obj.scale.y, obj.scale.z);

      // Highlight Selection Ring
      if (obj.id === selectedObjectId) {
        const maxRadius = 1.0 * Math.max(obj.scale.x, obj.scale.z);
        const selRing = new THREE.Mesh(
          new THREE.RingGeometry(maxRadius, maxRadius + 0.15, 32),
          new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide, transparent: true, opacity: 0.85 })
        );
        selRing.rotation.x = Math.PI / 2;
        selRing.position.y = -0.02;
        objGroup.add(selRing);
      }

      group.add(objGroup);
    });
  }, [
    spawnedObjects,
    selectedObjectId,
    envLightIntensity,
    waterFogDensity,
    waterColor,
    waterWaveHeight,
    antiCollisionRadius,
  ]);

  // Sync robotState position change with 3D scene & anti-clipping collision floor check
  useEffect(() => {
    if (robotGroupRef.current) {
      const rx = robotState.position.x;
      const ry = robotState.position.y;
      const minHeight = getMinGroundHeight(rx, ry, robotState.mode);
      const rz = Math.max(robotState.position.z, minHeight);

      // Map ROS 2 coordinates (X: forward along cave, Y: lateral width, Z: altitude height)
      // to Three.js coordinates (X: rx, Y: rz, Z: ry)
      robotGroupRef.current.position.set(rx, rz, ry);
      robotGroupRef.current.rotation.set(
        (robotState.orientation.x * Math.PI) / 180,
        (robotState.orientation.y * Math.PI) / 180,
        (robotState.orientation.z * Math.PI) / 180
      );
    }
  }, [robotState.position, robotState.orientation, robotState.mode]);

  // Determine current section label
  const getCurrentSectionName = (x: number): string => {
    if (x < -5) return "Section 1: Dry Cave (Rugged Rock)";
    if (x <= 29) return "Section 2: Flooded Cave (Water Surface z=0)";
    return "Section 3: Air Pocket Shaft (Air Ceiling)";
  };

  return (
    <div className="relative w-full h-full min-h-[420px] bg-slate-950 rounded-xl overflow-hidden border border-slate-800 shadow-2xl flex flex-col">
      {/* Viewport Top Control Overlay */}
      <div className="absolute top-3 left-3 right-3 z-10 flex flex-wrap items-center justify-between gap-2 bg-slate-900/80 backdrop-blur-md pl-3 pr-[8px] py-2 rounded-lg border border-slate-700/60 text-xs font-mono text-slate-200">
        <div className="flex flex-wrap items-center gap-2">
          <span className="inline-flex items-center gap-1.5 px-2.5 py-1 rounded bg-slate-800 text-sky-400 font-semibold border border-slate-700">
            <Compass className="w-3.5 h-3.5 animate-spin" style={{ animationDuration: "8s" }} />
            Gazebo Harmonic
          </span>
          <span className="inline-flex items-center gap-1 px-2 py-0.5 rounded bg-amber-500/10 text-amber-300 font-semibold border border-amber-500/30 text-[11px]">
            Spot Chassis
          </span>
          <span className={`inline-flex items-center gap-1 px-2 py-0.5 rounded font-semibold border text-[11px] ${
            robotState.mode === "FLYING"
              ? "bg-emerald-500/15 text-emerald-300 border-emerald-500/40 animate-pulse"
              : "bg-sky-500/15 text-sky-300 border-sky-500/40"
          }`}>
            Drone: {robotState.mode === "FLYING" ? "AIRBORNE DETACHED" : "DOCKED ON BACK"}
          </span>
          <span className="text-slate-400 font-mono hidden lg:inline">
            Zone: <strong className="text-emerald-400">{getCurrentSectionName(robotState.position.x)}</strong>
          </span>
        </div>

        {/* View Mode & Toggles */}
        <div className="flex flex-wrap items-center gap-1.5">
          <button
            id="btn-cam-orbit"
            onClick={() => setActiveCameraMode("orbit")}
            className={`px-2.5 py-1 rounded flex items-center gap-1 transition ${
              activeCameraMode === "orbit"
                ? "bg-sky-600 text-white font-bold"
                : "bg-slate-800 text-slate-300 hover:bg-slate-700"
            }`}
            title="3D Orbit Camera (Enhanced Brightness)"
          >
            <Camera className="w-3.5 h-3.5" /> Orbit
          </button>
          <button
            id="btn-cam-fpv"
            onClick={() => setActiveCameraMode("fpv")}
            className={`px-2.5 py-1 rounded flex items-center gap-1 transition ${
              activeCameraMode === "fpv"
                ? "bg-sky-600 text-white font-bold"
                : "bg-slate-800 text-slate-300 hover:bg-slate-700"
            }`}
            title="First-Person Drone Nose Cam"
          >
            <Eye className="w-3.5 h-3.5" /> FPV
          </button>
          <button
            id="btn-cam-follow"
            onClick={() => setActiveCameraMode("follow")}
            className={`px-2.5 py-1 rounded flex items-center gap-1 transition ${
              activeCameraMode === "follow"
                ? "bg-sky-600 text-white font-bold shadow-lg shadow-sky-600/30"
                : "bg-slate-800 text-slate-300 hover:bg-slate-700"
            }`}
            title="Third-Person Drone Follow Cam (Chase View)"
          >
            <Navigation className="w-3.5 h-3.5 text-emerald-400" /> Follow
          </button>
          <button
            id="btn-cam-topdown"
            onClick={() => setActiveCameraMode("topdown")}
            className={`px-2.5 py-1 rounded flex items-center gap-1 transition ${
              activeCameraMode === "topdown"
                ? "bg-sky-600 text-white font-bold"
                : "bg-slate-800 text-slate-300 hover:bg-slate-700"
            }`}
            title="Top-down SLAM Map (Bright Fog-Free)"
          >
            <Layers className="w-3.5 h-3.5" /> Map
          </button>

          {/* Gazebo Viewport 3D Sim World Editor Toggle Button */}
          <button
            id="btn-toggle-editor"
            onClick={() => setIsEditorMode(!isEditorMode)}
            className={`px-2.5 py-1 rounded flex items-center gap-1.5 transition text-xs font-bold ${
              isEditorMode
                ? "bg-sky-500 text-slate-950 shadow-[0_0_12px_rgba(56,189,248,0.5)] border border-sky-300"
                : "bg-slate-800 text-sky-400 border border-slate-700 hover:bg-slate-700"
            }`}
            title="Toggle Gazebo 3D World Scene Editor (Spawn, Transform & Edit Props)"
          >
            <Edit3 className="w-3.5 h-3.5" />
            <span>Sim Editor</span>
            <span className="ml-0.5 px-1.5 py-0.2 rounded-full bg-slate-950 text-sky-300 text-[10px]">
              {spawnedObjects.length}
            </span>
          </button>

          <div className="h-4 w-px bg-slate-700 mx-1 hidden sm:block" />

          {/* Anti-Collision & Sensor Ray Toggles */}
          <button
            id="toggle-anticollision"
            onClick={() => setAntiCollisionEnabled && setAntiCollisionEnabled(!antiCollisionEnabled)}
            className={`px-2.5 py-1 rounded text-xs transition flex items-center gap-1 ${
              antiCollisionEnabled
                ? "bg-emerald-600/90 text-white font-bold shadow-[0_0_8px_rgba(16,185,129,0.3)]"
                : "bg-slate-800 text-slate-400 border border-slate-700"
            }`}
            title="Toggle Obstacle Anti-Collision & Autonomous Evasive Steering Guard"
          >
            <ShieldCheck className="w-3.5 h-3.5" />
            <span className="hidden md:inline">Anti-Collision:</span> {antiCollisionEnabled ? "ON" : "OFF"}
          </button>

          <button
            id="toggle-lidar-beams"
            onClick={() => setShowLaserBeams(!showLaserBeams)}
            className={`px-2 py-1 rounded text-xs transition ${
              showLaserBeams ? "bg-emerald-600/80 text-white font-semibold" : "bg-slate-800 text-slate-400"
            }`}
            title="Toggle 3D LiDAR Laser Scan Beams"
          >
            LiDAR Rays
          </button>

          {/* Underwater Sonar Toggle Button */}
          <button
            id="toggle-sonar-pulse"
            onClick={() => {
              const nextVal = !showSonarPulse;
              setShowSonarPulse(nextVal);
              setSensorData((prev) => ({ ...prev, sonarActive: nextVal }));
            }}
            className={`px-2 py-1 rounded text-xs transition flex items-center gap-1 ${
              showSonarPulse
                ? "bg-rose-600/90 text-white font-bold shadow-[0_0_8px_rgba(244,63,94,0.4)]"
                : "bg-slate-800 text-slate-400 border border-slate-700"
            }`}
            title="Toggle Underwater Bathymetric Sonar Scanning Pulse"
          >
            <Radio className="w-3 h-3 text-rose-300 animate-pulse" />
            <span>Sonar:</span> {showSonarPulse ? "ON" : "OFF"}
          </button>

          {/* WiFi Video Stream Status Badge */}
          <div
            className={`px-2 py-1 rounded text-xs transition flex items-center gap-1 border ${
              sensorData.wifiStreamingActive
                ? "bg-sky-500/15 text-sky-300 border-sky-500/40"
                : "bg-slate-800 text-slate-400 border-slate-700"
            }`}
            title="5.8GHz High-Bandwidth Wi-Fi Video Stream from Flying Drone to Base Companion Computer"
          >
            <span className="w-2 h-2 rounded-full bg-sky-400 animate-ping" />
            <span className="hidden sm:inline font-mono font-semibold">WiFi Stream:</span>
            <span className="text-emerald-400 font-mono font-bold">1080p 60fps</span>
          </div>

          <button
            id="toggle-headlight"
            onClick={() => setHeadlight(!headlight)}
            className={`px-2 py-1 rounded text-xs transition ${
              headlight ? "bg-amber-500 text-slate-950 font-bold" : "bg-slate-800 text-slate-400"
            }`}
            title="Toggle Subterranean Headlight Spotlight"
          >
            <Zap className="w-3 h-3 inline mr-1" /> Light
          </button>

          <div className="h-4 w-px bg-slate-700 mx-1 hidden sm:block" />

          {/* Record Video Button with Directory Picker */}
          <div className="flex items-center gap-1">
            <button
              id="btn-select-dir"
              onClick={async () => {
                const userPrompt = window.prompt("Target Video Download Folder / Directory Path:", downloadDir);
                if (userPrompt !== null && userPrompt.trim().length > 0) {
                  setDownloadDir(userPrompt.trim());
                }
              }}
              className="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-[11px] font-mono flex items-center gap-1 border border-slate-700"
              title="Click to Change Video Export Directory"
            >
              <Folder className="w-3 h-3 text-amber-400" />
              <span className="truncate max-w-[90px] hidden lg:inline">{downloadDir}</span>
            </button>
            <button
              id="btn-viewport-record"
              onClick={handleRecordToggle}
              className={`px-2.5 py-1 rounded text-xs font-bold transition flex items-center gap-1.5 ${
                isRecording
                  ? "bg-rose-600 hover:bg-rose-500 text-white shadow-[0_0_12px_rgba(239,68,68,0.6)] animate-pulse"
                  : "bg-rose-950/70 hover:bg-rose-900 text-rose-300 border border-rose-800/80"
              }`}
              title={isRecording ? "Stop Video Recording & Export to Selected Directory" : "Start Video Recording (Asks for Directory)"}
            >
              <Video className="w-3.5 h-3.5" />
              {isRecording ? `REC ${formatRecordTime(recordSeconds)}` : "Record Video"}
            </button>
          </div>
        </div>
      </div>

      {/* Main 3D Canvas Mount */}
      <div ref={mountRef} className="relative w-full h-full flex-1 cursor-grab active:cursor-grabbing">
        {/* Anti-Collision Active Evasion Alert Overlay */}
        {evasionAlert?.active && (
          <div className="absolute top-16 right-4 z-20 pointer-events-none flex items-center gap-2 bg-rose-950/90 backdrop-blur border border-rose-500/80 px-3.5 py-2 rounded-lg text-rose-200 font-mono text-xs shadow-2xl animate-pulse">
            <ShieldAlert className="w-4 h-4 text-rose-400" />
            <div>
              <strong className="text-rose-400">ANTI-COLLISION GUARD INTERVENTION</strong>
              <div className="text-[11px] text-rose-300/90">{evasionAlert.obstacle} - Executing Evasive Steering Vector</div>
            </div>
          </div>
        )}
        {/* WebGL Error Fallback Card */}
        {hasWebGLError && (
          <div className="absolute inset-0 z-30 bg-slate-950/95 flex flex-col items-center justify-center p-6 text-center font-mono">
            <div className="w-12 h-12 rounded-full bg-rose-500/20 border border-rose-500/40 text-rose-400 flex items-center justify-center mb-3">
              <Zap className="w-6 h-6 animate-pulse" />
            </div>
            <h4 className="text-sm font-bold text-slate-100 mb-1">WebGL Graphics Context Reset</h4>
            <p className="text-xs text-slate-400 max-w-md mb-4 leading-relaxed">
              The 3D WebGL hardware graphics context was reset or constrained by the browser environment. Click below to re-initialize the Gazebo 3D Cave Visualizer.
            </p>
            <button
              id="btn-restore-webgl"
              onClick={() => setHasWebGLError(false)}
              className="px-4 py-2 bg-emerald-500 hover:bg-emerald-400 text-slate-950 font-bold rounded-lg transition shadow-lg text-xs flex items-center gap-2"
            >
              <RotateCcw className="w-4 h-4" />
              Re-initialize 3D Viewport
            </button>
          </div>
        )}
        {/* FPV Mode Bright Reticle Overlay */}
        {activeCameraMode === "fpv" && (
          <div className="absolute inset-0 pointer-events-none z-10 flex flex-col justify-between p-6 font-mono text-xs">
            <div className="flex items-start justify-between">
              <div className="bg-slate-950/80 backdrop-blur px-2.5 py-1.5 rounded border border-emerald-500/40 text-emerald-400 text-[11px] font-bold shadow-[0_0_10px_rgba(52,211,153,0.2)]">
                FPV NOSE-CAM • EXPOSURE +2.0 EV [HIGH GAIN BRIGHT]
              </div>
              <div className="bg-slate-950/80 backdrop-blur px-2.5 py-1 rounded border border-slate-700 text-sky-400 text-[10px]">
                ISO 3200 • 60 FPS • FL 18mm
              </div>
            </div>

            {/* FPV Crosshair */}
            <div className="self-center my-auto flex items-center justify-center opacity-70">
              <div className="w-12 h-12 border border-emerald-400/60 rounded-full flex items-center justify-center relative">
                <div className="w-2 h-2 bg-emerald-400 rounded-full"></div>
                <div className="absolute w-16 h-px bg-emerald-400/40"></div>
                <div className="absolute h-16 w-px bg-emerald-400/40"></div>
              </div>
            </div>
          </div>
        )}

        {/* Video Recording Live Overlay Badge */}
        {isRecording && (
          <div className="absolute top-16 left-4 z-20 pointer-events-none flex items-center gap-2 bg-rose-950/90 backdrop-blur border border-rose-500/60 px-3 py-1.5 rounded-lg text-rose-300 font-mono text-xs shadow-lg animate-pulse">
            <Circle className="w-3 h-3 fill-rose-500 text-rose-500" />
            <strong>RECORDING VIDEO STREAM: {formatRecordTime(recordSeconds)}</strong>
          </div>
        )}
      </div>

      {/* Floating Gazebo Simulation World Editor Panel */}
      {isEditorMode && (
        <div className="absolute top-14 right-3 bottom-14 w-80 md:w-96 z-30 bg-slate-900/95 backdrop-blur-xl border border-sky-500/40 rounded-xl shadow-[0_0_25px_rgba(0,0,0,0.8)] flex flex-col font-mono text-xs overflow-hidden text-slate-200">
          {/* Panel Header */}
          <div className="flex items-center justify-between px-3.5 py-2.5 bg-slate-950/80 border-b border-slate-800">
            <div className="flex items-center gap-2">
              <div className="w-6 h-6 rounded bg-sky-500/20 border border-sky-400/50 flex items-center justify-center text-sky-400">
                <Edit3 className="w-3.5 h-3.5" />
              </div>
              <div>
                <h3 className="font-bold text-slate-100 text-xs flex items-center gap-1.5">
                  Gazebo World Editor
                  <span className="px-1.5 py-0.2 rounded bg-sky-500/20 text-sky-300 text-[10px]">
                    {spawnedObjects.length} Objects
                  </span>
                </h3>
                <p className="text-[10px] text-slate-400">3D Simulation Viewport Editing Facilities</p>
              </div>
            </div>
            <button
              onClick={() => setIsEditorMode(false)}
              className="p-1 rounded hover:bg-slate-800 text-slate-400 hover:text-slate-100 transition"
              title="Close Editor Panel"
            >
              <X className="w-4 h-4" />
            </button>
          </div>

          {/* Editor Tabs Navigation */}
          <div className="grid grid-cols-3 bg-slate-950 border-b border-slate-800 p-1 gap-1">
            <button
              onClick={() => setEditorTab("objects")}
              className={`py-1.5 rounded text-[11px] font-bold flex items-center justify-center gap-1 transition ${
                editorTab === "objects"
                  ? "bg-sky-600 text-white shadow-md shadow-sky-600/30"
                  : "bg-slate-900 text-slate-400 hover:bg-slate-800 hover:text-slate-200"
              }`}
            >
              <Box className="w-3.5 h-3.5" />
              <span>Objects ({spawnedObjects.length})</span>
            </button>
            <button
              onClick={() => setEditorTab("spawn")}
              className={`py-1.5 rounded text-[11px] font-bold flex items-center justify-center gap-1 transition ${
                editorTab === "spawn"
                  ? "bg-sky-600 text-white shadow-md shadow-sky-600/30"
                  : "bg-slate-900 text-slate-400 hover:bg-slate-800 hover:text-slate-200"
              }`}
            >
              <Plus className="w-3.5 h-3.5 text-emerald-400" />
              <span>+ Spawn</span>
            </button>
            <button
              onClick={() => setEditorTab("environment")}
              className={`py-1.5 rounded text-[11px] font-bold flex items-center justify-center gap-1 transition ${
                editorTab === "environment"
                  ? "bg-sky-600 text-white shadow-md shadow-sky-600/30"
                  : "bg-slate-900 text-slate-400 hover:bg-slate-800 hover:text-slate-200"
              }`}
            >
              <Settings className="w-3.5 h-3.5 text-amber-400" />
              <span>Environment</span>
            </button>
          </div>

          {/* Editor Tab Content */}
          <div className="flex-1 overflow-y-auto p-3 space-y-3 custom-scrollbar">
            {/* TAB 1: OBJECTS OUTLINER & INSPECTOR */}
            {editorTab === "objects" && (
              <div className="space-y-3">
                {/* Outliner List */}
                <div className="bg-slate-950/70 p-2 rounded-lg border border-slate-800 space-y-1">
                  <div className="text-[10px] uppercase font-bold text-slate-400 px-1 mb-1 flex justify-between items-center">
                    <span>Scene Object Outliner</span>
                    <span className="text-slate-500">Click to Select</span>
                  </div>
                  {spawnedObjects.length === 0 ? (
                    <div className="text-center py-4 text-slate-500 text-[11px]">
                      No custom objects spawned yet. Switch to <strong>+ Spawn</strong> tab to add props!
                    </div>
                  ) : (
                    spawnedObjects.map((obj) => {
                      const isSel = obj.id === selectedObjectId;
                      return (
                        <div
                          key={obj.id}
                          onClick={() => setSelectedObjectId(obj.id)}
                          className={`flex items-center justify-between p-1.5 rounded cursor-pointer transition ${
                            isSel
                              ? "bg-sky-950/80 border border-sky-500/60 text-sky-200 font-bold"
                              : "bg-slate-900/80 border border-slate-800 hover:bg-slate-800 text-slate-300"
                          }`}
                        >
                          <div className="flex items-center gap-2 truncate">
                            <span
                              className="w-2.5 h-2.5 rounded-full shrink-0"
                              style={{ backgroundColor: obj.color }}
                            />
                            <span className="truncate text-[11px]">{obj.name}</span>
                          </div>
                          <div className="flex items-center gap-1 shrink-0">
                            <button
                              onClick={(e) => {
                                e.stopPropagation();
                                setSpawnedObjects((prev) =>
                                  prev.map((o) => (o.id === obj.id ? { ...o, visible: !o.visible } : o))
                                );
                              }}
                              className="p-1 hover:bg-slate-800 rounded text-slate-400 hover:text-slate-200"
                              title={obj.visible ? "Hide Object" : "Show Object"}
                            >
                              {obj.visible ? <Eye className="w-3 h-3 text-emerald-400" /> : <EyeOff className="w-3 h-3 text-slate-500" />}
                            </button>
                            <button
                              onClick={(e) => {
                                e.stopPropagation();
                                duplicateObject(obj.id);
                              }}
                              className="p-1 hover:bg-slate-800 rounded text-slate-400 hover:text-amber-300"
                              title="Duplicate Object"
                            >
                              <Copy className="w-3 h-3" />
                            </button>
                            <button
                              onClick={(e) => {
                                e.stopPropagation();
                                deleteObject(obj.id);
                              }}
                              className="p-1 hover:bg-slate-800 rounded text-rose-400 hover:text-rose-300"
                              title="Delete Object"
                            >
                              <Trash2 className="w-3 h-3" />
                            </button>
                          </div>
                        </div>
                      );
                    })
                  )}
                </div>

                {/* Selected Object Inspector */}
                {selectedObjectId && (
                  (() => {
                    const activeObj = spawnedObjects.find((o) => o.id === selectedObjectId);
                    if (!activeObj) return null;
                    return (
                      <div className="bg-slate-950/80 p-3 rounded-lg border border-sky-500/30 space-y-2.5">
                        <div className="flex items-center justify-between border-b border-slate-800 pb-1.5">
                          <span className="font-bold text-sky-400 text-[11px] uppercase flex items-center gap-1">
                            <Sliders className="w-3.5 h-3.5" /> Object Transform Inspector
                          </span>
                          <span className="px-1.5 py-0.2 rounded bg-slate-800 text-slate-300 text-[10px] uppercase font-mono">
                            {activeObj.type}
                          </span>
                        </div>

                        {/* Name Input */}
                        <div>
                          <label className="text-[10px] text-slate-400 block mb-1">Object Name Label:</label>
                          <input
                            type="text"
                            value={activeObj.name}
                            onChange={(e) =>
                              updateSelectedObject((o) => ({ ...o, name: e.target.value }))
                            }
                            className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200 text-xs font-mono focus:border-sky-500 focus:outline-none"
                          />
                        </div>

                        {/* Position Controls (X, Y, Z) */}
                        <div className="space-y-1.5">
                          <div className="flex justify-between items-center text-[10px] text-slate-400">
                            <span>Position (X: Forward, Y: Height, Z: Lateral):</span>
                            <button
                              onClick={() => snapToRobot(activeObj.id)}
                              className="text-sky-400 hover:underline flex items-center gap-1 text-[10px]"
                              title="Move object right in front of current robot location"
                            >
                              <Target className="w-3 h-3" /> Snap to Robot
                            </button>
                          </div>

                          {/* Position X */}
                          <div className="flex items-center gap-2 text-[11px]">
                            <span className="w-4 font-bold text-sky-400">X:</span>
                            <input
                              type="range"
                              min="-35"
                              max="35"
                              step="0.2"
                              value={activeObj.position.x}
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, x: val },
                                }));
                              }}
                              className="flex-1 accent-sky-500 h-1.5 bg-slate-800 rounded cursor-pointer"
                            />
                            <input
                              type="number"
                              step="0.1"
                              value={activeObj.position.x}
                              onChange={(e) => {
                                const val = parseFloat(e.target.value) || 0;
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, x: val },
                                }));
                              }}
                              className="w-14 bg-slate-900 border border-slate-700 rounded px-1 py-0.5 text-center text-slate-200 text-[11px]"
                            />
                          </div>

                          {/* Position Y */}
                          <div className="flex items-center gap-2 text-[11px]">
                            <span className="w-4 font-bold text-emerald-400">Y:</span>
                            <input
                              type="range"
                              min="-3"
                              max="8"
                              step="0.1"
                              value={activeObj.position.y}
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, y: val },
                                }));
                              }}
                              className="flex-1 accent-emerald-500 h-1.5 bg-slate-800 rounded cursor-pointer"
                            />
                            <input
                              type="number"
                              step="0.1"
                              value={activeObj.position.y}
                              onChange={(e) => {
                                const val = parseFloat(e.target.value) || 0;
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, y: val },
                                }));
                              }}
                              className="w-14 bg-slate-900 border border-slate-700 rounded px-1 py-0.5 text-center text-slate-200 text-[11px]"
                            />
                          </div>

                          {/* Position Z */}
                          <div className="flex items-center gap-2 text-[11px]">
                            <span className="w-4 font-bold text-purple-400">Z:</span>
                            <input
                              type="range"
                              min="-5"
                              max="5"
                              step="0.1"
                              value={activeObj.position.z}
                              onChange={(e) => {
                                const val = parseFloat(e.target.value);
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, z: val },
                                }));
                              }}
                              className="flex-1 accent-purple-500 h-1.5 bg-slate-800 rounded cursor-pointer"
                            />
                            <input
                              type="number"
                              step="0.1"
                              value={activeObj.position.z}
                              onChange={(e) => {
                                const val = parseFloat(e.target.value) || 0;
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, z: val },
                                }));
                              }}
                              className="w-14 bg-slate-900 border border-slate-700 rounded px-1 py-0.5 text-center text-slate-200 text-[11px]"
                            />
                          </div>
                        </div>

                        {/* Directional Nudge Pad */}
                        <div className="bg-slate-900/60 p-2 rounded border border-slate-800 flex items-center justify-between">
                          <span className="text-[10px] text-slate-400">Position Nudge:</span>
                          <div className="flex items-center gap-1">
                            <button
                              onClick={() =>
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, x: Number((o.position.x - 0.5).toFixed(2)) },
                                }))
                              }
                              className="px-1.5 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-[10px] border border-slate-700"
                              title="Nudge Back (X -0.5m)"
                            >
                              ← X
                            </button>
                            <button
                              onClick={() =>
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, x: Number((o.position.x + 0.5).toFixed(2)) },
                                }))
                              }
                              className="px-1.5 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-[10px] border border-slate-700"
                              title="Nudge Forward (X +0.5m)"
                            >
                              X →
                            </button>
                            <button
                              onClick={() =>
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, z: Number((o.position.z - 0.5).toFixed(2)) },
                                }))
                              }
                              className="px-1.5 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-[10px] border border-slate-700"
                              title="Nudge Left (Z -0.5m)"
                            >
                              ↑ Z
                            </button>
                            <button
                              onClick={() =>
                                updateSelectedObject((o) => ({
                                  ...o,
                                  position: { ...o.position, z: Number((o.position.z + 0.5).toFixed(2)) },
                                }))
                              }
                              className="px-1.5 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-[10px] border border-slate-700"
                              title="Nudge Right (Z +0.5m)"
                            >
                              ↓ Z
                            </button>
                          </div>
                        </div>

                        {/* Rotation & Scale */}
                        <div className="grid grid-cols-2 gap-2">
                          <div>
                            <label className="text-[10px] text-slate-400 block mb-0.5">Yaw Rotation:</label>
                            <div className="flex items-center gap-1">
                              <input
                                type="range"
                                min="0"
                                max="360"
                                step="15"
                                value={activeObj.rotation.y}
                                onChange={(e) => {
                                  const val = parseInt(e.target.value);
                                  updateSelectedObject((o) => ({
                                    ...o,
                                    rotation: { ...o.rotation, y: val },
                                  }));
                                }}
                                className="flex-1 accent-amber-500 h-1.5 bg-slate-800 rounded cursor-pointer"
                              />
                              <span className="text-[10px] text-amber-300 font-bold w-8 text-right">
                                {activeObj.rotation.y}°
                              </span>
                            </div>
                          </div>

                          <div>
                            <label className="text-[10px] text-slate-400 block mb-0.5">Scale Multiplier:</label>
                            <div className="flex items-center gap-1">
                              <input
                                type="range"
                                min="0.3"
                                max="3.0"
                                step="0.1"
                                value={activeObj.scale.x}
                                onChange={(e) => {
                                  const val = parseFloat(e.target.value);
                                  updateSelectedObject((o) => ({
                                    ...o,
                                    scale: { x: val, y: val, z: val },
                                  }));
                                }}
                                className="flex-1 accent-sky-500 h-1.5 bg-slate-800 rounded cursor-pointer"
                              />
                              <span className="text-[10px] text-sky-300 font-bold w-8 text-right">
                                {activeObj.scale.x.toFixed(1)}x
                              </span>
                            </div>
                          </div>
                        </div>

                        {/* Color Palette Swatches */}
                        <div>
                          <label className="text-[10px] text-slate-400 block mb-1">Color Palette Swatches:</label>
                          <div className="flex items-center gap-1.5">
                            {["#475569", "#334155", "#f97316", "#06b6d4", "#fbbf24", "#10b981", "#ef4444", "#f8fafc"].map(
                              (hex) => (
                                <button
                                  key={hex}
                                  onClick={() =>
                                    updateSelectedObject((o) => ({ ...o, color: hex }))
                                  }
                                  className={`w-5 h-5 rounded-full border transition ${
                                    activeObj.color === hex
                                      ? "border-white scale-110 shadow-md shadow-sky-400/50"
                                      : "border-slate-700 opacity-80 hover:opacity-100"
                                  }`}
                                  style={{ backgroundColor: hex }}
                                />
                              )
                            )}
                          </div>
                        </div>

                        {/* Intensity Slider if applicable */}
                        {(activeObj.type === "spotlight" || activeObj.type === "buoy") && (
                          <div>
                            <label className="text-[10px] text-slate-400 block mb-0.5">Beacon Light Intensity:</label>
                            <div className="flex items-center gap-2">
                              <input
                                type="range"
                                min="0.5"
                                max="8.0"
                                step="0.5"
                                value={activeObj.intensity || 3.0}
                                onChange={(e) => {
                                  const val = parseFloat(e.target.value);
                                  updateSelectedObject((o) => ({ ...o, intensity: val }));
                                }}
                                className="flex-1 accent-amber-400 h-1.5 bg-slate-800 rounded cursor-pointer"
                              />
                              <span className="text-[10px] text-amber-300 font-bold">
                                {(activeObj.intensity || 3.0).toFixed(1)} Lux
                              </span>
                            </div>
                          </div>
                        )}
                      </div>
                    );
                  })()
                )}
              </div>
            )}

            {/* TAB 2: SPAWN PROPS */}
            {editorTab === "spawn" && (
              <div className="space-y-2.5">
                <div className="text-[10px] text-slate-400">
                  Select a 3D simulation prop preset to add into the Gazebo cave world:
                </div>

                <div className="grid grid-cols-2 gap-2">
                  {/* Boulder */}
                  <div className="p-2.5 rounded-lg bg-slate-950/80 border border-slate-800 hover:border-slate-700 flex flex-col justify-between space-y-2">
                    <div className="flex items-center gap-2">
                      <div className="w-7 h-7 rounded bg-slate-800 flex items-center justify-center text-slate-300">
                        <Box className="w-4 h-4" />
                      </div>
                      <div>
                        <div className="font-bold text-slate-200 text-[11px]">Rugged Boulder</div>
                        <div className="text-[9px] text-slate-400">Rock Hazard</div>
                      </div>
                    </div>
                    <div className="flex items-center gap-1">
                      <button
                        onClick={() => spawnNewObject("boulder")}
                        className="flex-1 py-1 rounded bg-slate-800 hover:bg-slate-700 text-sky-300 text-[10px] font-bold"
                      >
                        Spawn Nose
                      </button>
                      <button
                        onClick={() => setClickToPlaceType("boulder")}
                        className="p-1 rounded bg-sky-600/80 hover:bg-sky-500 text-white"
                        title="Click on 3D Viewport to place"
                      >
                        <Crosshair className="w-3.5 h-3.5" />
                      </button>
                    </div>
                  </div>

                  {/* Stalagmite Column */}
                  <div className="p-2.5 rounded-lg bg-slate-950/80 border border-slate-800 hover:border-slate-700 flex flex-col justify-between space-y-2">
                    <div className="flex items-center gap-2">
                      <div className="w-7 h-7 rounded bg-slate-800 flex items-center justify-center text-slate-300">
                        <ArrowUp className="w-4 h-4 text-slate-400" />
                      </div>
                      <div>
                        <div className="font-bold text-slate-200 text-[11px]">Cave Pillar</div>
                        <div className="text-[9px] text-slate-400">Vertical Rock</div>
                      </div>
                    </div>
                    <div className="flex items-center gap-1">
                      <button
                        onClick={() => spawnNewObject("stalagmite")}
                        className="flex-1 py-1 rounded bg-slate-800 hover:bg-slate-700 text-sky-300 text-[10px] font-bold"
                      >
                        Spawn Nose
                      </button>
                      <button
                        onClick={() => setClickToPlaceType("stalagmite")}
                        className="p-1 rounded bg-sky-600/80 hover:bg-sky-500 text-white"
                        title="Click on 3D Viewport to place"
                      >
                        <Crosshair className="w-3.5 h-3.5" />
                      </button>
                    </div>
                  </div>

                  {/* Acoustic Buoy */}
                  <div className="p-2.5 rounded-lg bg-slate-950/80 border border-slate-800 hover:border-slate-700 flex flex-col justify-between space-y-2">
                    <div className="flex items-center gap-2">
                      <div className="w-7 h-7 rounded bg-amber-500/20 flex items-center justify-center text-amber-400">
                        <Radio className="w-4 h-4" />
                      </div>
                      <div>
                        <div className="font-bold text-slate-200 text-[11px]">Acoustic Buoy</div>
                        <div className="text-[9px] text-slate-400">Floats on Water</div>
                      </div>
                    </div>
                    <div className="flex items-center gap-1">
                      <button
                        onClick={() => spawnNewObject("buoy")}
                        className="flex-1 py-1 rounded bg-slate-800 hover:bg-slate-700 text-amber-300 text-[10px] font-bold"
                      >
                        Spawn Nose
                      </button>
                      <button
                        onClick={() => setClickToPlaceType("buoy")}
                        className="p-1 rounded bg-amber-600/80 hover:bg-amber-500 text-white"
                        title="Click on 3D Viewport to place"
                      >
                        <Crosshair className="w-3.5 h-3.5" />
                      </button>
                    </div>
                  </div>

                  {/* Inspection Target */}
                  <div className="p-2.5 rounded-lg bg-slate-950/80 border border-slate-800 hover:border-slate-700 flex flex-col justify-between space-y-2">
                    <div className="flex items-center gap-2">
                      <div className="w-7 h-7 rounded bg-cyan-500/20 flex items-center justify-center text-cyan-400">
                        <Target className="w-4 h-4" />
                      </div>
                      <div>
                        <div className="font-bold text-slate-200 text-[11px]">Optical Target</div>
                        <div className="text-[9px] text-slate-400">ARUCO Grid</div>
                      </div>
                    </div>
                    <div className="flex items-center gap-1">
                      <button
                        onClick={() => spawnNewObject("target")}
                        className="flex-1 py-1 rounded bg-slate-800 hover:bg-slate-700 text-cyan-300 text-[10px] font-bold"
                      >
                        Spawn Nose
                      </button>
                      <button
                        onClick={() => setClickToPlaceType("target")}
                        className="p-1 rounded bg-cyan-600/80 hover:bg-cyan-500 text-white"
                        title="Click on 3D Viewport to place"
                      >
                        <Crosshair className="w-3.5 h-3.5" />
                      </button>
                    </div>
                  </div>

                  {/* Lighting Tower */}
                  <div className="p-2.5 rounded-lg bg-slate-950/80 border border-slate-800 hover:border-slate-700 flex flex-col justify-between space-y-2">
                    <div className="flex items-center gap-2">
                      <div className="w-7 h-7 rounded bg-yellow-500/20 flex items-center justify-center text-yellow-400">
                        <Lightbulb className="w-4 h-4" />
                      </div>
                      <div>
                        <div className="font-bold text-slate-200 text-[11px]">Spotlight Tower</div>
                        <div className="text-[9px] text-slate-400">Subsea Lighting</div>
                      </div>
                    </div>
                    <div className="flex items-center gap-1">
                      <button
                        onClick={() => spawnNewObject("spotlight")}
                        className="flex-1 py-1 rounded bg-slate-800 hover:bg-slate-700 text-yellow-300 text-[10px] font-bold"
                      >
                        Spawn Nose
                      </button>
                      <button
                        onClick={() => setClickToPlaceType("spotlight")}
                        className="p-1 rounded bg-yellow-600/80 hover:bg-yellow-500 text-white"
                        title="Click on 3D Viewport to place"
                      >
                        <Crosshair className="w-3.5 h-3.5" />
                      </button>
                    </div>
                  </div>

                  {/* Waypoint Marker */}
                  <div className="p-2.5 rounded-lg bg-slate-950/80 border border-slate-800 hover:border-slate-700 flex flex-col justify-between space-y-2">
                    <div className="flex items-center gap-2">
                      <div className="w-7 h-7 rounded bg-emerald-500/20 flex items-center justify-center text-emerald-400">
                        <MapPin className="w-4 h-4" />
                      </div>
                      <div>
                        <div className="font-bold text-slate-200 text-[11px]">Waypoint Marker</div>
                        <div className="text-[9px] text-slate-400">Transponder Ring</div>
                      </div>
                    </div>
                    <div className="flex items-center gap-1">
                      <button
                        onClick={() => spawnNewObject("waypoint")}
                        className="flex-1 py-1 rounded bg-slate-800 hover:bg-slate-700 text-emerald-300 text-[10px] font-bold"
                      >
                        Spawn Nose
                      </button>
                      <button
                        onClick={() => setClickToPlaceType("waypoint")}
                        className="p-1 rounded bg-emerald-600/80 hover:bg-emerald-500 text-white"
                        title="Click on 3D Viewport to place"
                      >
                        <Crosshair className="w-3.5 h-3.5" />
                      </button>
                    </div>
                  </div>
                </div>
              </div>
            )}

            {/* TAB 3: ENVIRONMENT & PHYSICS */}
            {editorTab === "environment" && (
              <div className="space-y-3">
                <div className="bg-slate-950/80 p-3 rounded-lg border border-slate-800 space-y-2.5">
                  <div className="font-bold text-amber-400 text-[11px] uppercase border-b border-slate-800 pb-1 flex items-center gap-1">
                    <Sun className="w-3.5 h-3.5" /> Ambient Cave Lighting
                  </div>
                  <div className="flex items-center gap-2">
                    <input
                      type="range"
                      min="0.5"
                      max="8.0"
                      step="0.5"
                      value={envLightIntensity}
                      onChange={(e) => setEnvLightIntensity(parseFloat(e.target.value))}
                      className="flex-1 accent-amber-400 h-1.5 bg-slate-800 rounded cursor-pointer"
                    />
                    <span className="text-[11px] text-amber-300 font-bold w-12 text-right">
                      {envLightIntensity.toFixed(1)} Lux
                    </span>
                  </div>
                </div>

                <div className="bg-slate-950/80 p-3 rounded-lg border border-slate-800 space-y-2.5">
                  <div className="font-bold text-sky-400 text-[11px] uppercase border-b border-slate-800 pb-1 flex items-center gap-1">
                    <Waves className="w-3.5 h-3.5" /> Water Fog & Turbidity
                  </div>
                  <div>
                    <label className="text-[10px] text-slate-400 block mb-0.5">Water Fog Density:</label>
                    <div className="flex items-center gap-2">
                      <input
                        type="range"
                        min="0.01"
                        max="0.12"
                        step="0.005"
                        value={waterFogDensity}
                        onChange={(e) => setWaterFogDensity(parseFloat(e.target.value))}
                        className="flex-1 accent-sky-400 h-1.5 bg-slate-800 rounded cursor-pointer"
                      />
                      <span className="text-[11px] text-sky-300 font-bold w-12 text-right">
                        {waterFogDensity.toFixed(3)}
                      </span>
                    </div>
                  </div>

                  <div>
                    <label className="text-[10px] text-slate-400 block mb-1">Water Color Preset:</label>
                    <div className="grid grid-cols-2 gap-1.5">
                      {[
                        { name: "Deep Cyan", hex: "#0f2b48" },
                        { name: "Emerald Subsea", hex: "#064e3b" },
                        { name: "Muddy Cave", hex: "#451a03" },
                        { name: "Crystal Blue", hex: "#0284c7" },
                      ].map((preset) => (
                        <button
                          key={preset.hex}
                          onClick={() => setWaterColor(preset.hex)}
                          className={`py-1 px-1.5 rounded text-[10px] font-mono border transition truncate ${
                            waterColor === preset.hex
                              ? "bg-slate-800 border-sky-400 text-sky-300 font-bold"
                              : "bg-slate-900 border-slate-700 text-slate-400 hover:text-slate-200"
                          }`}
                        >
                          {preset.name}
                        </button>
                      ))}
                    </div>
                  </div>
                </div>

                <div className="bg-slate-950/80 p-3 rounded-lg border border-slate-800 space-y-2.5">
                  <div className="font-bold text-emerald-400 text-[11px] uppercase border-b border-slate-800 pb-1 flex items-center gap-1">
                    <ShieldCheck className="w-3.5 h-3.5" /> Anti-Collision Safety Buffer
                  </div>
                  <div className="flex items-center gap-2">
                    <input
                      type="range"
                      min="0.8"
                      max="4.0"
                      step="0.2"
                      value={antiCollisionRadius}
                      onChange={(e) => setAntiCollisionRadius(parseFloat(e.target.value))}
                      className="flex-1 accent-emerald-400 h-1.5 bg-slate-800 rounded cursor-pointer"
                    />
                    <span className="text-[11px] text-emerald-300 font-bold w-12 text-right">
                      {antiCollisionRadius.toFixed(1)} m
                    </span>
                  </div>
                </div>

                <button
                  onClick={() => {
                    if (window.confirm("Are you sure you want to clear all custom spawned objects?")) {
                      setSpawnedObjects([]);
                      setSelectedObjectId(null);
                    }
                  }}
                  className="w-full py-2 bg-rose-950/80 hover:bg-rose-900 text-rose-300 border border-rose-800 rounded-lg text-xs font-bold transition flex items-center justify-center gap-1.5"
                >
                  <Trash2 className="w-3.5 h-3.5" />
                  Clear All Custom Objects
                </button>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Click-To-Place Active Viewport Top Banner */}
      {clickToPlaceType && (
        <div className="absolute top-16 left-1/2 -translate-x-1/2 z-30 bg-sky-950/95 border border-sky-400 text-sky-200 px-4 py-2 rounded-full shadow-2xl flex items-center gap-3 font-mono text-xs animate-bounce">
          <Crosshair className="w-4 h-4 text-sky-300 animate-spin" style={{ animationDuration: "4s" }} />
          <span>
            <strong>CLICK-TO-PLACE ACTIVE:</strong> Click anywhere in the 3D viewport canvas to place a <strong>{clickToPlaceType}</strong>
          </span>
          <button
            onClick={() => setClickToPlaceType(null)}
            className="px-2 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-[11px]"
          >
            Cancel
          </button>
        </div>
      )}

      {/* Bottom Telemetry HUD Bar */}
      <div className="absolute bottom-3 left-3 right-3 z-10 bg-slate-900/85 backdrop-blur-md px-4 py-2.5 rounded-lg border border-slate-700/70 text-xs font-mono text-slate-200 flex flex-wrap items-center justify-between gap-3">
        <div className="flex items-center gap-4">
          <div>
            <span className="text-slate-400">Position (x, y, z):</span>{" "}
            <strong className="text-sky-300">
              [{(robotState?.position?.x ?? 0).toFixed(2)}, {(robotState?.position?.y ?? 0).toFixed(2)}, {(robotState?.position?.z ?? 0).toFixed(2)}]m
            </strong>
          </div>
          <div>
            <span className="text-slate-400">Mode:</span>{" "}
            <span
              className={`px-2 py-0.5 rounded font-bold ${
                robotState?.mode === "WALKING"
                  ? "bg-amber-500/20 text-amber-400 border border-amber-500/30"
                  : robotState?.mode === "SAILING"
                  ? "bg-cyan-500/20 text-cyan-400 border border-cyan-500/30"
                  : "bg-purple-500/20 text-purple-400 border border-purple-500/30"
              }`}
            >
              {robotState?.mode ?? "WALKING"}
            </span>
          </div>
        </div>

        <div className="flex items-center gap-4 text-slate-300">
          <div>
            <span className="text-slate-400">Buoyancy:</span>{" "}
            <span className="text-emerald-400 font-semibold">{(robotState?.buoyancyForce ?? 0).toFixed(1)} N</span>
          </div>
          <div>
            <span className="text-slate-400">Sonar Depth:</span>{" "}
            <span className="text-rose-400 font-semibold">{(sensorData?.sonarDepth ?? 0).toFixed(2)} m</span>
          </div>
          <div>
            <span className="text-slate-400">Rotor RPM:</span>{" "}
            <span className="text-sky-400 font-semibold">{robotState?.propellerRpm ?? 0}</span>
          </div>
          <div>
            <span className="text-slate-400">Battery:</span>{" "}
            <span className="text-amber-400 font-semibold">{(robotState?.battery ?? 0).toFixed(0)}%</span>
          </div>
        </div>
      </div>
    </div>
  );
};
