import React, { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { RobotState, LocomotionMode, EnvironmentSection, SensorData, CameraMode } from "../types";
import { Camera, Eye, Layers, Maximize2, RotateCcw, Zap, Compass, Navigation, Video, Circle, Sun, ShieldCheck, ShieldAlert, Folder, Download } from "lucide-react";

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
  const legsRef = useRef<THREE.Group[]>([]);
  const pontoonsRef = useRef<THREE.Group[]>([]);
  const propellersRef = useRef<THREE.Mesh[]>([]);
  const lidarMeshRef = useRef<THREE.Group | null>(null);
  const laserRaysRef = useRef<THREE.LineSegments | null>(null);
  const sonarPulseMeshRef = useRef<THREE.Mesh | null>(null);
  const waterMeshRef = useRef<THREE.Mesh | null>(null);
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

    // SECTION 1: DRY CAVE (x: -22 to -5, z > 0)
    const dryGroundGeo = new THREE.PlaneGeometry(17, 12, 32, 24);
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
    dryGround.position.set(-13.5, 0, 0);
    dryGround.receiveShadow = true;
    caveGroup.add(dryGround);

    // Stalagmites on dry section
    const stalagmiteMat = new THREE.MeshStandardMaterial({ color: 0x3d3935, roughness: 0.8 });
    for (let i = 0; i < 12; i++) {
      const height = 0.8 + Math.random() * 1.8;
      const coneGeo = new THREE.ConeGeometry(0.3 + Math.random() * 0.3, height, 6);
      const cone = new THREE.Mesh(coneGeo, stalagmiteMat);
      cone.position.set(-20 + Math.random() * 13, height / 2, -5 + Math.random() * 10);
      caveGroup.add(cone);
    }

    // SECTION 2: FLOODED WATER SECTION (x: -5 to 12, z=0 surface, seabed at z=-3.5)
    // Water Surface Plane (z = 0)
    const waterGeo = new THREE.PlaneGeometry(17, 12, 32, 24);
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
    waterMesh.position.set(3.5, 0, 0); // z = 0 water plane
    waterMeshRef.current = waterMesh;
    caveGroup.add(waterMesh);

    // Underwater Seabed Floor (z = -3.5)
    const seabedGeo = new THREE.PlaneGeometry(17, 12, 20, 15);
    const seabedMat = new THREE.MeshStandardMaterial({ color: 0x121c24, roughness: 0.9 });
    const seabedMesh = new THREE.Mesh(seabedGeo, seabedMat);
    seabedMesh.rotation.x = -Math.PI / 2;
    seabedMesh.position.set(3.5, -3.5, 0);
    caveGroup.add(seabedMesh);

    // Submerged rock formations
    const rockMat = new THREE.MeshStandardMaterial({ color: 0x1a2630, roughness: 0.9 });
    for (let i = 0; i < 8; i++) {
      const rockGeo = new THREE.DodecahedronGeometry(0.5 + Math.random() * 0.7);
      const rock = new THREE.Mesh(rockGeo, rockMat);
      rock.position.set(-3 + Math.random() * 13, -2.8, -4 + Math.random() * 8);
      caveGroup.add(rock);
    }

    // SECTION 3: AIR POCKET CAVE (x: 12 to 24, z = 0 to 6)
    // Elevated Dry Ledge
    const ledgeGeo = new THREE.BoxGeometry(7, 2.5, 10);
    const ledgeMat = new THREE.MeshStandardMaterial({ color: 0x362d26, roughness: 0.85 });
    const ledge = new THREE.Mesh(ledgeGeo, ledgeMat);
    ledge.position.set(18.5, 1.25, 0);
    ledge.receiveShadow = true;
    caveGroup.add(ledge);

    // Cave Walls & Cavern Ceiling
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x1c1a17, roughness: 0.95, side: THREE.DoubleSide });

    // Cavern Ceiling with stalactites hanging
    const ceilingGeo = new THREE.PlaneGeometry(45, 14, 20, 10);
    const ceiling = new THREE.Mesh(ceilingGeo, wallMat);
    ceiling.rotation.x = Math.PI / 2;
    ceiling.position.set(0, 5.5, 0);
    caveGroup.add(ceiling);

    // Stalactites hanging down
    for (let i = 0; i < 20; i++) {
      const len = 0.8 + Math.random() * 2.2;
      const coneGeo = new THREE.ConeGeometry(0.25, len, 6);
      const stalactite = new THREE.Mesh(coneGeo, stalagmiteMat);
      stalactite.rotation.x = Math.PI;
      stalactite.position.set(-20 + Math.random() * 40, 5.5 - len / 2, -5 + Math.random() * 10);
      caveGroup.add(stalactite);
    }

    // Back Cave Wall
    const backWallGeo = new THREE.PlaneGeometry(45, 9);
    const backWall = new THREE.Mesh(backWallGeo, wallMat);
    backWall.position.set(0, 1, -6);
    caveGroup.add(backWall);

    scene.add(caveGroup);

    // ==========================================
    // 6. HYBRID TRI-MODAL DRONE ROBOT MODEL
    // ==========================================
    const robotGroup = new THREE.Group();
    robotGroupRef.current = robotGroup;

    // Body Hull (Carbon fiber composite)
    const hullMat = new THREE.MeshStandardMaterial({ color: 0x22262d, roughness: 0.4, metalness: 0.8 });
    const bodyGeo = new THREE.BoxGeometry(1.2, 0.35, 0.8);
    const bodyMesh = new THREE.Mesh(bodyGeo, hullMat);
    bodyMesh.castShadow = true;
    robotGroup.add(bodyMesh);

    // Glowing LED Ring
    const ringGeo = new THREE.TorusGeometry(0.3, 0.03, 8, 24);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0xffaa00 });
    const ringMesh = new THREE.Mesh(ringGeo, ringMat);
    ringMesh.rotation.x = Math.PI / 2;
    ringMesh.position.set(0, 0.18, 0);
    robotGroup.add(ringMesh);

    // Spotlight Headlight (High Intensity for Cave Exploration & FPV)
    const spotlight = new THREE.SpotLight(0xffffff, 8.0, 30, Math.PI / 4, 0.3, 1);
    spotlight.position.set(0.6, 0, 0);
    spotlight.target.position.set(3, 0, 0);
    spotlightRef.current = spotlight;
    robotGroup.add(spotlight);
    robotGroup.add(spotlight.target);

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

    // --- Kinematic Chain 1: Quadruped Legs ---
    const legs: THREE.Group[] = [];
    const legPositions = [
      { x: 0.5, z: 0.45, name: "FL" },
      { x: 0.5, z: -0.45, name: "FR" },
      { x: -0.5, z: 0.45, name: "BL" },
      { x: -0.5, z: -0.45, name: "BR" },
    ];

    const legMat = new THREE.MeshStandardMaterial({ color: 0x333b47, metalness: 0.7, roughness: 0.5 });
    const jointMat = new THREE.MeshStandardMaterial({ color: 0xff8800 });

    legPositions.forEach((pos) => {
      const legGroup = new THREE.Group();
      legGroup.position.set(pos.x, -0.1, pos.z);

      // Hip Joint
      const hipMesh = new THREE.Mesh(new THREE.SphereGeometry(0.08, 12, 12), jointMat);
      legGroup.add(hipMesh);

      // Thigh Link
      const thighMesh = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.4, 0.1), legMat);
      thighMesh.position.set(0, -0.2, 0);
      legGroup.add(thighMesh);

      // Shank / Foot Link
      const shankMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.05, 0.45), legMat);
      shankMesh.position.set(0, -0.55, 0);
      legGroup.add(shankMesh);

      robotGroup.add(legGroup);
      legs.push(legGroup);
    });
    legsRef.current = legs;

    // --- Kinematic Chain 2: Hydrofoil Pontoons (Sailing Mode) ---
    const pontoons: THREE.Group[] = [];
    const pontoonMat = new THREE.MeshStandardMaterial({ color: 0xeeb012, roughness: 0.3, metalness: 0.5 });

    [-0.55, 0.55].forEach((zPos) => {
      const pGroup = new THREE.Group();
      pGroup.position.set(0, -0.1, zPos);

      const pontoonMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 1.3, 16), pontoonMat);
      pontoonMesh.rotation.z = Math.PI / 2;
      pGroup.add(pontoonMesh);

      robotGroup.add(pGroup);
      pontoons.push(pGroup);
    });
    pontoonsRef.current = pontoons;

    // --- Kinematic Chain 3: Quadrotor Propellers (Flying Mode) ---
    const props: THREE.Mesh[] = [];
    const propMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.75 });

    [
      { x: 0.6, z: 0.55 },
      { x: 0.6, z: -0.55 },
      { x: -0.6, z: 0.55 },
      { x: -0.6, z: -0.55 },
    ].forEach((pPos) => {
      const armMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.3), legMat);
      armMesh.position.set(pPos.x, 0.1, pPos.z);
      robotGroup.add(armMesh);

      const propDisc = new THREE.Mesh(new THREE.CylinderGeometry(0.35, 0.35, 0.01, 16), propMat);
      propDisc.position.set(pPos.x, 0.25, pPos.z);
      robotGroup.add(propDisc);
      props.push(propDisc);
    });
    propellersRef.current = props;

    // --- Sensor Suite 3D Models ---
    // LiDAR Dome
    const lidarGroup = new THREE.Group();
    lidarGroup.position.set(0, 0.25, 0);
    const lidarBase = new THREE.Mesh(
      new THREE.CylinderGeometry(0.1, 0.1, 0.1, 16),
      new THREE.MeshStandardMaterial({ color: 0x111111 })
    );
    const lidarDome = new THREE.Mesh(
      new THREE.CylinderGeometry(0.09, 0.09, 0.08, 16),
      new THREE.MeshStandardMaterial({ color: 0x0088ff, roughness: 0.2 })
    );
    lidarDome.position.y = 0.08;
    lidarGroup.add(lidarBase);
    lidarGroup.add(lidarDome);
    robotGroup.add(lidarGroup);
    lidarMeshRef.current = lidarGroup;

    // Laser Ray Visualizer Lines (360 degree scan beam rays)
    const rayCount = 36;
    const linePositions = new Float32Array(rayCount * 6);
    const rayGeo = new THREE.BufferGeometry();
    rayGeo.setAttribute("position", new THREE.BufferAttribute(linePositions, 3));
    const rayMat = new THREE.LineBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.6 });
    const laserRays = new THREE.LineSegments(rayGeo, rayMat);
    laserRays.position.set(0, 0.33, 0);
    robotGroup.add(laserRays);
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
    sonarMesh.position.set(0.3, -0.2, 0);
    robotGroup.add(sonarMesh);
    sonarPulseMeshRef.current = sonarMesh;

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

    const domElem = mountRef.current;
    domElem.addEventListener("mousedown", handleMouseDown);
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

      // Update Robot Kinematics according to Mode & Position
      if (robotGroupRef.current) {
        const rPos = robotGroupRef.current.position;

        // Animate LiDAR Spinning Dome
        if (lidarMeshRef.current) {
          lidarMeshRef.current.rotation.y += 0.12;
        }

        // Animate LiDAR Laser Beams
        if (laserRaysRef.current && showLaserBeamsRef.current) {
          const pos = laserRaysRef.current.geometry.attributes.position;
          for (let i = 0; i < rayCount; i++) {
            const angle = (i / rayCount) * Math.PI * 2 + elapsedTime * 4;
            const dist = 3 + Math.sin(angle * 4 + elapsedTime * 2) * 1.5;
            pos.setXYZ(i * 2, 0, 0, 0);
            pos.setXYZ(i * 2 + 1, Math.cos(angle) * dist, (Math.random() - 0.5) * 0.5, Math.sin(angle) * dist);
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

        // Mode Specific Kinematic Adaptations & Visual Effects
        const currentMode = robotStateRef.current.mode;
        if (currentMode === "WALKING") {
          // Quadruped Leg Walking Gait Motion
          const gaitSpeed = 8;
          legsRef.current.forEach((leg, idx) => {
            const offset = idx % 2 === 0 ? 0 : Math.PI;
            leg.rotation.z = Math.sin(elapsedTime * gaitSpeed + offset) * 0.35;
            leg.position.y = -0.1 + Math.max(0, Math.sin(elapsedTime * gaitSpeed + offset) * 0.08);
          });
          // Retract Pontoons
          pontoonsRef.current.forEach((p) => {
            p.position.y = 0.1;
          });
          // Idle Propellers
          propellersRef.current.forEach((pr) => {
            pr.rotation.y = 0;
          });
        } else if (currentMode === "SAILING") {
          // Lock Legs into Floating Stance
          legsRef.current.forEach((leg) => {
            leg.rotation.z = -0.6;
            leg.position.y = 0.1;
          });
          // Lower Pontoons to Waterline
          pontoonsRef.current.forEach((p) => {
            p.position.y = -0.15;
          });
          // Bobbing Water Surface Motion
          rPos.y = -0.05 + Math.sin(elapsedTime * 2) * 0.04;
        } else if (currentMode === "FLYING") {
          // Fold Legs into Compact Skids
          legsRef.current.forEach((leg) => {
            leg.rotation.z = -1.2;
            leg.position.y = 0.12;
          });
          // Retract Pontoons
          pontoonsRef.current.forEach((p) => {
            p.position.y = 0.1;
          });
          // High RPM Spinning Propellers
          propellersRef.current.forEach((pr) => {
            pr.rotation.y += 0.8;
          });
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
            // First Person Drone Camera View (Elevated for clear front visibility)
            cameraRef.current.position.set(rPos.x + 0.65, rPos.y + 0.25, rPos.z);
            cameraRef.current.lookAt(rPos.x + 12, rPos.y + 0.2, rPos.z);
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

  // Sync robotState position change with 3D scene
  useEffect(() => {
    if (robotGroupRef.current) {
      robotGroupRef.current.position.set(robotState.position.x, robotState.position.y, robotState.position.z);
      robotGroupRef.current.rotation.set(
        (robotState.orientation.x * Math.PI) / 180,
        (robotState.orientation.y * Math.PI) / 180,
        (robotState.orientation.z * Math.PI) / 180
      );
    }
  }, [robotState.position, robotState.orientation]);

  // Determine current section label
  const getCurrentSectionName = (x: number): string => {
    if (x < -5) return "Section 1: Dry Cave (Rugged Rock)";
    if (x <= 12) return "Section 2: Flooded Cave (Water Surface z=0)";
    return "Section 3: Air Pocket Shaft (Air Ceiling)";
  };

  return (
    <div className="relative w-full h-full min-h-[420px] bg-slate-950 rounded-xl overflow-hidden border border-slate-800 shadow-2xl flex flex-col">
      {/* Viewport Top Control Overlay */}
      <div className="absolute top-3 left-3 right-3 z-10 flex flex-wrap items-center justify-between gap-2 bg-slate-900/80 backdrop-blur-md px-3 py-2 rounded-lg border border-slate-700/60 text-xs font-mono text-slate-200">
        <div className="flex items-center gap-2">
          <span className="inline-flex items-center gap-1.5 px-2.5 py-1 rounded bg-slate-800 text-sky-400 font-semibold border border-slate-700">
            <Compass className="w-3.5 h-3.5 animate-spin" style={{ animationDuration: "8s" }} />
            Gazebo Harmonic
          </span>
          <span className="text-slate-400 font-mono hidden sm:inline">
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
              showLaserBeams ? "bg-emerald-600/80 text-white" : "bg-slate-800 text-slate-400"
            }`}
            title="Toggle 3D LiDAR Laser Scan Beams"
          >
            LiDAR Rays
          </button>

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

      {/* Bottom Telemetry HUD Bar */}
      <div className="absolute bottom-3 left-3 right-3 z-10 bg-slate-900/85 backdrop-blur-md px-4 py-2.5 rounded-lg border border-slate-700/70 text-xs font-mono text-slate-200 flex flex-wrap items-center justify-between gap-3">
        <div className="flex items-center gap-4">
          <div>
            <span className="text-slate-400">Position (x, y, z):</span>{" "}
            <strong className="text-sky-300">
              [{robotState.position.x.toFixed(2)}, {robotState.position.y.toFixed(2)}, {robotState.position.z.toFixed(2)}]m
            </strong>
          </div>
          <div>
            <span className="text-slate-400">Mode:</span>{" "}
            <span
              className={`px-2 py-0.5 rounded font-bold ${
                robotState.mode === "WALKING"
                  ? "bg-amber-500/20 text-amber-400 border border-amber-500/30"
                  : robotState.mode === "SAILING"
                  ? "bg-cyan-500/20 text-cyan-400 border border-cyan-500/30"
                  : "bg-purple-500/20 text-purple-400 border border-purple-500/30"
              }`}
            >
              {robotState.mode}
            </span>
          </div>
        </div>

        <div className="flex items-center gap-4 text-slate-300">
          <div>
            <span className="text-slate-400">Buoyancy:</span>{" "}
            <span className="text-emerald-400 font-semibold">{robotState.buoyancyForce.toFixed(1)} N</span>
          </div>
          <div>
            <span className="text-slate-400">Rotor RPM:</span>{" "}
            <span className="text-sky-400 font-semibold">{robotState.propellerRpm}</span>
          </div>
          <div>
            <span className="text-slate-400">Battery:</span>{" "}
            <span className="text-amber-400 font-semibold">{robotState.battery.toFixed(0)}%</span>
          </div>
        </div>
      </div>
    </div>
  );
};
