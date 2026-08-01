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
  Wifi,
  Rss,
  Server,
  Map,
  Box,
  Layers,
  Download,
  CheckCircle2,
  Compass,
  Eye,
  Share2,
  Grid,
  Sliders,
  Database,
  Sparkles,
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  GripVertical,
  RotateCcw,
  LayoutGrid,
  Plus,
  Trash2,
  Edit3,
  X,
  Settings,
  Terminal,
  BarChart2,
  Shield,
  Play,
  Pause,
  Save,
  Upload,
  EyeOff,
  SlidersHorizontal,
  Check,
  Gauge,
  Cpu,
  Filter,
} from "lucide-react";

export type DashboardCardType =
  | "camera"
  | "lidar"
  | "slam3d"
  | "voxels"
  | "telemetry_gauge"
  | "quick_actions"
  | "ros_logs"
  | "sensor_chart";

export type AccentColor = "sky" | "emerald" | "amber" | "rose" | "indigo" | "cyan" | "purple";

export interface QuickButtonConfig {
  id: string;
  label: string;
  action: string;
  color: AccentColor;
}

export interface DashboardCardConfig {
  id: string;
  type: DashboardCardType;
  title: string;
  span: 1 | 2; // 1 = half width, 2 = full width
  accentColor: AccentColor;
  visible: boolean;
  gaugeMetric?: "speed" | "altitude" | "battery" | "pitch" | "roll" | "pressure";
  quickButtons?: QuickButtonConfig[];
}

const DEFAULT_CARDS: DashboardCardConfig[] = [
  {
    id: "camera",
    type: "camera",
    title: "RGB FPV Camera Feed",
    span: 1,
    accentColor: "sky",
    visible: true,
  },
  {
    id: "lidar",
    type: "lidar",
    title: "360° LiDAR Scan Map",
    span: 1,
    accentColor: "emerald",
    visible: true,
  },
  {
    id: "slam3d",
    type: "slam3d",
    title: "3D SLAM Mapping & Reconstruction Window",
    span: 2,
    accentColor: "sky",
    visible: true,
  },
  {
    id: "voxels",
    type: "voxels",
    title: "Shaded 3D Mesh Reconstruction",
    span: 2,
    accentColor: "indigo",
    visible: true,
  },
  {
    id: "quick_actions",
    type: "quick_actions",
    title: "ROS 2 Quick Action Control Bar",
    span: 1,
    accentColor: "amber",
    visible: true,
    quickButtons: [
      { id: "b1", label: "Headlight Toggle", action: "toggle_headlight", color: "amber" },
      { id: "b2", label: "Sonar Ping", action: "toggle_sonar", color: "cyan" },
      { id: "b3", label: "WiFi Relay", action: "toggle_wifi", color: "emerald" },
      { id: "b4", label: "Recharge Battery", action: "battery_full", color: "sky" },
      { id: "b5", label: "Emergency Stop", action: "emergency_stop", color: "rose" },
    ],
  },
  {
    id: "telemetry_gauge",
    type: "telemetry_gauge",
    title: "Live Subsea Telemetry Gauges",
    span: 1,
    accentColor: "cyan",
    visible: true,
    gaugeMetric: "speed",
  },
  {
    id: "ros_logs",
    type: "ros_logs",
    title: "Live ROS 2 Diagnostics Feed",
    span: 1,
    accentColor: "purple",
    visible: true,
  },
  {
    id: "sensor_chart",
    type: "sensor_chart",
    title: "Real-time IMU & Battery Plotter",
    span: 1,
    accentColor: "emerald",
    visible: true,
  },
];

interface SensorDashboardProps {
  robotState: RobotState;
  sensorData: SensorData;
  onSetBattery?: (val: number) => void;
  onToggleSonar?: () => void;
  onToggleWifi?: () => void;
}

export const SensorDashboard: React.FC<SensorDashboardProps> = ({
  robotState,
  sensorData,
  onSetBattery,
  onToggleSonar,
  onToggleWifi,
}) => {
  const lidarCanvasRef = useRef<HTMLCanvasElement>(null);
  const occupancyGridCanvasRef = useRef<HTMLCanvasElement>(null);
  const slamTrajectoryCanvasRef = useRef<HTMLCanvasElement>(null);
  const trajectoryPointsRef = useRef<{ x: number; y: number }[]>([]);
  const featureClustersRef = useRef<{ x: number; y: number; life: number }[]>([]);
  const discoveredCellsRef = useRef<Set<string>>(new Set());
  const cameraCanvasRef = useRef<HTMLCanvasElement>(null);
  const plotterCanvasRef = useRef<HTMLCanvasElement>(null);

  const isLowBattery = robotState.battery < 20;

  // Video Recording State
  const [isRecording, setIsRecording] = useState(false);
  const [recordSeconds, setRecordSeconds] = useState(0);

  // 3D SLAM Mapping Result Window Canvas & Controls
  const mapCanvasRef = useRef<HTMLCanvasElement>(null);
  const [mapViewMode, setMapViewMode] = useState<"iso" | "topdown" | "side">("iso");
  const [showLidarLayer, setShowLidarLayer] = useState(true);
  const [showSonarLayer, setShowSonarLayer] = useState(true);
  const [showWifiLayer, setShowWifiLayer] = useState(true);
  const [showWaypointsLayer, setShowWaypointsLayer] = useState(true);
  const [mapZoom, setMapZoom] = useState(1.0);
  const [isExportingMap, setIsExportingMap] = useState(false);

  // Shaded 3D Mesh Reconstruction Canvas & Explored Voxels
  const meshCanvasRef = useRef<HTMLCanvasElement>(null);
  const exploredMeshRef = useRef<Set<string>>(new Set());

  // Dashboard Card Layout & Customization State
  const [cardConfigs, setCardConfigs] = useState<DashboardCardConfig[]>(() => {
    const saved = localStorage.getItem("cavex_dashboard_cards_v2");
    if (saved) {
      try {
        return JSON.parse(saved);
      } catch (e) {
        return DEFAULT_CARDS;
      }
    }
    return DEFAULT_CARDS;
  });

  const [cardOrder, setCardOrder] = useState<string[]>(() => {
    const saved = localStorage.getItem("cavex_dashboard_order_v2");
    if (saved) {
      try {
        return JSON.parse(saved);
      } catch (e) {
        return DEFAULT_CARDS.map((c) => c.id);
      }
    }
    return DEFAULT_CARDS.map((c) => c.id);
  });

  const [draggedCard, setDraggedCard] = useState<string | null>(null);
  const [isEditingLayout, setIsEditingLayout] = useState(false);
  const [editingButtonCardId, setEditingButtonCardId] = useState<string | null>(null);
  const [showAddWidgetModal, setShowAddWidgetModal] = useState(false);
  const [showVisibilityDrawer, setShowVisibilityDrawer] = useState(false);

  // New Button Form State
  const [newBtnLabel, setNewBtnLabel] = useState("");
  const [newBtnAction, setNewBtnAction] = useState("toggle_headlight");
  const [newBtnColor, setNewBtnColor] = useState<AccentColor>("sky");

  // Save to localStorage when layout changes
  useEffect(() => {
    localStorage.setItem("cavex_dashboard_cards_v2", JSON.stringify(cardConfigs));
  }, [cardConfigs]);

  useEffect(() => {
    localStorage.setItem("cavex_dashboard_order_v2", JSON.stringify(cardOrder));
  }, [cardOrder]);

  // Card Management Helper Functions
  const moveCard = (id: string, direction: "prev" | "next") => {
    setCardOrder((prev) => {
      const idx = prev.indexOf(id);
      if (idx === -1) return prev;
      const targetIdx = direction === "prev" ? idx - 1 : idx + 1;
      if (targetIdx < 0 || targetIdx >= prev.length) return prev;
      const next = [...prev];
      const [removed] = next.splice(idx, 1);
      next.splice(targetIdx, 0, removed);
      return next;
    });
  };

  const updateCardConfig = (id: string, updater: (prev: DashboardCardConfig) => DashboardCardConfig) => {
    setCardConfigs((prev) => prev.map((c) => (c.id === id ? updater(c) : c)));
  };

  const deleteCard = (id: string) => {
    setCardConfigs((prev) => prev.filter((c) => c.id !== id));
    setCardOrder((prev) => prev.filter((cardId) => cardId !== id));
  };

  const toggleCardVisibility = (id: string) => {
    updateCardConfig(id, (c) => ({ ...c, visible: !c.visible }));
  };

  const handleDragStart = (e: React.DragEvent, id: string) => {
    setDraggedCard(id);
    e.dataTransfer.setData("text/plain", id);
    e.dataTransfer.effectAllowed = "move";
  };

  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = "move";
  };

  const handleDrop = (e: React.DragEvent, targetId: string) => {
    e.preventDefault();
    if (!draggedCard || draggedCard === targetId) return;

    setCardOrder((prev) => {
      const sourceIdx = prev.indexOf(draggedCard);
      const targetIdx = prev.indexOf(targetId);
      if (sourceIdx === -1 || targetIdx === -1) return prev;

      const next = [...prev];
      const [removed] = next.splice(sourceIdx, 1);
      next.splice(targetIdx, 0, removed);
      return next;
    });
    setDraggedCard(null);
  };

  // Preset Layout Setters
  const applyPresetLayout = (preset: "default" | "slam_focus" | "telemetry") => {
    if (preset === "default") {
      setCardConfigs(DEFAULT_CARDS);
      setCardOrder(DEFAULT_CARDS.map((c) => c.id));
    } else if (preset === "slam_focus") {
      const reordered = ["slam3d", "voxels", "lidar", "camera", "ros_logs", "telemetry_gauge", "quick_actions", "sensor_chart"];
      setCardOrder(reordered);
      setCardConfigs((prev) =>
        prev.map((c) =>
          c.id === "slam3d" || c.id === "voxels" ? { ...c, span: 2, visible: true } : { ...c, span: 1, visible: true }
        )
      );
    } else if (preset === "telemetry") {
      const reordered = ["quick_actions", "telemetry_gauge", "sensor_chart", "camera", "ros_logs", "lidar", "slam3d", "voxels"];
      setCardOrder(reordered);
      setCardConfigs((prev) =>
        prev.map((c) =>
          c.id === "quick_actions" || c.id === "telemetry_gauge" ? { ...c, span: 2, visible: true } : { ...c, visible: true }
        )
      );
    }
  };

  // Export / Import Layout JSON
  const exportLayoutJson = () => {
    const layoutData = {
      cardConfigs,
      cardOrder,
      exportedAt: new Date().toISOString(),
      app: "CaveX Subsea 3D SLAM Dashboard",
    };
    const blob = new Blob([JSON.stringify(layoutData, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = `cavex_dashboard_layout_${Date.now()}.json`;
    link.click();
    URL.revokeObjectURL(url);
  };

  const importLayoutJson = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (event) => {
      try {
        const parsed = JSON.parse(event.target?.result as string);
        if (parsed && Array.isArray(parsed.cardConfigs) && Array.isArray(parsed.cardOrder)) {
          setCardConfigs(parsed.cardConfigs);
          setCardOrder(parsed.cardOrder);
          alert("Dashboard layout imported successfully!");
        } else {
          alert("Invalid dashboard layout file format.");
        }
      } catch (err) {
        alert("Failed to parse JSON layout file.");
      }
    };
    reader.readAsText(file);
  };

  // Add Custom Widget
  const spawnNewWidget = (type: DashboardCardType) => {
    const newId = `custom_widget_${Date.now().toString(36)}`;
    const titleMap: Record<DashboardCardType, string> = {
      camera: "Custom Camera Feed",
      lidar: "Custom LiDAR Sensor Map",
      slam3d: "Custom 3D SLAM Map",
      voxels: "Custom 3D Mesh Reconstruction",
      telemetry_gauge: "Custom Telemetry Gauge Panel",
      quick_actions: "Custom Action Buttons Toolbar",
      ros_logs: "Custom ROS 2 Log Console",
      sensor_chart: "Custom Sensor Plotter Graph",
    };

    const newCard: DashboardCardConfig = {
      id: newId,
      type,
      title: titleMap[type] || "Custom Dashboard Card",
      span: type === "slam3d" || type === "voxels" ? 2 : 1,
      accentColor: "sky",
      visible: true,
      quickButtons:
        type === "quick_actions"
          ? [
              { id: "cb1", label: "Ping Sonar", action: "toggle_sonar", color: "cyan" },
              { id: "cb2", label: "Charge Battery", action: "battery_full", color: "emerald" },
            ]
          : undefined,
    };

    setCardConfigs((prev) => [...prev, newCard]);
    setCardOrder((prev) => [...prev, newId]);
    setShowAddWidgetModal(false);
  };

  // Execute Quick Action Trigger
  const triggerQuickAction = (action: string) => {
    switch (action) {
      case "toggle_headlight":
        // simulated toggle
        alert("ROS 2 Command Sent: /headlight/set_state -> TOGGLE");
        break;
      case "toggle_sonar":
        if (onToggleSonar) onToggleSonar();
        break;
      case "toggle_wifi":
        if (onToggleWifi) onToggleWifi();
        break;
      case "battery_full":
        if (onSetBattery) onSetBattery(100);
        break;
      case "emergency_stop":
        alert("⚠️ EMERGENCY STOP ACTIVATED: ROS 2 /cmd_vel zeroed out & motor brakes engaged.");
        break;
      case "calibrate_imu":
        alert("Gyro & Accelerometer recalibration sequence initiated on ROS 2 topic /imu/calibrate.");
        break;
      case "record_video":
        setIsRecording(!isRecording);
        break;
      case "export_map":
        handleExportPcdMap();
        break;
      default:
        alert(`Executed ROS Action: ${action}`);
        break;
    }
  };

  // Add Custom Button to Quick Actions Card
  const addCustomButtonToCard = (cardId: string) => {
    if (!newBtnLabel.trim()) return;
    const newBtn: QuickButtonConfig = {
      id: `btn_${Date.now()}`,
      label: newBtnLabel.trim(),
      action: newBtnAction,
      color: newBtnColor,
    };

    updateCardConfig(cardId, (c) => ({
      ...c,
      quickButtons: [...(c.quickButtons || []), newBtn],
    }));

    setNewBtnLabel("");
    setEditingButtonCardId(null);
  };

  const deleteButtonFromCard = (cardId: string, buttonId: string) => {
    updateCardConfig(cardId, (c) => ({
      ...c,
      quickButtons: (c.quickButtons || []).filter((b) => b.id !== buttonId),
    }));
  };


  // PCD Point Cloud Data Export Function
  const handleExportPcdMap = () => {
    setIsExportingMap(true);
    setTimeout(() => {
      let pcdContent = `# .PCD v.7 - Point Cloud Data file generated by CaveX 3D SLAM Engine\n`;
      pcdContent += `VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n`;
      pcdContent += `WIDTH 385420\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 385420\nDATA ascii\n`;

      // Generate synthetic point cloud records matching the 3 sections
      for (let i = 0; i < 200; i++) {
        const x = (-15 + Math.random() * 30).toFixed(3);
        const y = (-3 + Math.random() * 6).toFixed(3);
        const z = (Math.random() * 24.5).toFixed(3);
        const rgb = 16777215; // white
        pcdContent += `${x} ${y} ${z} ${rgb}\n`;
      }

      const blob = new Blob([pcdContent], { type: "text/plain;charset=utf-8" });
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = `cavex_3d_slam_pointcloud_map_${Date.now()}.pcd`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(url);
      setIsExportingMap(false);
    }, 600);
  };

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
    ctx.fillText(`Confidence: ${(sensorData?.slamConfidence ?? 100).toFixed(0)}%`, 8, 30);
  }, [sensorData, robotState]);

  // Update & Render Occupancy Grid Overlay
  useEffect(() => {
    // 1. Update Discovered Cells
    const rx = robotState.position.x;
    const ry = robotState.position.y;
    // Map a swath of floor
    for (let dx = -3; dx <= 3; dx += 0.5) {
      for (let dy = -3; dy <= 3; dy += 0.5) {
        if (dx * dx + dy * dy <= 9) {
          const cx = Math.round((rx + dx) * 2) / 2;
          const cy = Math.round((ry + dy) * 2) / 2;
          discoveredCellsRef.current.add(`${cx},${cy}`);
        }
      }
    }

    // 2. Render to Overlay Canvas
    const canvas = occupancyGridCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    const centerX = w / 2;
    const centerY = h / 2;
    const scale = 16; // 1 meter = 16px

    ctx.fillStyle = "rgba(16, 185, 129, 0.25)"; // Emerald-500 transparent
    ctx.strokeStyle = "rgba(16, 185, 129, 0.4)";
    ctx.lineWidth = 1;

    discoveredCellsRef.current.forEach((cell) => {
      const [cellX, cellY] = cell.split(",").map(Number);
      
      // Calculate position relative to robot
      const dx = cellX - rx;
      const dy = cellY - ry;
      
      // In LiDAR map, robot always faces right (if yaw is zero)
      // The local map is fixed in orientation, just translated.
      const sx = centerX + dx * scale;
      const sy = centerY + dy * scale;
      
      const cellSize = 0.5 * scale;
      
      if (sx > -cellSize && sx < w + cellSize && sy > -cellSize && sy < h + cellSize) {
        ctx.fillRect(sx - cellSize / 2, sy - cellSize / 2, cellSize, cellSize);
        ctx.strokeRect(sx - cellSize / 2, sy - cellSize / 2, cellSize, cellSize);
      }
    });
  }, [robotState.position.x, robotState.position.y]);

  // Update & Render SLAM Trajectory & Feature Point Clusters Overlay
  useEffect(() => {
    const canvas = slamTrajectoryCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const w = canvas.width;
    const h = canvas.height;

    // 1. Record New Trajectory Pose
    const pose = { x: sensorData.slamPose.x, y: sensorData.slamPose.y };
    const lastPose = trajectoryPointsRef.current[trajectoryPointsRef.current.length - 1];
    
    // Only append if moved a bit, to keep array size manageable
    if (!lastPose || Math.hypot(lastPose.x - pose.x, lastPose.y - pose.y) > 0.1) {
      trajectoryPointsRef.current.push(pose);
      if (trajectoryPointsRef.current.length > 300) {
        trajectoryPointsRef.current.shift();
      }
    }

    // 2. Generate Random Feature Points based on count
    // The number of feature points to generate per tick scales with sensorData.featurePointsCount
    const numFeaturesToGen = Math.min(Math.floor(sensorData.featurePointsCount / 10), 10);
    for (let i = 0; i < numFeaturesToGen; i++) {
      featureClustersRef.current.push({
        x: pose.x + (Math.random() - 0.5) * 4,
        y: pose.y + (Math.random() - 0.5) * 4,
        life: 1.0, // life decays from 1 to 0
      });
    }

    // Decay life & filter out dead feature points
    featureClustersRef.current.forEach(f => f.life -= 0.02);
    featureClustersRef.current = featureClustersRef.current.filter(f => f.life > 0);

    // 3. Clear and Render
    ctx.clearRect(0, 0, w, h);

    const centerX = w / 2;
    const centerY = h / 2;
    const scale = 16; // 1 meter = 16px
    const rx = robotState.position.x;
    const ry = robotState.position.y;

    // Render Trajectory Line
    if (trajectoryPointsRef.current.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = "rgba(167, 139, 250, 0.9)"; // Purple-400
      ctx.lineWidth = 2;
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      
      trajectoryPointsRef.current.forEach((pt, idx) => {
        const dx = pt.x - rx;
        const dy = pt.y - ry;
        const sx = centerX + dx * scale;
        const sy = centerY + dy * scale;
        if (idx === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }

    // Render Feature Clusters (cyan dots with fading opacity)
    featureClustersRef.current.forEach(pt => {
      const dx = pt.x - rx;
      const dy = pt.y - ry;
      const sx = centerX + dx * scale;
      const sy = centerY + dy * scale;
      
      ctx.fillStyle = `rgba(56, 189, 248, ${Math.max(0, pt.life)})`; // Sky-400
      ctx.beginPath();
      ctx.arc(sx, sy, 1.5, 0, Math.PI * 2);
      ctx.fill();
    });

  }, [sensorData.slamPose, sensorData.featurePointsCount, robotState.position]);

  // Render 3D SLAM Mapping Result Window Canvas
  useEffect(() => {
    const canvas = mapCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;

    // Dark sleek map background
    ctx.fillStyle = "#030712";
    ctx.fillRect(0, 0, w, h);

    // Draw Grid Pattern
    ctx.strokeStyle = "#111827";
    ctx.lineWidth = 1;
    const gridSize = 24 * mapZoom;
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

    // Coordinate mapping helper from 3D ROS space (x: -39 to +41, y: -5 to +5, z: -3 to +25)
    // to 2D Canvas Screen coordinates based on selected mapViewMode
    const toScreen = (rx: number, ry: number, rz: number) => {
      if (mapViewMode === "topdown") {
        // 2D Top Down View (x along canvas width, y along canvas height)
        const sx = ((rx + 39) / 80) * w * mapZoom + (1 - mapZoom) * (w / 2);
        const sy = h / 2 + (ry / 10) * (h * 0.7) * mapZoom;
        return { x: sx, y: sy };
      } else if (mapViewMode === "side") {
        // Side Elevation Profile View (x along width, z altitude along height)
        const sx = ((rx + 39) / 80) * w * mapZoom + (1 - mapZoom) * (w / 2);
        const sy = h * 0.82 - (rz / 26) * (h * 0.75) * mapZoom;
        return { x: sx, y: sy };
      } else {
        // Isometric 3D Projection
        const isoX = (rx * 0.85 - ry * 0.85) * 6 * mapZoom;
        const isoY = (rx * 0.35 + ry * 0.35) * 4.5 * mapZoom - rz * 3.5 * mapZoom;
        const sx = w * 0.45 + isoX;
        const sy = h * 0.70 + isoY;
        return { x: sx, y: sy };
      }
    };

    // 1. Draw Section Boundaries & Cavern Enclosures
    ctx.strokeStyle = "rgba(75, 85, 99, 0.4)";
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);

    // Section 1: Dry Cave (-39 to -5)
    const sec1A = toScreen(-39, 0, 0);
    const sec1B = toScreen(-5, 0, 0);
    ctx.beginPath();
    ctx.moveTo(sec1A.x, sec1A.y);
    ctx.lineTo(sec1B.x, sec1B.y);
    ctx.stroke();

    // Section 2: Flooded Water (-5 to 29)
    const sec2A = toScreen(-5, 0, 0);
    const sec2B = toScreen(29, 0, 0);
    ctx.strokeStyle = "rgba(6, 182, 212, 0.5)";
    ctx.beginPath();
    ctx.moveTo(sec2A.x, sec2A.y);
    ctx.lineTo(sec2B.x, sec2B.y);
    ctx.stroke();

    // Section 3: Vertical Shaft (29 to 41, height up to 25m)
    const shaftBase = toScreen(35.5, 0, 0);
    const shaftApex = toScreen(35.5, 0, 24);
    ctx.strokeStyle = "rgba(168, 85, 247, 0.6)";
    ctx.beginPath();
    ctx.moveTo(shaftBase.x, shaftBase.y);
    ctx.lineTo(shaftApex.x, shaftApex.y);
    ctx.stroke();
    ctx.setLineDash([]);

    // 2. Render Point Cloud Layers
    // Layer 1: Dry Cave Rock Points (Amber/Orange)
    if (showLidarLayer) {
      ctx.fillStyle = "#f59e0b";
      for (let i = 0; i < 1600; i++) {
        const seed = i * 1.618;
        const px = -39.0 + (i / 1600) * 34.0;
        const py = Math.sin(seed * 3) * 1.8 + Math.cos(seed * 5) * 0.5;
        const pz = 0.5 + Math.cos(seed * 2) * 1.2 + Math.sin(seed * 7) * 0.6;
        const pt = toScreen(px, py, pz);
        ctx.fillRect(pt.x - 1, pt.y - 1, 2, 2);
      }
    }

    // Layer 2: Flooded Water Seabed Bathymetry Points (Cyan/Teal)
    if (showSonarLayer) {
      ctx.fillStyle = "#06b6d4";
      for (let i = 0; i < 2000; i++) {
        const seed = i * 2.718;
        const px = -5 + (i / 2000) * 34.0;
        const py = Math.cos(seed * 2) * 2.2 + Math.sin(seed * 3.5) * 1.1;
        const pz = -0.5 - Math.sin(seed * 4) * 1.5 + Math.cos(seed * 5) * 0.4; // underwater seabed depth profile
        const pt = toScreen(px, py, pz);
        ctx.fillRect(pt.x - 1, pt.y - 1, 2, 2);
      }
    }

    // Layer 3: Vertical Ascent Shaft Chimney Helical Point Cloud (Magenta/Sky-Blue)
    if (showLidarLayer) {
      for (let i = 0; i < 3000; i++) {
        const angle = i * 0.15;
        const radius = 4.5 + Math.sin(i * 0.1) * 0.5 + Math.cos(i * 0.05) * 0.2;
        const px = 35.5 + Math.cos(angle) * radius;
        const py = Math.sin(angle) * radius;
        const pz = (i / 3000) * 25.0; // ascending 25m shaft
        const pt = toScreen(px, py, pz);

        ctx.fillStyle = pz > 15 ? "#c084fc" : "#38bdf8";
        ctx.fillRect(pt.x - 1.2, pt.y - 1.2, 2.4, 2.4);
      }
    }

    // Layer 4: WiFi Video Stream SLAM Keyframe Markers (Lime/Yellow)
    if (showWifiLayer) {
      ctx.fillStyle = "#a3e635";
      for (let k = 0; k < 12; k++) {
        const kx = 35.5 + Math.cos(k) * 2;
        const ky = Math.sin(k) * 1.2;
        const kz = 2 + k * 1.8;
        const pt = toScreen(kx, ky, kz);

        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // 3. Render Waypoint Trajectory Lines
    if (showWaypointsLayer) {
      const waypoints = [
        { x: -34.5, y: 0, z: 1.35 },
        { x: -20, y: 0.3, z: 1.35 },
        { x: -7, y: 0, z: 1.35 },
        { x: -3, y: 0, z: 0.55 },
        { x: 5, y: -0.4, z: 0.55 },
        { x: 15, y: 0.3, z: 0.55 },
        { x: 25, y: 0, z: 0.55 },
        { x: 29, y: 0, z: 0.8 },
        { x: 30.0, y: 0, z: 2.0 },
        { x: 34.2, y: 1.5, z: 5.5 },
        { x: 35.8, y: -1.5, z: 8.5 },
        { x: 35.0, y: 0.8, z: 12.0 },
        { x: 35.8, y: -1.2, z: 15.5 },
        { x: 34.5, y: 1.2, z: 19.0 },
        { x: 35.5, y: 0, z: 23.5 },
      ];

      ctx.strokeStyle = "#10b981";
      ctx.lineWidth = 2;
      ctx.beginPath();
      waypoints.forEach((wp, idx) => {
        const pt = toScreen(wp.x, wp.y, wp.z);
        if (idx === 0) ctx.moveTo(pt.x, pt.y);
        else ctx.lineTo(pt.x, pt.y);
      });
      ctx.stroke();

      // Waypoint Node Rings
      waypoints.forEach((wp) => {
        const pt = toScreen(wp.x, wp.y, wp.z);
        ctx.fillStyle = "#10b981";
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 3.5, 0, Math.PI * 2);
        ctx.fill();
      });
    }

    // 4. Render Spot Ground Carrier Base Station (Parked at X=12.0, Y=0, Z=0.6)
    const spotPos = toScreen(12.0, 0, 0.6);
    ctx.fillStyle = "#f59e0b";
    ctx.beginPath();
    ctx.arc(spotPos.x, spotPos.y, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#fef08a";
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.fillStyle = "#fde047";
    ctx.font = "bold 9px monospace";
    ctx.fillText("Spot Base Station", spotPos.x + 9, spotPos.y + 3);

    // 5. Render Active Flying Drone (at current robotState.position)
    const dronePos = toScreen(
      robotState.position.x,
      robotState.position.y,
      robotState.position.z
    );

    // WiFi Link Ray connecting Flying Drone to Ground Spot Base Station!
    if (robotState.mode === "FLYING") {
      ctx.strokeStyle = "rgba(56, 189, 248, 0.75)";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(dronePos.x, dronePos.y);
      ctx.lineTo(spotPos.x, spotPos.y);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Drone Marker
    ctx.fillStyle = "#38bdf8";
    ctx.beginPath();
    ctx.arc(dronePos.x, dronePos.y, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Pulse Ring around Drone
    ctx.strokeStyle = "rgba(56, 189, 248, 0.6)";
    ctx.beginPath();
    ctx.arc(dronePos.x, dronePos.y, 13, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = "#38bdf8";
    ctx.font = "bold 10px monospace";
    ctx.fillText(
      robotState.mode === "FLYING" ? "VTOL Drone (Flying)" : "Rover Active",
      dronePos.x + 12,
      dronePos.y - 4
    );
  }, [
    robotState,
    sensorData,
    mapViewMode,
    showLidarLayer,
    showSonarLayer,
    showWifiLayer,
    showWaypointsLayer,
    mapZoom,
  ]);

  // Render Shaded 3D Mesh Reconstruction (InstancedMesh simulation with dynamic shadows & altitude HSL coloring)
  useEffect(() => {
    const canvas = meshCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;

    // Dark high-tech cavern canvas background
    ctx.fillStyle = "#020617";
    ctx.fillRect(0, 0, w, h);

    // Isometric projection mapping function
    function toMeshIso(rx: number, ry: number, rz: number) {
      const isoX = (rx * 0.82 - ry * 0.82) * 6.5 * mapZoom;
      const isoY = (rx * 0.35 + ry * 0.35) * 4.8 * mapZoom - rz * 3.8 * mapZoom;
      return { x: w * 0.46 + isoX, y: h * 0.68 + isoY };
    }

    // Subtle isometric grid lines
    ctx.strokeStyle = "rgba(30, 41, 59, 0.4)";
    ctx.lineWidth = 1;
    for (let x = -20; x <= 40; x += 5) {
      const p1 = toMeshIso(x, -10, 0);
      const p2 = toMeshIso(x, 10, 0);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }
    for (let y = -10; y <= 10; y += 5) {
      const p1 = toMeshIso(-20, y, 0);
      const p2 = toMeshIso(40, y, 0);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }

    // Update explored mesh voxels based on robot position (1cm resolution grid)
    if (robotState) {
      const rx = robotState.position.x;
      const ry = robotState.position.y;
      const rz = robotState.position.z;
      for (let dx = -6; dx <= 6; dx += 2) {
        for (let dy = -4; dy <= 4; dy += 2) {
          for (let dz = -4; dz <= 4; dz += 2) {
            const gx = Math.round((rx + dx) * 100) / 100;
            const gy = Math.round((ry + dy) * 100) / 100;
            const gz = Math.round((rz + dz) * 100) / 100;
            exploredMeshRef.current.add(`${gx.toFixed(2)},${gy.toFixed(2)},${gz.toFixed(2)}`);
          }
        }
      }
    }

    // Altitude-based HSL color generator for fully shaded InstancedMesh voxels
    const getAltitudeColor = (alt: number, lightFactor: number) => {
      const normAlt = Math.max(0, Math.min(1, (alt + 2) / 27));
      let hue = 160;
      if (alt < 0) hue = 200 + alt * 10; // Submerged cyan-blue
      else if (alt <= 5) hue = 160 + (alt / 5) * 60; // Emerald to purple
      else if (alt <= 18) hue = 220 + ((alt - 5) / 13) * 60; // Violet/indigo
      else hue = 280 + ((alt - 18) / 7) * 70; // Gold/amber

      const sat = 85;
      const lightness = Math.max(15, Math.min(80, (45 + normAlt * 20) * lightFactor));
      return `hsl(${hue}, ${sat}%, ${lightness}%)`;
    };

    // Directional light vector for shadow offset
    const lightDir = { x: 1.2, y: 0.8 };

    // Prepare Voxel Array sorted for correct depth occlusion
    const voxelsToRender: Array<{ x: number; y: number; z: number }> = [];

    // Dry Cave Floor & Tunnel Topology
    for (let x = -18; x <= 28; x += 1.5) {
      for (let y = -4; y <= 4; y += 1.5) {
        const key = `${Math.round(x)},${Math.round(y)},0`;
        const isExp = exploredMeshRef.current.has(key) || x < -10;
        if (!isExp) continue;

        let z = 0;
        if (x >= 0 && x <= 28) {
          z = Math.abs(y) % 2 === 0 ? -1.2 : -0.8;
        } else {
          z = Math.sin(x * 0.3) * 0.3 + Math.cos(y * 0.5) * 0.2;
        }
        voxelsToRender.push({ x, y, z });
      }
    }

    // Shaft Chimney (x = 30 to 36, z = 0 to 24m)
    for (let z = 0; z <= 24; z += 2) {
      for (let angle = 0; angle < Math.PI * 2; angle += Math.PI / 4) {
        const x = 33 + Math.cos(angle) * 4.2;
        const y = Math.sin(angle) * 4.2;
        const key = `${Math.round(x)},${Math.round(y)},${Math.round(z)}`;
        if (exploredMeshRef.current.has(key) || z < 8 || robotState.position.x > 15) {
          voxelsToRender.push({ x, y, z });
        }
      }
    }

    // Depth sorting
    voxelsToRender.sort((a, b) => a.x + a.y - a.z * 0.5 - (b.x + b.y - b.z * 0.5));

    // 1. Pass 1: Render Dynamic Shadows onto Ground
    ctx.fillStyle = "rgba(2, 6, 23, 0.6)";
    voxelsToRender.forEach((v) => {
      if (v.z > 0.1) {
        const sp1 = toMeshIso(v.x + lightDir.x, v.y + lightDir.y, 0);
        const sp2 = toMeshIso(v.x + 1.2 + lightDir.x, v.y + lightDir.y, 0);
        const sp3 = toMeshIso(v.x + 1.2 + lightDir.x, v.y + 1.2 + lightDir.y, 0);
        const sp4 = toMeshIso(v.x + lightDir.x, v.y + 1.2 + lightDir.y, 0);

        ctx.beginPath();
        ctx.moveTo(sp1.x, sp1.y);
        ctx.lineTo(sp2.x, sp2.y);
        ctx.lineTo(sp3.x, sp3.y);
        ctx.lineTo(sp4.x, sp4.y);
        ctx.closePath();
        ctx.fill();
      }
    });

    // 2. Pass 2: Render Fully Shaded 3D Instanced Mesh Voxels
    const boxSize = 1.1;
    voxelsToRender.forEach((v) => {
      const topP1 = toMeshIso(v.x, v.y, v.z + boxSize);
      const topP2 = toMeshIso(v.x + boxSize, v.y, v.z + boxSize);
      const topP3 = toMeshIso(v.x + boxSize, v.y + boxSize, v.z + boxSize);
      const topP4 = toMeshIso(v.x, v.y + boxSize, v.z + boxSize);

      const botP2 = toMeshIso(v.x + boxSize, v.y, v.z);
      const botP3 = toMeshIso(v.x + boxSize, v.y + boxSize, v.z);
      const botP4 = toMeshIso(v.x, v.y + boxSize, v.z);

      // Top Face (Lit by main overhead light)
      ctx.fillStyle = getAltitudeColor(v.z, 1.2);
      ctx.strokeStyle = "rgba(15, 23, 42, 0.4)";
      ctx.lineWidth = 0.5;
      ctx.beginPath();
      ctx.moveTo(topP1.x, topP1.y);
      ctx.lineTo(topP2.x, topP2.y);
      ctx.lineTo(topP3.x, topP3.y);
      ctx.lineTo(topP4.x, topP4.y);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // Front-Right Face
      ctx.fillStyle = getAltitudeColor(v.z, 0.85);
      ctx.beginPath();
      ctx.moveTo(topP2.x, topP2.y);
      ctx.lineTo(topP3.x, topP3.y);
      ctx.lineTo(botP3.x, botP3.y);
      ctx.lineTo(botP2.x, botP2.y);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // Front-Left Face
      ctx.fillStyle = getAltitudeColor(v.z, 0.65);
      ctx.beginPath();
      ctx.moveTo(topP3.x, topP3.y);
      ctx.lineTo(topP4.x, topP4.y);
      ctx.lineTo(botP4.x, botP4.y);
      ctx.lineTo(botP3.x, botP3.y);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    });

    // 3. Render SLAM Robot Tracking Agent with Beacon & Line Drop
    if (robotState) {
      const rp = toMeshIso(robotState.position.x, robotState.position.y, robotState.position.z);
      const rpGround = toMeshIso(robotState.position.x, robotState.position.y, 0);

      ctx.strokeStyle = "rgba(16, 185, 129, 0.6)";
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(rp.x, rp.y);
      ctx.lineTo(rpGround.x, rpGround.y);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.beginPath();
      ctx.arc(rp.x, rp.y, 7, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(16, 185, 129, 0.35)";
      ctx.fill();

      ctx.beginPath();
      ctx.arc(rp.x, rp.y, 3.5, 0, Math.PI * 2);
      ctx.fillStyle = "#10b981";
      ctx.fill();

      ctx.fillStyle = "#34d399";
      ctx.font = "bold 10px monospace";
      ctx.fillText(
        `SLAM AGENT: [${(robotState?.position?.x ?? 0).toFixed(1)}m, ${(robotState?.position?.y ?? 0).toFixed(1)}m, ${(robotState?.position?.z ?? 0).toFixed(1)}m]`,
        rp.x + 10,
        rp.y - 4
      );
    }
  }, [robotState, mapZoom]);

  // Render Sensor Plotter Canvas (Real-time IMU Acceleration & Battery waveform)
  useEffect(() => {
    const canvas = plotterCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;

    ctx.fillStyle = "#020617";
    ctx.fillRect(0, 0, w, h);

    // Draw grid
    ctx.strokeStyle = "rgba(30, 41, 59, 0.5)";
    ctx.lineWidth = 1;
    for (let x = 0; x < w; x += 30) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let y = 0; y < h; y += 20) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // Sine wave IMU acceleration + noise
    const now = Date.now() * 0.003;
    ctx.strokeStyle = "#10b981";
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let x = 0; x < w; x += 2) {
      const t = now - (w - x) * 0.02;
      const y = h / 2 + Math.sin(t * 2) * 22 + Math.cos(t * 5) * 8 + (Math.random() - 0.5) * 4;
      if (x === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Secondary curve (Battery discharge voltage)
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let x = 0; x < w; x += 4) {
      const y = h * 0.75 - (x / w) * 15 + Math.sin(x * 0.05 + now) * 3;
      if (x === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    ctx.fillStyle = "#10b981";
    ctx.font = "10px monospace";
    ctx.fillText("IMU Z-Acc (m/s²)", 10, 16);
    ctx.fillStyle = "#38bdf8";
    ctx.fillText("Battery Bus (V)", 120, 16);
  }, [robotState, sensorData]);

  // Accent Color Mapping Helper
  const getAccentBorder = (color?: AccentColor) => {
    switch (color) {
      case "emerald":
        return "border-emerald-500/40 text-emerald-400 bg-emerald-950/20";
      case "amber":
        return "border-amber-500/40 text-amber-400 bg-amber-950/20";
      case "rose":
        return "border-rose-500/40 text-rose-400 bg-rose-950/20";
      case "indigo":
        return "border-indigo-500/40 text-indigo-400 bg-indigo-950/20";
      case "cyan":
        return "border-cyan-500/40 text-cyan-400 bg-cyan-950/20";
      case "purple":
        return "border-purple-500/40 text-purple-400 bg-purple-950/20";
      case "sky":
      default:
        return "border-sky-500/40 text-sky-400 bg-sky-950/20";
    }
  };

  const getAccentHeader = (color?: AccentColor) => {
    switch (color) {
      case "emerald":
        return "text-emerald-300";
      case "amber":
        return "text-amber-300";
      case "rose":
        return "text-rose-300";
      case "indigo":
        return "text-indigo-300";
      case "cyan":
        return "text-cyan-300";
      case "purple":
        return "text-purple-300";
      case "sky":
      default:
        return "text-sky-300";
    }
  };

  // Render individual card based on config
  const renderCardContentByConfig = (card: DashboardCardConfig) => {
    switch (card.type) {
      case "camera":
        return (
          <div className="bg-slate-950 rounded-lg p-2.5 border border-slate-800 flex flex-col justify-between gap-2 h-full">
            <div className="flex items-center justify-between text-xs font-mono text-slate-400 pr-24">
              <span className={`flex items-center gap-1.5 font-semibold ${getAccentHeader(card.accentColor)}`}>
                <Camera className="w-3.5 h-3.5" /> {card.title}
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
              <div className="absolute top-2 left-2 flex flex-wrap items-center gap-1">
                <span className="bg-slate-900/80 backdrop-blur text-sky-300 border border-slate-700 text-[9px] px-1.5 py-0.5 rounded font-mono font-bold">
                  FPV ISO: 3200
                </span>
                <span className="bg-emerald-500/20 text-emerald-400 border border-emerald-500/30 text-[9px] px-1.5 py-0.5 rounded font-mono font-bold">
                  +{brightnessLevel === "ultra" ? "3.0" : brightnessLevel === "high" ? "1.5" : "0.0"} EV
                </span>
              </div>
              <div className="absolute top-2 right-2 flex items-center gap-1 bg-sky-950/80 backdrop-blur text-sky-300 border border-sky-500/40 text-[9px] px-1.5 py-0.5 rounded font-mono font-bold">
                <Wifi className="w-3 h-3 text-sky-400 animate-pulse" />
                <span>WiFi Stream: {(sensorData?.wifiBitrateMbps ?? 0).toFixed(1)} Mbps</span>
              </div>
            </div>

            <div className="flex items-center gap-1.5 pt-1 border-t border-slate-800/80 font-mono text-[10px]">
              <button
                id="btn-record-video-dash"
                onClick={() => setIsRecording(!isRecording)}
                className={`flex-1 py-1 px-2 rounded font-bold transition flex items-center justify-center gap-1.5 ${
                  isRecording
                    ? "bg-rose-600 hover:bg-rose-500 text-white shadow-[0_0_10px_rgba(239,68,68,0.5)] animate-pulse"
                    : "bg-slate-900 hover:bg-slate-800 text-rose-400 border border-rose-500/30 hover:border-rose-500"
                }`}
              >
                <Video className="w-3.5 h-3.5" />
                {isRecording ? `REC (${formatRecordTime(recordSeconds)})` : "Record Video"}
              </button>

              <button
                id="btn-toggle-camera-brightness"
                onClick={() =>
                  setBrightnessLevel((prev) =>
                    prev === "standard" ? "high" : prev === "high" ? "ultra" : "standard"
                  )
                }
                className="py-1 px-2 rounded bg-slate-900 hover:bg-slate-800 text-amber-300 border border-slate-700 font-bold transition flex items-center gap-1"
              >
                <Sun className="w-3.5 h-3.5 text-amber-400" />
                <span className="uppercase text-[9px]">{brightnessLevel}</span>
              </button>
            </div>
          </div>
        );

      case "lidar":
        return (
          <div className="bg-slate-950 rounded-lg p-2.5 border border-slate-800 flex flex-col justify-between gap-2 h-full">
            <div className="flex items-center justify-between text-xs font-mono text-slate-400 pr-24">
              <span className={`flex items-center gap-1.5 font-semibold ${getAccentHeader(card.accentColor)}`}>
                <Scan className="w-3.5 h-3.5" /> {card.title}
              </span>
              <span className="text-slate-500">Topic: /scan (20Hz)</span>
            </div>

            <div className="relative w-full aspect-video rounded overflow-hidden border border-slate-800 bg-slate-950 flex items-center justify-center">
              <canvas ref={lidarCanvasRef} width={320} height={180} className="w-full h-full object-contain" />
              <canvas ref={occupancyGridCanvasRef} width={320} height={180} className="absolute inset-0 w-full h-full object-contain pointer-events-none opacity-80 mix-blend-screen" />
              <canvas ref={slamTrajectoryCanvasRef} width={320} height={180} className="absolute inset-0 w-full h-full object-contain pointer-events-none opacity-90" />
            </div>
          </div>
        );

      case "slam3d":
        return (
          <div className="bg-slate-950 rounded-xl p-3.5 border border-slate-800 flex flex-col gap-3 font-mono h-full">
            <div className="flex flex-wrap items-center justify-between gap-2 pb-2 border-b border-slate-800 pr-24">
              <div className="flex items-center gap-2">
                <span className="p-1.5 rounded-lg bg-sky-500/10 text-sky-400 border border-sky-500/30">
                  <Map className="w-4 h-4" />
                </span>
                <div>
                  <h4 className={`text-xs font-bold uppercase tracking-wider flex items-center gap-1.5 ${getAccentHeader(card.accentColor)}`}>
                    {card.title}
                    <Sparkles className="w-3.5 h-3.5 text-amber-400" />
                  </h4>
                  <p className="text-[10px] text-slate-400">
                    Real-time multi-modal point cloud aggregation & bathymetric surface profile
                  </p>
                </div>
              </div>

              <div className="flex flex-wrap items-center gap-1.5 bg-slate-900 p-1 rounded-lg border border-slate-800 text-[11px]">
                <button
                  onClick={() => setMapViewMode("iso")}
                  className={`px-2 py-0.5 rounded transition ${mapViewMode === "iso" ? "bg-sky-600 text-white font-bold" : "text-slate-400 hover:text-slate-200"}`}
                >
                  Isometric 3D
                </button>
                <button
                  onClick={() => setMapViewMode("topdown")}
                  className={`px-2 py-0.5 rounded transition ${mapViewMode === "topdown" ? "bg-sky-600 text-white font-bold" : "text-slate-400 hover:text-slate-200"}`}
                >
                  2D Occupancy
                </button>
                <button
                  onClick={() => setMapViewMode("side")}
                  className={`px-2 py-0.5 rounded transition ${mapViewMode === "side" ? "bg-sky-600 text-white font-bold" : "text-slate-400 hover:text-slate-200"}`}
                >
                  Side Elevation
                </button>
              </div>
            </div>

            <div className="relative w-full aspect-[21/8] min-h-[220px] rounded-lg overflow-hidden border border-slate-800 bg-slate-950">
              <canvas ref={mapCanvasRef} width={960} height={320} className="w-full h-full object-cover" />
              <div className="absolute top-2 left-2 flex flex-wrap items-center gap-1 bg-slate-900/80 backdrop-blur-md p-1 rounded-md border border-slate-700/80 text-[10px]">
                <button
                  onClick={() => setShowLidarLayer(!showLidarLayer)}
                  className={`px-2 py-0.5 rounded transition ${showLidarLayer ? "bg-amber-500/20 text-amber-300 border border-amber-500/50 font-bold" : "text-slate-500 line-through"}`}
                >
                  3D LiDAR
                </button>
                <button
                  onClick={() => setShowSonarLayer(!showSonarLayer)}
                  className={`px-2 py-0.5 rounded transition ${showSonarLayer ? "bg-cyan-500/20 text-cyan-300 border border-cyan-500/50 font-bold" : "text-slate-500 line-through"}`}
                >
                  Sonar Seabed
                </button>
                <button
                  onClick={() => setShowWifiLayer(!showWifiLayer)}
                  className={`px-2 py-0.5 rounded transition ${showWifiLayer ? "bg-lime-500/20 text-lime-300 border border-lime-500/50 font-bold" : "text-slate-500 line-through"}`}
                >
                  WiFi Keyframes
                </button>
                <button
                  onClick={() => setShowWaypointsLayer(!showWaypointsLayer)}
                  className={`px-2 py-0.5 rounded transition ${showWaypointsLayer ? "bg-emerald-500/20 text-emerald-300 border border-emerald-500/50 font-bold" : "text-slate-500 line-through"}`}
                >
                  Trajectory
                </button>
              </div>

              <div className="absolute bottom-2 right-2 flex items-center gap-1 bg-slate-900/80 backdrop-blur p-1 rounded border border-slate-700 text-[10px]">
                <button onClick={() => setMapZoom((z) => Math.max(0.6, z - 0.2))} className="px-2 py-0.5 bg-slate-800 text-slate-200 font-bold rounded">-</button>
                <span className="px-1 text-sky-300 font-bold">{(mapZoom * 100).toFixed(0)}%</span>
                <button onClick={() => setMapZoom((z) => Math.min(2.5, z + 0.2))} className="px-2 py-0.5 bg-slate-800 text-slate-200 font-bold rounded">+</button>
              </div>
            </div>

            <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-6 gap-2 text-[10px] pt-1">
              <div className="bg-slate-900/80 p-2 rounded border border-slate-800">
                <span className="text-slate-400 block">Mapped Volume</span>
                <strong className="text-emerald-400 text-xs">2,410.8 m³</strong>
              </div>
              <div className="bg-slate-900/80 p-2 rounded border border-slate-800">
                <span className="text-slate-400 block">PCD Points</span>
                <strong className="text-sky-400 text-xs">385,420 pts</strong>
              </div>
              <div className="bg-slate-900/80 p-2 rounded border border-slate-800">
                <span className="text-slate-400 block">Resolution</span>
                <strong className="text-amber-300 text-xs">0.05 m</strong>
              </div>
              <div className="bg-slate-900/80 p-2 rounded border border-slate-800">
                <span className="text-slate-400 block">Loop Closures</span>
                <strong className="text-purple-300 text-xs">14 Closures</strong>
              </div>
              <div className="bg-slate-900/80 p-2 rounded border border-slate-800">
                <span className="text-slate-400 block">Shaft Chimney</span>
                <strong className="text-sky-300 text-xs">25.0m Mapped</strong>
              </div>
              <button
                onClick={handleExportPcdMap}
                disabled={isExportingMap}
                className="py-1 px-2.5 rounded bg-sky-600 hover:bg-sky-500 text-white font-bold transition flex items-center justify-center gap-1 border border-sky-400/40 shadow"
              >
                {isExportingMap ? <RefreshCw className="w-3.5 h-3.5 animate-spin" /> : <Download className="w-3.5 h-3.5" />}
                <span>{isExportingMap ? "Generating..." : "Export PCD"}</span>
              </button>
            </div>
          </div>
        );

      case "voxels":
        return (
          <div className="bg-slate-950 rounded-xl p-3.5 border border-indigo-500/30 flex flex-col gap-3 font-mono h-full shadow-xl bg-gradient-to-r from-slate-950 via-slate-900 to-indigo-950/40">
            <div className="flex items-center justify-between pb-2 border-b border-slate-800 pr-24">
              <div className="flex items-center gap-2">
                <span className="p-1.5 rounded-lg bg-indigo-500/20 text-indigo-400 border border-indigo-500/40">
                  <Box className="w-4 h-4" />
                </span>
                <div>
                  <h4 className={`text-xs font-bold uppercase tracking-wider flex items-center gap-2 ${getAccentHeader(card.accentColor)}`}>
                    <span>{card.title}</span>
                    <span className="px-2 py-0.5 rounded-full text-[9px] bg-emerald-500/20 text-emerald-300 border border-emerald-500/40">
                      InstancedMesh Active
                    </span>
                  </h4>
                  <p className="text-[10px] text-slate-400">
                    Live surface tessellation with dynamic shadow casting, altitude-based coloring & responsive topology
                  </p>
                </div>
              </div>
            </div>

            <div className="relative w-full aspect-[21/8] min-h-[240px] rounded-lg overflow-hidden border border-slate-800 bg-[#020617] shadow-inner">
              <canvas ref={meshCanvasRef} width={960} height={320} className="w-full h-full object-cover" />
            </div>
          </div>
        );

      case "quick_actions":
        return (
          <div className="bg-slate-950 rounded-lg p-3 border border-slate-800 flex flex-col justify-between gap-3 h-full font-mono">
            <div className="flex items-center justify-between pr-24 border-b border-slate-800 pb-2">
              <span className={`text-xs font-bold flex items-center gap-1.5 ${getAccentHeader(card.accentColor)}`}>
                <Zap className="w-3.5 h-3.5" /> {card.title}
              </span>
              <span className="text-[10px] text-slate-400">ROS 2 Services</span>
            </div>

            <div className="grid grid-cols-2 sm:grid-cols-3 gap-2 py-1">
              {(card.quickButtons || []).map((btn) => (
                <button
                  key={btn.id}
                  onClick={() => triggerQuickAction(btn.action)}
                  className={`p-2 rounded-lg border text-xs font-bold transition flex items-center justify-between gap-2 shadow hover:scale-[1.02] ${getAccentBorder(btn.color)}`}
                >
                  <span className="truncate">{btn.label}</span>
                  <Play className="w-3 h-3 shrink-0 opacity-70" />
                </button>
              ))}
            </div>

            {isEditingLayout && (
              <button
                onClick={() => setEditingButtonCardId(card.id)}
                className="w-full py-1.5 rounded bg-amber-950/60 hover:bg-amber-900/60 text-amber-300 border border-amber-800/80 text-[11px] font-bold flex items-center justify-center gap-1 transition"
              >
                <Plus className="w-3.5 h-3.5" /> Configure Buttons for this Card
              </button>
            )}
          </div>
        );

      case "telemetry_gauge": {
        const linearSpeed = robotState?.velocity
          ? Math.sqrt((robotState.velocity.x ?? 0) ** 2 + (robotState.velocity.y ?? 0) ** 2 + (robotState.velocity.z ?? 0) ** 2)
          : 0;
        const subseaDepth = Math.abs(robotState?.position?.z ?? 0);
        return (
          <div className="bg-slate-950 rounded-lg p-3 border border-slate-800 flex flex-col justify-between gap-3 h-full font-mono">
            <div className="flex items-center justify-between pr-24 border-b border-slate-800 pb-2">
              <span className={`text-xs font-bold flex items-center gap-1.5 ${getAccentHeader(card.accentColor)}`}>
                <Gauge className="w-3.5 h-3.5" /> {card.title}
              </span>
              <span className="text-[10px] text-slate-400">Subsea Sensors</span>
            </div>

            <div className="grid grid-cols-2 gap-2 text-xs">
              <div className="bg-slate-900 p-2.5 rounded border border-slate-800 flex flex-col gap-1">
                <span className="text-slate-400 text-[10px]">Speed (Forward/Reverse)</span>
                <div className="flex items-baseline gap-1">
                  <span className="text-lg font-bold text-sky-400">{linearSpeed.toFixed(2)}</span>
                  <span className="text-[10px] text-slate-400">m/s</span>
                </div>
                <div className="w-full h-1.5 bg-slate-800 rounded-full overflow-hidden">
                  <div className="h-full bg-sky-400 transition-all duration-300" style={{ width: `${Math.min(100, (linearSpeed / 2.5) * 100)}%` }} />
                </div>
              </div>

              <div className="bg-slate-900 p-2.5 rounded border border-slate-800 flex flex-col gap-1">
                <span className="text-slate-400 text-[10px]">Altitude / Subsea Depth</span>
                <div className="flex items-baseline gap-1">
                  <span className="text-lg font-bold text-emerald-400">{subseaDepth.toFixed(1)}</span>
                  <span className="text-[10px] text-slate-400">m</span>
                </div>
                <div className="w-full h-1.5 bg-slate-800 rounded-full overflow-hidden">
                  <div className="h-full bg-emerald-400 transition-all duration-300" style={{ width: `${Math.min(100, (subseaDepth / 25) * 100)}%` }} />
                </div>
              </div>

              <div className="bg-slate-900 p-2.5 rounded border border-slate-800 flex flex-col gap-1">
                <span className="text-slate-400 text-[10px]">Battery Bus Health</span>
                <div className="flex items-baseline gap-1">
                  <span className={`text-lg font-bold ${isLowBattery ? "text-rose-400" : "text-amber-300"}`}>{robotState?.battery ?? 0}%</span>
                  <span className="text-[10px] text-slate-400">SOC</span>
                </div>
                <div className="w-full h-1.5 bg-slate-800 rounded-full overflow-hidden">
                  <div className={`h-full ${isLowBattery ? "bg-rose-500" : "bg-amber-400"} transition-all duration-300`} style={{ width: `${robotState?.battery ?? 0}%` }} />
                </div>
              </div>

              <div className="bg-slate-900 p-2.5 rounded border border-slate-800 flex flex-col gap-1">
                <span className="text-slate-400 text-[10px]">Ambient Subsea Pressure</span>
                <div className="flex items-baseline gap-1">
                  <span className="text-lg font-bold text-cyan-300">{(101.3 + subseaDepth * 9.8).toFixed(1)}</span>
                  <span className="text-[10px] text-slate-400">kPa</span>
                </div>
                <div className="w-full h-1.5 bg-slate-800 rounded-full overflow-hidden">
                  <div className="h-full bg-cyan-400 transition-all duration-300" style={{ width: `${Math.min(100, ((101.3 + subseaDepth * 9.8) / 350) * 100)}%` }} />
                </div>
              </div>
            </div>
          </div>
        );
      }

      case "ros_logs":
        return (
          <div className="bg-slate-950 rounded-lg p-3 border border-slate-800 flex flex-col justify-between gap-2 h-full font-mono">
            <div className="flex items-center justify-between pr-24 border-b border-slate-800 pb-2">
              <span className={`text-xs font-bold flex items-center gap-1.5 ${getAccentHeader(card.accentColor)}`}>
                <Terminal className="w-3.5 h-3.5" /> {card.title}
              </span>
              <span className="text-[10px] text-slate-400">ROS 2 /rosout</span>
            </div>

            <div className="bg-black p-2.5 rounded border border-slate-900 text-[11px] h-[150px] overflow-y-auto flex flex-col gap-1 text-slate-300">
              <div className="text-emerald-400">[INFO] [sensor_fusion_node]: PointCloud2 received. 385,420 points aggregated.</div>
              <div className="text-sky-300">[INFO] [slam_toolbox]: Loop closure validated at Keyframe #14 (Residual: 0.002m).</div>
              <div className="text-amber-300">[WARN] [imu_driver]: Gyro thermal compensation shift +0.02 deg/s.</div>
              <div className="text-purple-300">[DEBUG] [wifi_relay]: Link bitrate 48.5 Mbps on 5.8 GHz link.</div>
              <div className="text-emerald-400">[INFO] [battery_monitor]: Voltage 24.2V nominal. Bus temperature 28.5°C.</div>
              <div className="text-sky-300">[INFO] [nav2_controller]: Path planner generated 15 waypoints through shaft chimney.</div>
            </div>
          </div>
        );

      case "sensor_chart":
        return (
          <div className="bg-slate-950 rounded-lg p-3 border border-slate-800 flex flex-col justify-between gap-2 h-full font-mono">
            <div className="flex items-center justify-between pr-24 border-b border-slate-800 pb-2">
              <span className={`text-xs font-bold flex items-center gap-1.5 ${getAccentHeader(card.accentColor)}`}>
                <BarChart2 className="w-3.5 h-3.5" /> {card.title}
              </span>
              <span className="text-[10px] text-slate-400">IMU / Power Plotter</span>
            </div>

            <div className="relative w-full aspect-video rounded overflow-hidden border border-slate-800 bg-black">
              <canvas ref={plotterCanvasRef} width={320} height={180} className="w-full h-full object-cover" />
            </div>
          </div>
        );

      default:
        return null;
    }
  };

  const visibleCardsInOrder = cardOrder
    .map((id) => cardConfigs.find((c) => c.id === id))
    .filter((c): c is DashboardCardConfig => Boolean(c && c.visible));

  return (
    <div className="w-full bg-slate-900 border border-slate-800 rounded-xl p-4 shadow-xl flex flex-col gap-4 text-slate-200">
      {/* Top Header & Editor Controls Ribbon */}
      <div className="flex flex-wrap items-center justify-between border-b border-slate-800 pb-3 gap-2">
        <div className="flex items-center gap-2">
          <Activity className="w-5 h-5 text-sky-400" />
          <div>
            <h3 className="font-semibold text-sm font-mono text-sky-400 flex items-center gap-2">
              Autonomous Multi-Sensor Perception & Power Suite
              <span className="text-[10px] font-bold bg-sky-950 text-sky-300 border border-sky-800 px-2 py-0.5 rounded">
                Editor Mode Support
              </span>
            </h3>
            <p className="text-[11px] text-slate-400 font-mono">
              Fully configurable layout, customizable sensor cards, color themes & ROS action buttons
            </p>
          </div>
        </div>

        <div className="flex flex-wrap items-center gap-2 font-mono text-xs">
          {/* Dashboard Edit Mode Toggle Button */}
          <button
            id="btn-toggle-edit-dashboard"
            onClick={() => setIsEditingLayout(!isEditingLayout)}
            className={`flex items-center gap-1.5 px-3 py-1.5 rounded-lg border font-bold transition shadow ${
              isEditingLayout
                ? "bg-amber-500 hover:bg-amber-400 text-slate-950 border-amber-400 animate-pulse"
                : "bg-sky-600 hover:bg-sky-500 text-white border-sky-400/40"
            }`}
          >
            <Sliders className="w-4 h-4" />
            {isEditingLayout ? "Exit Layout Editor" : "Edit Dashboard Layout"}
          </button>

          {/* Preset Layout Dropdown */}
          <div className="flex items-center gap-1 bg-slate-950 p-1 rounded-lg border border-slate-800">
            <span className="text-[10px] text-slate-400 px-1 font-semibold">Presets:</span>
            <button
              onClick={() => applyPresetLayout("default")}
              className="px-2 py-0.5 rounded hover:bg-slate-800 text-[11px] text-slate-300 font-semibold"
            >
              Default
            </button>
            <button
              onClick={() => applyPresetLayout("slam_focus")}
              className="px-2 py-0.5 rounded hover:bg-slate-800 text-[11px] text-sky-300 font-semibold"
            >
              3D SLAM Focus
            </button>
            <button
              onClick={() => applyPresetLayout("telemetry")}
              className="px-2 py-0.5 rounded hover:bg-slate-800 text-[11px] text-amber-300 font-semibold"
            >
              Telemetry
            </button>
          </div>

          <button
            id="btn-reset-layout-order"
            onClick={() => {
              setCardConfigs(DEFAULT_CARDS);
              setCardOrder(DEFAULT_CARDS.map((c) => c.id));
            }}
            className="flex items-center gap-1 text-[11px] bg-slate-800 hover:bg-slate-700 text-sky-300 border border-slate-700 px-2 py-1 rounded transition font-bold"
            title="Reset Dashboard Panels to Default Layout Order"
          >
            <RotateCcw className="w-3.5 h-3.5 text-sky-400" /> Reset
          </button>
        </div>
      </div>

      {/* Extended Layout Customizer Drawer (Visible when isEditingLayout is TRUE) */}
      {isEditingLayout && (
        <div className="bg-slate-950 border-2 border-amber-500/50 rounded-xl p-3.5 flex flex-col gap-3 font-mono shadow-2xl animate-fadeIn">
          <div className="flex flex-wrap items-center justify-between gap-2 border-b border-slate-800 pb-2">
            <div className="flex items-center gap-2 text-amber-400 font-bold text-xs">
              <SlidersHorizontal className="w-4 h-4 text-amber-400" />
              <span>DASHBOARD EDITOR TOOLBAR & WIDGET CATALOG</span>
            </div>

            <div className="flex flex-wrap items-center gap-2 text-xs">
              {/* + Add New Widget Button */}
              <button
                onClick={() => setShowAddWidgetModal(true)}
                className="px-3 py-1 bg-emerald-600 hover:bg-emerald-500 text-white font-bold rounded flex items-center gap-1 transition shadow"
              >
                <Plus className="w-3.5 h-3.5" /> + Add Card / Widget
              </button>

              {/* Toggle Card Visibility Drawer */}
              <button
                onClick={() => setShowVisibilityDrawer(!showVisibilityDrawer)}
                className="px-3 py-1 bg-slate-800 hover:bg-slate-700 text-sky-300 border border-slate-700 font-bold rounded flex items-center gap-1 transition"
              >
                <Eye className="w-3.5 h-3.5" /> Show / Hide Cards ({visibleCardsInOrder.length}/{cardConfigs.length})
              </button>

              {/* Export JSON */}
              <button
                onClick={exportLayoutJson}
                className="px-2.5 py-1 bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 font-bold rounded flex items-center gap-1 transition text-[11px]"
              >
                <Save className="w-3.5 h-3.5 text-sky-400" /> Export JSON
              </button>

              {/* Import JSON */}
              <label className="px-2.5 py-1 bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 font-bold rounded flex items-center gap-1 cursor-pointer transition text-[11px]">
                <Upload className="w-3.5 h-3.5 text-emerald-400" /> Import JSON
                <input type="file" accept=".json" onChange={importLayoutJson} className="hidden" />
              </label>
            </div>
          </div>

          {/* Visibility Drawer Quick Toggles */}
          {showVisibilityDrawer && (
            <div className="grid grid-cols-2 sm:grid-cols-4 gap-2 bg-slate-900 p-2.5 rounded-lg border border-slate-800 text-xs">
              {cardConfigs.map((card) => (
                <button
                  key={card.id}
                  onClick={() => toggleCardVisibility(card.id)}
                  className={`p-2 rounded border text-left flex items-center justify-between gap-1 transition ${
                    card.visible
                      ? "bg-slate-800 text-sky-300 border-sky-500/50 font-bold"
                      : "bg-slate-950 text-slate-500 border-slate-800 line-through"
                  }`}
                >
                  <span className="truncate">{card.title}</span>
                  {card.visible ? <Eye className="w-3.5 h-3.5 text-emerald-400 shrink-0" /> : <EyeOff className="w-3.5 h-3.5 text-slate-600 shrink-0" />}
                </button>
              ))}
            </div>
          )}
        </div>
      )}

      {/* Main Grid View of Reorderable & Editable Dashboard Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {visibleCardsInOrder.map((card, index) => {
          const isWide = card.span === 2;
          return (
            <div
              key={card.id}
              draggable={true}
              onDragStart={(e) => handleDragStart(e, card.id)}
              onDragOver={handleDragOver}
              onDrop={(e) => handleDrop(e, card.id)}
              className={`relative group flex flex-col rounded-xl transition-all duration-200 ${
                isWide ? "md:col-span-2" : "md:col-span-1"
              } ${
                draggedCard === card.id
                  ? "opacity-40 border-2 border-dashed border-sky-400"
                  : "hover:border-slate-700"
              }`}
            >
              {/* Card Editor Ribbon Overlay Controls */}
              <div className="absolute top-2 right-2 z-30 flex items-center gap-1 bg-slate-900/90 backdrop-blur border border-slate-700/80 p-1 rounded-md shadow-md font-mono text-xs">
                {isEditingLayout ? (
                  <>
                    {/* Width Span Selector */}
                    <button
                      onClick={() =>
                        updateCardConfig(card.id, (c) => ({
                          ...c,
                          span: c.span === 1 ? 2 : 1,
                        }))
                      }
                      className="px-1.5 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-amber-300 border border-amber-500/30 font-bold text-[10px]"
                      title="Toggle 1-Column vs 2-Column Full Width Span"
                    >
                      {card.span === 1 ? "1 Col" : "2 Cols"}
                    </button>

                    {/* Color Swatch Picker */}
                    <div className="flex items-center gap-0.5 px-1 border-x border-slate-800">
                      {(["sky", "emerald", "amber", "rose", "indigo", "cyan", "purple"] as AccentColor[]).map((col) => (
                        <button
                          key={col}
                          onClick={() => updateCardConfig(card.id, (c) => ({ ...c, accentColor: col }))}
                          className={`w-3 h-3 rounded-full border ${
                            card.accentColor === col ? "border-white scale-125 shadow" : "border-transparent"
                          }`}
                          style={{
                            backgroundColor:
                              col === "emerald"
                                ? "#10b981"
                                : col === "amber"
                                ? "#f59e0b"
                                : col === "rose"
                                ? "#f43f5e"
                                : col === "indigo"
                                ? "#6366f1"
                                : col === "cyan"
                                ? "#06b6d4"
                                : col === "purple"
                                ? "#a855f7"
                                : "#0284c7",
                          }}
                          title={`Set accent color to ${col}`}
                        />
                      ))}
                    </div>

                    {/* Hide Card Eye */}
                    <button
                      onClick={() => toggleCardVisibility(card.id)}
                      className="p-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300"
                      title="Hide card"
                    >
                      <EyeOff className="w-3 h-3 text-slate-400" />
                    </button>

                    {/* Delete Card */}
                    <button
                      onClick={() => deleteCard(card.id)}
                      className="p-1 rounded bg-rose-950/80 hover:bg-rose-900 text-rose-300 border border-rose-800/80"
                      title="Delete custom card"
                    >
                      <Trash2 className="w-3 h-3" />
                    </button>
                  </>
                ) : null}

                {/* Drag Handle */}
                <GripVertical
                  className="w-4 h-4 text-slate-400 cursor-grab active:cursor-grabbing hover:text-sky-300"
                  title="Drag and drop to rearrange panel"
                />

                {/* Left/Right Directional Shift Buttons */}
                <button
                  onClick={() => moveCard(card.id, "prev")}
                  disabled={index === 0}
                  className="p-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-30 disabled:pointer-events-none text-slate-200"
                  title="Move Panel Left / Up"
                >
                  <ArrowLeft className="w-3 h-3" />
                </button>
                <button
                  onClick={() => moveCard(card.id, "next")}
                  disabled={index === visibleCardsInOrder.length - 1}
                  className="p-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-30 disabled:pointer-events-none text-slate-200"
                  title="Move Panel Right / Down"
                >
                  <ArrowRight className="w-3 h-3" />
                </button>
              </div>

              {/* Inline Title Editor Input when Layout Editor is Active */}
              {isEditingLayout && (
                <div className="bg-amber-950/40 border-t border-x border-amber-500/40 rounded-t-xl p-1.5 flex items-center gap-2 font-mono text-xs">
                  <span className="text-[10px] text-amber-400 font-bold uppercase">Card Title:</span>
                  <input
                    type="text"
                    value={card.title}
                    onChange={(e) => updateCardConfig(card.id, (c) => ({ ...c, title: e.target.value }))}
                    className="flex-1 bg-slate-900 border border-slate-700 rounded px-2 py-0.5 text-slate-200 font-bold focus:outline-none focus:border-amber-400 text-xs"
                  />
                </div>
              )}

              {renderCardContentByConfig(card)}
            </div>
          );
        })}
      </div>

      {/* Spawn Widget Modal */}
      {showAddWidgetModal && (
        <div className="fixed inset-0 z-50 bg-slate-950/80 backdrop-blur-sm flex items-center justify-center p-4 font-mono">
          <div className="bg-slate-900 border border-slate-700 rounded-xl max-w-md w-full p-5 shadow-2xl flex flex-col gap-4 text-slate-200">
            <div className="flex items-center justify-between border-b border-slate-800 pb-2">
              <h4 className="font-bold text-sm text-sky-400 flex items-center gap-2">
                <Plus className="w-4 h-4 text-sky-400" /> Spawn New Dashboard Widget
              </h4>
              <button onClick={() => setShowAddWidgetModal(false)} className="p-1 hover:bg-slate-800 rounded">
                <X className="w-4 h-4 text-slate-400" />
              </button>
            </div>

            <p className="text-xs text-slate-400">Select a widget type to add to your custom simulation dashboard:</p>

            <div className="grid grid-cols-1 gap-2">
              {[
                { type: "telemetry_gauge" as DashboardCardType, label: "Live Telemetry Gauges", desc: "Speed, Depth, Battery & Pressure" },
                { type: "quick_actions" as DashboardCardType, label: "ROS 2 Quick Action Buttons", desc: "Configurable command triggers" },
                { type: "ros_logs" as DashboardCardType, label: "ROS 2 Diagnostic Terminal", desc: "Real-time system log feed" },
                { type: "sensor_chart" as DashboardCardType, label: "IMU & Sensor Plotter Graph", desc: "Live scrolling waveform plot" },
                { type: "camera" as DashboardCardType, label: "FPV Camera Feed", desc: "RGB visual sensor stream" },
                { type: "lidar" as DashboardCardType, label: "LiDAR Scan Map", desc: "2D laser occupancy scan" },
              ].map((item) => (
                <button
                  key={item.type}
                  onClick={() => spawnNewWidget(item.type)}
                  className="p-3 bg-slate-950 hover:bg-slate-800 border border-slate-800 hover:border-sky-500/50 rounded-lg text-left flex items-center justify-between group transition"
                >
                  <div>
                    <strong className="text-xs font-bold text-sky-300 group-hover:text-white block">{item.label}</strong>
                    <span className="text-[10px] text-slate-400">{item.desc}</span>
                  </div>
                  <Plus className="w-4 h-4 text-sky-400 group-hover:scale-125 transition" />
                </button>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Button Configurator Modal */}
      {editingButtonCardId && (
        <div className="fixed inset-0 z-50 bg-slate-950/80 backdrop-blur-sm flex items-center justify-center p-4 font-mono">
          <div className="bg-slate-900 border border-slate-700 rounded-xl max-w-lg w-full p-5 shadow-2xl flex flex-col gap-4 text-slate-200">
            <div className="flex items-center justify-between border-b border-slate-800 pb-2">
              <h4 className="font-bold text-sm text-amber-400 flex items-center gap-2">
                <Settings className="w-4 h-4 text-amber-400" /> Configure ROS 2 Action Buttons
              </h4>
              <button onClick={() => setEditingButtonCardId(null)} className="p-1 hover:bg-slate-800 rounded">
                <X className="w-4 h-4 text-slate-400" />
              </button>
            </div>

            {/* Existing Buttons List */}
            <div className="flex flex-col gap-2">
              <span className="text-xs font-bold text-slate-300">Existing Buttons:</span>
              <div className="flex flex-col gap-1.5 max-h-40 overflow-y-auto bg-slate-950 p-2 rounded border border-slate-800">
                {(cardConfigs.find((c) => c.id === editingButtonCardId)?.quickButtons || []).map((btn) => (
                  <div key={btn.id} className="flex items-center justify-between bg-slate-900 p-2 rounded border border-slate-800 text-xs">
                    <div>
                      <strong className="text-sky-300">{btn.label}</strong>
                      <span className="text-[10px] text-slate-500 block">Action: {btn.action}</span>
                    </div>
                    <button
                      onClick={() => deleteButtonFromCard(editingButtonCardId, btn.id)}
                      className="p-1 bg-rose-950 hover:bg-rose-900 text-rose-300 rounded border border-rose-800"
                      title="Remove button"
                    >
                      <Trash2 className="w-3.5 h-3.5" />
                    </button>
                  </div>
                ))}
              </div>
            </div>

            {/* Add New Button Form */}
            <div className="flex flex-col gap-2 pt-2 border-t border-slate-800">
              <span className="text-xs font-bold text-amber-300">+ Add Custom Button:</span>
              <div className="flex flex-col gap-2 text-xs">
                <input
                  type="text"
                  placeholder="Button Label (e.g., Return to Base)"
                  value={newBtnLabel}
                  onChange={(e) => setNewBtnLabel(e.target.value)}
                  className="bg-slate-950 border border-slate-700 rounded p-2 text-slate-200 focus:outline-none focus:border-amber-400"
                />

                <div className="flex items-center gap-2">
                  <span className="text-[10px] text-slate-400 w-20">ROS Action:</span>
                  <select
                    value={newBtnAction}
                    onChange={(e) => setNewBtnAction(e.target.value)}
                    className="flex-1 bg-slate-950 border border-slate-700 rounded p-2 text-slate-200 focus:outline-none focus:border-amber-400"
                  >
                    <option value="toggle_headlight">Toggle Headlight</option>
                    <option value="toggle_sonar">Trigger Sonar Ping</option>
                    <option value="toggle_wifi">WiFi Relay Toggle</option>
                    <option value="battery_full">Recharge Battery (100%)</option>
                    <option value="emergency_stop">Emergency Stop Lock</option>
                    <option value="calibrate_imu">Calibrate Gyro IMU</option>
                    <option value="record_video">Record FPV Video</option>
                    <option value="export_map">Export PCD PointCloud Map</option>
                  </select>
                </div>

                <div className="flex items-center gap-2">
                  <span className="text-[10px] text-slate-400 w-20">Button Style:</span>
                  <select
                    value={newBtnColor}
                    onChange={(e) => setNewBtnColor(e.target.value as AccentColor)}
                    className="flex-1 bg-slate-950 border border-slate-700 rounded p-2 text-slate-200 focus:outline-none focus:border-amber-400"
                  >
                    <option value="sky">Sky Blue</option>
                    <option value="emerald">Emerald Green</option>
                    <option value="amber">Amber Gold</option>
                    <option value="rose">Rose Red</option>
                    <option value="indigo">Indigo Purple</option>
                    <option value="cyan">Cyan</option>
                  </select>
                </div>

                <button
                  onClick={() => addCustomButtonToCard(editingButtonCardId)}
                  disabled={!newBtnLabel.trim()}
                  className="mt-1 py-2 bg-amber-600 hover:bg-amber-500 disabled:opacity-40 text-slate-950 font-bold rounded transition shadow"
                >
                  Add Button to Card
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};


