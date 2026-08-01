import React, { useState } from "react";
import { RobotState, JointState } from "../types";
import { Sliders, Code2, Layers, Cpu, ShieldCheck, RefreshCw } from "lucide-react";

interface KinematicChainEditorProps {
  robotState: RobotState;
  setRobotState: React.Dispatch<React.SetStateAction<RobotState>>;
}

export const KinematicChainEditor: React.FC<KinematicChainEditorProps> = ({
  robotState,
  setRobotState,
}) => {
  const [activeTab, setActiveTab] = useState<"joints" | "urdf_xacro">("joints");
  const [urdfCode, setUrdfCode] = useState<string>(`<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="hybrid_tri_drone">
  <link name="base_link">
    <inertial>
      <mass value="4.2"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.11" iyz="0.0" izz="0.15"/>
    </inertial>
  </link>

  <!-- Kinematic Chain 1: Quadruped Legs -->
  <joint name="hip_fl_joint" type="revolute">
    <axis xyz="1 0 0"/>
    <limit lower="-0.8" upper="0.8" effort="35.0" velocity="4.0"/>
  </joint>

  <!-- Kinematic Chain 2: Hydrofoil Pontoons -->
  <joint name="pontoon_left_joint" type="revolute">
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="0.0" effort="20.0" velocity="2.0"/>
  </joint>

  <!-- Kinematic Chain 3: Quadrotor Rotors -->
  <joint name="rotor_fl_joint" type="continuous">
    <axis xyz="0 0 1"/>
    <limit effort="15.0" velocity="100.0"/>
  </joint>
</robot>`);

  const handleJointChange = (jointName: string, val: number) => {
    setRobotState((prev) => ({
      ...prev,
      jointStates: {
        ...prev.jointStates,
        [jointName]: val,
      },
    }));
  };

  const jointDefinitions: JointState[] = [
    { name: "hip_fl_joint", angle: robotState.jointStates["hip_fl_joint"] ?? 0, min: -0.8, max: 0.8, type: "revolute", chain: "leg_fl" },
    { name: "thigh_fl_joint", angle: robotState.jointStates["thigh_fl_joint"] ?? -0.2, min: -1.2, max: 1.2, type: "revolute", chain: "leg_fl" },
    { name: "shank_fl_joint", angle: robotState.jointStates["shank_fl_joint"] ?? -0.5, min: -2.0, max: 0.5, type: "revolute", chain: "leg_fl" },
    { name: "pontoon_left_joint", angle: robotState.jointStates["pontoon_left_joint"] ?? -0.15, min: -1.57, max: 0.0, type: "revolute", chain: "hydrofoil" },
    { name: "pontoon_right_joint", angle: robotState.jointStates["pontoon_right_joint"] ?? 0.15, min: 0.0, max: 1.57, type: "revolute", chain: "hydrofoil" },
    { name: "rotor_fl_joint", angle: robotState.jointStates["rotor_fl_joint"] ?? 0, min: 0, max: 6.28, type: "continuous", chain: "rotor" },
  ];

  return (
    <div className="w-full bg-slate-900 border border-slate-800 rounded-xl p-4 shadow-xl flex flex-col gap-4 text-slate-200">
      <div className="flex items-center justify-between border-b border-slate-800 pb-3">
        <h3 className="font-semibold text-sm font-mono flex items-center gap-2 text-purple-400">
          <Cpu className="w-4 h-4 text-purple-400" />
          URDF Kinematic Chain Controller & Limit Inspector
        </h3>

        {/* Tab switcher */}
        <div className="flex items-center gap-1 bg-slate-950 p-1 rounded-lg border border-slate-800 text-xs font-mono">
          <button
            id="tab-joint-limits"
            onClick={() => setActiveTab("joints")}
            className={`px-3 py-1 rounded transition flex items-center gap-1.5 ${
              activeTab === "joints" ? "bg-purple-600 text-white font-bold" : "text-slate-400 hover:text-slate-200"
            }`}
          >
            <Sliders className="w-3.5 h-3.5" /> Joint Limits & Chains
          </button>
          <button
            id="tab-urdf-xml"
            onClick={() => setActiveTab("urdf_xacro")}
            className={`px-3 py-1 rounded transition flex items-center gap-1.5 ${
              activeTab === "urdf_xacro" ? "bg-purple-600 text-white font-bold" : "text-slate-400 hover:text-slate-200"
            }`}
          >
            <Code2 className="w-3.5 h-3.5" /> URDF Xacro XML
          </button>
        </div>
      </div>

      {activeTab === "joints" ? (
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 font-mono text-xs">
          {/* Active Kinematic Constraints Summary */}
          <div className="bg-slate-950 p-3 rounded-lg border border-slate-800 flex flex-col gap-2">
            <span className="text-slate-400 font-semibold flex items-center gap-1.5 text-purple-300">
              <ShieldCheck className="w-4 h-4 text-purple-400" /> Active Kinematic Constraints ({robotState.mode})
            </span>
            <div className="space-y-1.5 text-slate-300">
              <div className="flex justify-between p-1.5 bg-slate-900 rounded border border-slate-800">
                <span>Quadruped Leg Motors:</span>
                <span className={robotState.mode === "WALKING" ? "text-amber-400 font-bold" : "text-slate-500"}>
                  {robotState.mode === "WALKING" ? "ENABLED (12 DOF)" : "LOCKED"}
                </span>
              </div>
              <div className="flex justify-between p-1.5 bg-slate-900 rounded border border-slate-800">
                <span>Hydrofoil Pontoon Rudders:</span>
                <span className={robotState.mode === "SAILING" ? "text-cyan-400 font-bold" : "text-slate-500"}>
                  {robotState.mode === "SAILING" ? "DEPLOYED (-0.15m)" : "RETRACTED"}
                </span>
              </div>
              <div className="flex justify-between p-1.5 bg-slate-900 rounded border border-slate-800">
                <span>Quadrotor Aerial Thrusters:</span>
                <span className={robotState.mode === "FLYING" ? "text-purple-400 font-bold" : "text-slate-500"}>
                  {robotState.mode === "FLYING" ? "HIGH RPM (4200 RPM)" : "IDLE"}
                </span>
              </div>
            </div>
          </div>

          {/* Joint Sliders */}
          <div className="bg-slate-950 p-3 rounded-lg border border-slate-800 flex flex-col gap-3">
            <span className="text-slate-400 font-semibold">URDF Joint Limit & Offset Sliders</span>
            <div className="space-y-3">
              {jointDefinitions.map((j) => (
                <div key={j.name} className="flex flex-col gap-1">
                  <div className="flex justify-between text-[11px] text-slate-400">
                    <span>{j.name} ({j.chain})</span>
                    <strong className="text-purple-300">{(j.angle ?? 0).toFixed(2)} rad</strong>
                  </div>
                  <input
                    type="range"
                    min={j.min}
                    max={j.max}
                    step={0.01}
                    value={j.angle}
                    onChange={(e) => handleJointChange(j.name, parseFloat(e.target.value))}
                    className="w-full accent-purple-500 bg-slate-800 h-1.5 rounded-lg cursor-pointer"
                  />
                  <div className="flex justify-between text-[10px] text-slate-500">
                    <span>Min: {j.min}</span>
                    <span>Max: {j.max}</span>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      ) : (
        <div className="flex flex-col gap-2 font-mono text-xs">
          <span className="text-slate-400 font-semibold">URDF Xacro Specification Editor</span>
          <textarea
            value={urdfCode}
            onChange={(e) => setUrdfCode(e.target.value)}
            rows={12}
            className="w-full bg-slate-950 text-purple-200 border border-slate-800 rounded-lg p-3 font-mono text-xs focus:outline-none focus:border-purple-500"
          />
        </div>
      )}
    </div>
  );
};
