import React, { useState } from "react";
import { ROS2File } from "../types";
import { Sparkles, Send, X, Bot, Check, Copy, Loader2, Code2 } from "lucide-react";

interface AICopilotModalProps {
  isOpen: boolean;
  onClose: () => void;
  activeFile?: ROS2File;
}

export const AICopilotModal: React.FC<AICopilotModalProps> = ({ isOpen, onClose, activeFile }) => {
  const [prompt, setPrompt] = useState(
    activeFile
      ? `Generate a ROS 2 Jazzy node extension for ${activeFile.name} to handle multi-modal transition telemetry in flooded caves.`
      : "Write a ROS 2 Jazzy Python node that fuses underwater Sonar depth readings with 3D LiDAR ranges for flooded cave SLAM."
  );
  const [loading, setLoading] = useState(false);
  const [response, setResponse] = useState("");
  const [copied, setCopied] = useState(false);

  if (!isOpen) return null;

  const handleGenerate = async () => {
    if (!prompt.trim()) return;
    setLoading(true);
    setResponse("");

    try {
      const res = await fetch("/api/gemini/ros2-assistant", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          prompt: `${prompt}\n\nExisting File Context:\n${activeFile ? activeFile.content : ""}`,
        }),
      });

      const data = await res.json();
      if (data.error) {
        setResponse(`Error: ${data.error}`);
      } else {
        setResponse(data.text);
      }
    } catch (err: any) {
      setResponse(`Error connecting to server: ${err.message}`);
    } finally {
      setLoading(false);
    }
  };

  const handleCopy = () => {
    navigator.clipboard.writeText(response);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className="fixed inset-0 z-50 bg-slate-950/80 backdrop-blur-sm flex items-center justify-center p-4">
      <div className="bg-slate-900 border border-slate-800 rounded-xl w-full max-w-3xl max-h-[85vh] flex flex-col shadow-2xl overflow-hidden text-slate-200">
        {/* Modal Header */}
        <div className="flex items-center justify-between p-4 border-b border-slate-800 bg-slate-950">
          <div className="flex items-center gap-2">
            <div className="w-8 h-8 rounded-lg bg-purple-600/20 border border-purple-500/40 flex items-center justify-center text-purple-400">
              <Sparkles className="w-4 h-4" />
            </div>
            <div>
              <h3 className="font-semibold text-sm font-mono text-purple-300">
                ROS 2 Jazzy AI Copilot (Gemini 3.6 Flash)
              </h3>
              <p className="text-xs text-slate-400 font-sans">
                Generates ROS 2 C++/Python nodes, tunes Nav2 params, and modifies URDF models.
              </p>
            </div>
          </div>

          <button
            onClick={onClose}
            className="p-1 rounded text-slate-400 hover:text-white hover:bg-slate-800 transition"
          >
            <X className="w-5 h-5" />
          </button>
        </div>

        {/* Prompt Input */}
        <div className="p-4 flex flex-col gap-3 bg-slate-900 border-b border-slate-800 font-mono text-xs">
          <label className="text-slate-400 font-semibold">Prompt Instructions:</label>
          <div className="flex gap-2">
            <input
              type="text"
              value={prompt}
              onChange={(e) => setPrompt(e.target.value)}
              onKeyDown={(e) => e.key === "Enter" && handleGenerate()}
              placeholder="Ask AI to generate a node, tune SLAM, or adjust joint limits..."
              className="flex-1 bg-slate-950 border border-slate-800 rounded-lg px-3 py-2 text-slate-200 text-xs focus:outline-none focus:border-purple-500"
            />
            <button
              onClick={handleGenerate}
              disabled={loading}
              className="px-4 py-2 bg-purple-600 hover:bg-purple-500 text-white rounded-lg font-bold flex items-center gap-1.5 transition disabled:opacity-50 shadow-lg shadow-purple-600/20"
            >
              {loading ? <Loader2 className="w-4 h-4 animate-spin" /> : <Send className="w-4 h-4" />}
              {loading ? "Generating..." : "Generate"}
            </button>
          </div>
        </div>

        {/* AI Output Stream View */}
        <div className="p-4 flex-1 overflow-y-auto font-mono text-xs bg-slate-950 flex flex-col gap-2">
          {response ? (
            <div className="flex flex-col gap-2">
              <div className="flex items-center justify-between border-b border-slate-800 pb-2">
                <span className="text-purple-400 font-semibold flex items-center gap-1.5">
                  <Bot className="w-4 h-4" /> AI Response Code:
                </span>
                <button
                  onClick={handleCopy}
                  className="px-2.5 py-1 rounded bg-slate-900 hover:bg-slate-800 text-slate-300 border border-slate-700 text-[11px] flex items-center gap-1"
                >
                  {copied ? <Check className="w-3 h-3 text-emerald-400" /> : <Copy className="w-3 h-3" />}
                  {copied ? "Copied!" : "Copy"}
                </button>
              </div>
              <pre className="whitespace-pre-wrap text-slate-300 bg-slate-900/60 p-3 rounded border border-slate-800/80 leading-relaxed overflow-x-auto">
                {response}
              </pre>
            </div>
          ) : (
            <div className="h-48 flex flex-col items-center justify-center text-slate-500 gap-2">
              <Code2 className="w-8 h-8 text-slate-600" />
              <p>Type a prompt above to generate ROS 2 Jazzy code & Gazebo parameters.</p>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
