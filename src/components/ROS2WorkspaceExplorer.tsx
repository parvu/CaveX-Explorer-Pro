import React, { useState } from "react";
import { ROS2File } from "../types";
import { ROS2_JAZZY_WORKSPACE_FILES } from "../data/ros2WorkspaceData";
import { downloadFullWorkspaceAsShellScript, downloadFile } from "../utils/workspaceExport";
import { Folder, FileCode, Download, Copy, Check, Terminal, FileText, Code2, Sparkles } from "lucide-react";

interface ROS2WorkspaceExplorerProps {
  onOpenAICopilot?: (file: ROS2File) => void;
}

export const ROS2WorkspaceExplorer: React.FC<ROS2WorkspaceExplorerProps> = ({ onOpenAICopilot }) => {
  const [files, setFiles] = useState<ROS2File[]>(ROS2_JAZZY_WORKSPACE_FILES);
  const [selectedFilePath, setSelectedFilePath] = useState<string>(ROS2_JAZZY_WORKSPACE_FILES[0].path);
  const [copied, setCopied] = useState(false);

  const selectedFile = files.find((f) => f.path === selectedFilePath) || files[0];

  const handleContentChange = (newContent: string) => {
    setFiles((prev) =>
      prev.map((f) => (f.path === selectedFilePath ? { ...f, content: newContent } : f))
    );
  };

  const handleCopyCode = () => {
    navigator.clipboard.writeText(selectedFile.content);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className="w-full bg-slate-900 border border-slate-800 rounded-xl p-4 shadow-xl flex flex-col gap-4 text-slate-200">
      <div className="flex flex-wrap items-center justify-between border-b border-slate-800 pb-3 gap-2">
        <div>
          <h3 className="font-semibold text-base font-mono flex items-center gap-2 text-sky-400">
            <Folder className="w-5 h-5 text-sky-400" />
            ROS 2 Jazzy Package Workspace Explorer & Code Generator
          </h3>
          <p className="text-xs text-slate-400 font-sans mt-0.5">
            Full colcon build compatible package structure: launch files, python nodes, C++ hydrodynamics plugin, URDF, and Nav2 YAMLs.
          </p>
        </div>

        {/* Download Buttons */}
        <div className="flex items-center gap-2">
          <button
            id="btn-download-file"
            onClick={() => downloadFile(selectedFile.name, selectedFile.content)}
            className="px-3 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 text-xs font-mono font-medium flex items-center gap-1.5 transition"
          >
            <Download className="w-3.5 h-3.5 text-sky-400" /> Save {selectedFile.name}
          </button>
          <button
            id="btn-export-workspace"
            onClick={() => downloadFullWorkspaceAsShellScript(files)}
            className="px-3 py-1.5 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-white text-xs font-mono font-bold flex items-center gap-1.5 transition shadow-lg shadow-emerald-600/20"
          >
            <Terminal className="w-3.5 h-3.5" /> Export ROS 2 Workspace (Shell Script)
          </button>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
        {/* Left: File Tree Sidebar */}
        <div className="md:col-span-1 bg-slate-950 rounded-lg p-2.5 border border-slate-800 flex flex-col gap-1 font-mono text-xs">
          <span className="text-[11px] text-slate-400 font-semibold px-2 py-1 flex items-center gap-1.5 border-b border-slate-800 mb-1">
            <Folder className="w-3.5 h-3.5 text-sky-400" /> hybrid_cave_drone/
          </span>

          <div className="space-y-0.5 overflow-y-auto max-h-[380px]">
            {files.map((file) => (
              <button
                key={file.path}
                onClick={() => setSelectedFilePath(file.path)}
                className={`w-full text-left px-2.5 py-1.5 rounded flex items-center justify-between text-xs transition ${
                  selectedFilePath === file.path
                    ? "bg-sky-950/80 text-sky-300 font-bold border border-sky-800"
                    : "text-slate-400 hover:bg-slate-900 hover:text-slate-200"
                }`}
              >
                <span className="truncate flex items-center gap-1.5">
                  <FileCode className="w-3.5 h-3.5 shrink-0 text-slate-500" />
                  {file.name}
                </span>
                <span className="text-[10px] text-slate-600 uppercase font-sans">{file.language}</span>
              </button>
            ))}
          </div>
        </div>

        {/* Right: Code Viewer & Editor */}
        <div className="md:col-span-3 bg-slate-950 rounded-lg p-3 border border-slate-800 flex flex-col gap-2 font-mono text-xs">
          <div className="flex items-center justify-between border-b border-slate-800/80 pb-2">
            <div className="flex items-center gap-2 text-slate-300 font-semibold">
              <FileText className="w-4 h-4 text-sky-400" />
              <span>{selectedFile.path}</span>
            </div>

            <div className="flex items-center gap-2">
              {onOpenAICopilot && (
                <button
                  onClick={() => onOpenAICopilot(selectedFile)}
                  className="px-2.5 py-1 rounded bg-purple-950/80 hover:bg-purple-900 text-purple-300 border border-purple-800 text-[11px] font-semibold flex items-center gap-1 transition"
                >
                  <Sparkles className="w-3 h-3 text-purple-400" /> Ask AI to Optimize
                </button>
              )}
              <button
                onClick={handleCopyCode}
                className="px-2.5 py-1 rounded bg-slate-900 hover:bg-slate-800 text-slate-300 border border-slate-700 text-[11px] flex items-center gap-1 transition"
              >
                {copied ? <Check className="w-3 h-3 text-emerald-400" /> : <Copy className="w-3 h-3" />}
                {copied ? "Copied!" : "Copy Code"}
              </button>
            </div>
          </div>

          <textarea
            value={selectedFile.content}
            onChange={(e) => handleContentChange(e.target.value)}
            rows={16}
            className="w-full bg-slate-950 text-slate-200 font-mono text-xs p-3 focus:outline-none rounded border border-slate-800/60 leading-relaxed resize-y"
            spellCheck={false}
          />
        </div>
      </div>
    </div>
  );
};
