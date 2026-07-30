import { ROS2File } from "../types";

export function downloadFile(filename: string, text: string) {
  const element = document.createElement("a");
  const file = new Blob([text], { type: "text/plain" });
  element.href = URL.createObjectURL(file);
  element.download = filename;
  document.body.appendChild(element);
  element.click();
  document.body.removeChild(element);
}

export function downloadFullWorkspaceAsShellScript(files: ROS2File[]) {
  let script = `#!/bin/bash
# Auto-generated ROS 2 Jazzy Hybrid Drone Workspace Builder Script
set -e

echo "Creating ROS 2 Jazzy Hybrid Drone Workspace..."
mkdir -p hybrid_drone_ws/src/hybrid_cave_drone/launch
mkdir -p hybrid_drone_ws/src/hybrid_cave_drone/urdf
mkdir -p hybrid_drone_ws/src/hybrid_cave_drone/config
mkdir -p hybrid_drone_ws/src/hybrid_cave_drone/src

`;

  files.forEach((file) => {
    script += `cat << 'EOF' > ${file.path}
${file.content}
EOF
chmod +x ${file.path} 2>/dev/null || true

`;
  });

  script += `echo "ROS 2 Jazzy Workspace generated successfully!"
echo "To build: cd hybrid_drone_ws && colcon build"
`;

  downloadFile("setup_ros2_workspace.sh", script);
}
