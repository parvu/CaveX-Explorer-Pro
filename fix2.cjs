const fs = require('fs');
let code = fs.readFileSync('src/components/GazeboSimViewport.tsx', 'utf-8');

const targetStr = `      // SLAM 3D MESH RECONSTRUCTION UPDATE
      const currentCamMode = activeCameraModeRef.current;
      if (robotGroupRef.current if (robotGroupRef.current && slamVoxelMeshRef.current) {if (robotGroupRef.current && slamVoxelMeshRef.current) { slamVoxelMeshRef.current) {`;

code = code.replace(targetStr, `      // SLAM 3D MESH RECONSTRUCTION UPDATE
      const currentCamMode = activeCameraModeRef.current;
      if (robotGroupRef.current && slamVoxelMeshRef.current) {`);

fs.writeFileSync('src/components/GazeboSimViewport.tsx', code);
