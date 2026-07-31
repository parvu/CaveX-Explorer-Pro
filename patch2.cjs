const fs = require('fs');
let code = fs.readFileSync('src/components/GazeboSimViewport.tsx', 'utf-8');

const targetStr = `      // Update Robot Kinematics according to Mode & Position`;

const replacement = `      // SLAM 3D MESH RECONSTRUCTION UPDATE
      if (robotGroupRef.current && slamVoxelMeshRef.current) {
        if (currentCamMode === "topdown") {
           slamVoxelMeshRef.current.visible = true;
        } else {
           slamVoxelMeshRef.current.visible = false;
        }

        const rx = robotGroupRef.current.position.x;
        const ry = robotGroupRef.current.position.y;
        const rz = robotGroupRef.current.position.z;

        // Reset SLAM Map on Simulation Reset
        if (Math.abs(rx - -15) < 0.1 && Math.abs(ry - 0.6) < 0.1 && Math.abs(rz - 0) < 0.1) {
           exploredVoxelsRef.current.clear();
           voxelCountRef.current = 0;
           slamVoxelMeshRef.current.count = 0;
        }

        // Generate Voxels around the robot
        const maxVoxels = 20000;
        let vCount = voxelCountRef.current;
        let needsUpdate = false;

        for (let dx = -8; dx <= 8; dx += 1) {
          for (let dy = -6; dy <= 6; dy += 1) {
            for (let dz = -6; dz <= 6; dz += 1) {
              if (dx*dx + dy*dy + dz*dz <= 49) { // 7m sphere
                 const gx = Math.round(rx + dx);
                 const gy = Math.round(ry + dy); // altitude
                 const gz = Math.round(rz + dz); // lateral
                 
                 let isSolid = false;
                 // Cave bounds estimation
                 if (gx < 30) {
                    if (gy < (gx >= -5 && gx <= 29 ? -1 : 0)) isSolid = true; // floor
                    if (gz > 3 || gz < -3) isSolid = true; // walls
                 } else {
                    if (gy < 0) isSolid = true; // floor under shaft
                    if (gz*gz + (gx-35.5)*(gx-35.5) > 25) isSolid = true; // cylinder shaft wall
                 }
                 
                 if (isSolid) {
                    const key = \`\${gx},\${gy},\${gz}\`;
                    if (!exploredVoxelsRef.current.has(key) && vCount < maxVoxels) {
                       exploredVoxelsRef.current.add(key);
                       tempMatrix.setPosition(gx, gy, gz);
                       slamVoxelMeshRef.current.setMatrixAt(vCount, tempMatrix);
                       
                       const color = new THREE.Color();
                       // Use altitude (gy) for color shading
                       color.setHSL((gy + 10) / 40, 0.8, 0.6);
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

      // Update Robot Kinematics according to Mode & Position`;

code = code.replace('      // Update Robot Kinematics according to Mode & Position', replacement);
fs.writeFileSync('src/components/GazeboSimViewport.tsx', code);
