const fs = require('fs');
let code = fs.readFileSync('src/components/GazeboSimViewport.tsx', 'utf-8');

const refsStr = `  const lidarMeshRef = useRef<THREE.Group | null>(null);
  const laserRaysRef = useRef<THREE.LineSegments | null>(null);
  const sonarPulseMeshRef = useRef<THREE.Mesh | null>(null);
  const waterMeshRef = useRef<THREE.Mesh | null>(null);
  const slamVoxelMeshRef = useRef<THREE.InstancedMesh | null>(null);
  const exploredVoxelsRef = useRef<Set<string>>(new Set());
  const voxelCountRef = useRef<number>(0);
  const tempMatrix = new THREE.Matrix4();`;

code = code.replace(/  const lidarMeshRef = useRef<THREE.Group \| null>\(null\);\n  const laserRaysRef = useRef<THREE.LineSegments \| null>\(null\);\n  const sonarPulseMeshRef = useRef<THREE.Mesh \| null>\(null\);\n  const waterMeshRef = useRef<THREE.Mesh \| null>\(null\);/, refsStr);

const sceneAddStr = `    scene.add(caveGroup);

    // ==========================================
    // 5.5 SLAM 3D MESH RECONSTRUCTION
    // ==========================================
    const maxVoxels = 20000;
    const voxelGeo = new THREE.BoxGeometry(0.5, 0.5, 0.5);
    const voxelMat = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      roughness: 0.8,
      metalness: 0.2,
      transparent: true,
      opacity: 0.9,
    });
    const slamVoxelMesh = new THREE.InstancedMesh(voxelGeo, voxelMat, maxVoxels);
    slamVoxelMesh.count = 0;
    slamVoxelMesh.castShadow = true;
    slamVoxelMesh.receiveShadow = true;
    scene.add(slamVoxelMesh);
    slamVoxelMeshRef.current = slamVoxelMesh;`;

code = code.replace('    scene.add(caveGroup);', sceneAddStr);

fs.writeFileSync('src/components/GazeboSimViewport.tsx', code);
