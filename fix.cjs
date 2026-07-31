const fs = require('fs');
let code = fs.readFileSync('src/components/GazeboSimViewport.tsx', 'utf-8');
code = code.replace(/const currentCamMode = activeCameraModeRef.current;\n        if \(currentCamMode === "topdown"\) \{/g, 'if (currentCamMode === "topdown") {');
code = code.replace(/\} else const currentCamMode = activeCameraModeRef.current;\n        if \(currentCamMode === "topdown"\) \{/g, '} else if (currentCamMode === "topdown") {');
fs.writeFileSync('src/components/GazeboSimViewport.tsx', code);
