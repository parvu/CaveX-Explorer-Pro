import express from "express";
import path from "path";
import { createServer as createViteServer } from "vite";
import { GoogleGenAI } from "@google/genai";
import dotenv from "dotenv";

dotenv.config();

async function startServer() {
  const app = express();
  const PORT = 3000;

  app.use(express.json({ limit: "10mb" }));

  // Initialize Gemini AI Client lazily or safely
  let aiClient: GoogleGenAI | null = null;
  function getAiClient() {
    if (!aiClient) {
      const apiKey = process.env.GEMINI_API_KEY;
      if (!apiKey) {
        throw new Error("GEMINI_API_KEY environment variable is not configured.");
      }
      aiClient = new GoogleGenAI({
        apiKey,
        httpOptions: {
          headers: {
            "User-Agent": "aistudio-build",
          },
        },
      });
    }
    return aiClient;
  }

  // Health check endpoint
  app.get("/api/health", (req, res) => {
    res.json({ status: "ok", timestamp: new Date().toISOString() });
  });

  // Live telemetry sink for the ROS2 stack (see web_telemetry_bridge.py),
  // replacing the desktop Gazebo/rtabmap_viz GUIs. In-memory only -- this
  // is a display cache, not a store of record.
  let latestTelemetry: { data: unknown; receivedAt: number } | null = null;
  app.post("/api/telemetry", (req, res) => {
    latestTelemetry = { data: req.body, receivedAt: Date.now() };
    res.json({ status: "ok" });
  });
  app.get("/api/telemetry", (req, res) => {
    if (!latestTelemetry || Date.now() - latestTelemetry.receivedAt > 5000) {
      return res.json({ live: false });
    }
    res.json({ live: true, ...latestTelemetry });
  });

  // Reverse direction: web UI -> ROS2 waypoint goals. Push queues a single
  // pending goal (last write wins -- one robot, one active goal); GET pops
  // it (returns it once, then clears) -- web_telemetry_bridge.py polls this
  // and republishes it as a PoseStamped on /cavex/nav/goal for
  // waypoint_follower.py to drive to.
  let pendingGoal: { x: number; y: number } | null = null;
  app.post("/api/waypoint-goal", (req, res) => {
    const { x, y } = req.body ?? {};
    if (typeof x !== "number" || typeof y !== "number") {
      return res.status(400).json({ error: "x and y (numbers) are required." });
    }
    pendingGoal = { x, y };
    res.json({ status: "ok" });
  });
  app.get("/api/waypoint-goal", (req, res) => {
    const goal = pendingGoal;
    pendingGoal = null;
    res.json({ goal });
  });

  // AI Assistant endpoint for ROS 2 Jazzy node & URDF code generation
  app.post("/api/gemini/ros2-assistant", async (req, res) => {
    try {
      const { prompt, systemPrompt } = req.body;
      if (!prompt) {
        return res.status(400).json({ error: "Prompt is required." });
      }

      const apiKey = process.env.GEMINI_API_KEY;
      if (!apiKey || apiKey === "MY_GEMINI_API_KEY") {
        // Graceful fallback when API Key is missing or default placeholder
        return res.json({
          text: `// [AI COPILOT NOTICE] GEMINI_API_KEY is not configured in the environment.
// Using offline ROS 2 Jazzy template generator for prompt: "${prompt}"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class CaveExplorationNode(Node):
    """
    ROS 2 Jazzy Node for Tri-Modal Autonomous Cave Exploration
    Supports: Walking (Quadruped), Sailing (Hydrofoil), Flying (Quadrotor)
    """
    def __init__(self):
        super().__init__('cave_exploration_copilot_node')
        
        # Publishers & Subscribers
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sonar_sub = self.create_subscription(Range, '/sonar/range', self.sonar_callback, 10)
        
        self.current_mode = "WALKING"
        self.timer = self.create_timer(1.0, self.publish_telemetry)
        self.get_logger().info("CaveExplorationNode initialized on ROS 2 Jazzy Harmonic.")

    def scan_callback(self, msg: LaserScan):
        min_range = min(msg.ranges) if msg.ranges else 999.0
        if min_range < 0.5 and self.current_mode == "WALKING":
            self.get_logger().warn(f"Obstacle detected at {min_range:.2f}m! Triggering FSM recalculation.")

    def sonar_callback(self, msg: Range):
        if msg.range < 0.1 and self.current_mode != "SAILING":
            self.get_logger().info("Water contact detected! Deploying hydrofoil pontoons.")
            self.current_mode = "SAILING"

    def publish_telemetry(self):
        msg = String()
        msg.data = f"MODE:{self.current_mode}"
        self.mode_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CaveExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`,
        });
      }

      const ai = getAiClient();
      const response = await ai.models.generateContent({
        model: "gemini-3.6-flash",
        contents: prompt,
        config: {
          systemInstruction:
            systemPrompt ||
            "You are an expert Robotics Engineer specializing in ROS 2 Jazzy, Gazebo Harmonic/Garden, URDF/Xacro modeling, and multi-modal autonomous underwater/aerial vehicles.",
          temperature: 0.7,
        },
      });

      return res.json({ text: response.text });
    } catch (error: any) {
      console.error("Gemini API Error:", error);
      return res.status(500).json({
        error: error.message || "Failed to communicate with Gemini API",
      });
    }
  });

  // Vite middleware in development mode
  if (process.env.NODE_ENV !== "production") {
    const vite = await createViteServer({
      server: { middlewareMode: true },
      appType: "spa",
    });
    app.use(vite.middlewares);
  } else {
    const distPath = path.join(process.cwd(), "dist");
    app.use(express.static(distPath));
    app.get("*", (req, res) => {
      res.sendFile(path.join(distPath, "index.html"));
    });
  }

  app.listen(PORT, "0.0.0.0", () => {
    console.log(`ROS 2 Jazzy Simulator Server running on http://0.0.0.0:${PORT}`);
  });
}

startServer().catch((err) => {
  console.error("Failed to start server:", err);
});
