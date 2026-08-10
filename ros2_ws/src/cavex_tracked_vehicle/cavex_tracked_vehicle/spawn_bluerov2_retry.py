#!/usr/bin/env python3
"""
spawn_bluerov2_retry.py

Real request: "rov2 not spawen[ing] [always], try something else." The
original spawn_bluerov2 (gazebo_tracked_vehicle.launch.py) only fires once,
via OnProcessExit chained off spawn_entity's own `ros_gz_sim create`
process -- structurally, that chain fires on ANY exit (success or failure),
so a hung create call was ruled out as the cause; but it's still a single
fire-once attempt with no verification that it actually worked and no
retry if the world genuinely wasn't ready yet (a real race: the boat's own
model may not be fully registered in gz-transport's pose stream the instant
its spawn process exits).

This script replaces that fire-once, unverified attempt with an active
wait-then-retry, run as its own independent process (not chained off
anything else's exit):
  1. Polls the world's real pose stream (gz.msgs10.Pose_V on
     /world/<world>/pose/info -- the same topic this project already reads
     elsewhere, e.g. motorized_tether_control.py) for BOAT_MODEL_NAME, up
     to BOAT_WAIT_TIMEOUT_S, before attempting anything -- removes the
     timing race entirely instead of hoping a fixed delay was long enough.
  2. If bluerov2 already exists (e.g. the original OnProcessExit spawn
     already succeeded, this script running as a pure safety net), does
     nothing and exits cleanly -- never double-spawns.
  3. Otherwise calls the real `/world/<world>/create` service directly via
     gz-transport's own Python bindings (gz.transport13), with real retries
     (SPAWN_RETRIES attempts, SPAWN_RETRY_DELAY_S apart) if the service
     call fails or times out, logging clearly on each attempt and on final
     failure -- not a single silent fire-and-forget.
"""
import sys
import time

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory

WORLD_NAME = 'cavex_world'
BOAT_MODEL_NAME = 'cavex_tracked_blueboat'
ROV_MODEL_NAME = 'bluerov2'
ROV_MODEL_SDF = ('/home/parvu/CaveX-Explorer-Pro/ros2_ws/src/'
                  'cavex_tracked_vehicle/models/bluerov2/model.sdf')
GZ_POSE_TOPIC = f'/world/{WORLD_NAME}/pose/info'
CREATE_SERVICE = f'/world/{WORLD_NAME}/create'

BOAT_WAIT_TIMEOUT_S = 60.0
BOAT_POLL_PERIOD_S = 0.5
EXISTENCE_CHECK_WINDOW_S = 3.0  # widened after a live race was found: 1.0s could elapse before
                                 # the fast OnProcessExit path's own just-created entity showed up
                                 # in the next pose/info publish, cycle -- Gazebo's create service
                                 # turned out to no-op harmlessly on the resulting name collision
                                 # rather than double-spawning, but the check should genuinely see
                                 # it in the normal case, not rely on that no-op behavior
SPAWN_RETRIES = 5
SPAWN_RETRY_DELAY_S = 2.0
# Matches gazebo_tracked_vehicle.launch.py's own current spawn_bluerov2 pose
# -- see that file's own comment for the real derivation of these numbers.
SPAWN_X, SPAWN_Y, SPAWN_Z = -35.1, 0.0, 6.4755


def _model_seen(node: Node, model_name: str, window_s: float) -> bool:
    """Subscribe to the world's real pose stream for window_s seconds and
    report whether model_name appeared in it at all."""
    seen = []

    def _cb(msg: Pose_V):
        for pose in msg.pose:
            if pose.name == model_name:
                seen.append(True)

    node.subscribe(Pose_V, GZ_POSE_TOPIC, _cb)
    deadline = time.monotonic() + window_s
    while time.monotonic() < deadline and not seen:
        time.sleep(0.1)
    node.unsubscribe(GZ_POSE_TOPIC)
    return bool(seen)


def spawn_rov(node: Node) -> bool:
    req = EntityFactory()
    req.sdf_filename = ROV_MODEL_SDF
    req.name = ROV_MODEL_NAME
    req.pose.position.x = SPAWN_X
    req.pose.position.y = SPAWN_Y
    req.pose.position.z = SPAWN_Z
    result, response = node.request(CREATE_SERVICE, req, EntityFactory, Boolean, 5000)
    return bool(result and response.data)


def main():
    node = Node()

    if _model_seen(node, ROV_MODEL_NAME, EXISTENCE_CHECK_WINDOW_S):
        print(f"{ROV_MODEL_NAME} already exists -- nothing to do "
              "(this script is a safety net, not the primary spawn path).")
        return 0

    print(f"Waiting up to {BOAT_WAIT_TIMEOUT_S}s for {BOAT_MODEL_NAME} to "
          "actually appear in the world's pose stream before attempting "
          "the ROV spawn (removes the real timing race a fixed delay "
          "would only guess at)...")
    if not _model_seen(node, BOAT_MODEL_NAME, BOAT_WAIT_TIMEOUT_S):
        print(f"ERROR: {BOAT_MODEL_NAME} never appeared within "
              f"{BOAT_WAIT_TIMEOUT_S}s -- giving up on spawning "
              f"{ROV_MODEL_NAME} (the boat itself never spawning is a "
              "separate, more serious problem this script can't fix).",
              file=sys.stderr)
        return 1

    for attempt in range(1, SPAWN_RETRIES + 1):
        print(f"Spawning {ROV_MODEL_NAME}, attempt {attempt}/{SPAWN_RETRIES}...")
        if spawn_rov(node):
            print(f"{ROV_MODEL_NAME} spawned successfully.")
            return 0
        if attempt < SPAWN_RETRIES:
            print(f"Spawn attempt {attempt} failed (service call returned "
                  f"false or timed out) -- retrying in {SPAWN_RETRY_DELAY_S}s.")
            time.sleep(SPAWN_RETRY_DELAY_S)

    print(f"ERROR: {ROV_MODEL_NAME} failed to spawn after {SPAWN_RETRIES} "
          "attempts.", file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
