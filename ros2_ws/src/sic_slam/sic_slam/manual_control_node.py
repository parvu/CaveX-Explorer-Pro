#!/usr/bin/env python3
"""Drives the 6 real thrusters from ManualControl.qml's button commands
(published to /sic_slam/manual_cmd as gz.msgs.StringMsg by
sic_slam_gui's ManualControl plugin -- see that package's README/comments).

Held-command semantics: a direction button sets the current command and
it stays in effect (continuously re-published at CONTROL_PERIOD_S) until
"stop" or a different direction is pressed -- there is no press/release
tracking on the QML side, just single click-to-set commands, so this is
the simplest semantics that matches a "click and it keeps moving" pad.

Auto-launched by sim_launch.py (real fix, 2026-08-25: previously a
standalone scripts/ file like ate_circle_demo.py, but manual control only
does anything after the operator both starts this AND toggles Manual in
the GUI -- unlike the ATE/measurement scripts, there's no reason not to
have it always running, and the earlier standalone setup meant clicking
the GUI's buttons silently did nothing because this was never started).
While "manual_on" is false, this node publishes NOTHING at all -- not
even zeros -- so an autonomous script (ate_circle_demo.py etc.) started
separately keeps sole, uncontested control of the same thruster topics.
Running both manual mode ON and an autonomous script at the same time is
not guarded against -- that's on the operator, not a case this node
resolves for you.

Same world-frame-force -> body-frame -> 4-horizontal/2-vertical-thruster
allocation as ate_circle_demo.py (KD_GEOM etc.), but manual commands are
plain body-frame unit forces (no PD position error, no yaw control --
manual driving is direct, not station-keeping).
"""
import math
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double
from gz.msgs10.stringmsg_pb2 import StringMsg

FORCE_N = 40.0  # per-axis body-frame force while a direction is held
CONTROL_PERIOD_S = 0.1
KD_GEOM = math.sqrt(2.0) / 2.0

# command -> (fx_body, fy_body, fz_body)
COMMANDS = {
    "forward": (FORCE_N, 0.0, 0.0),
    "backward": (-FORCE_N, 0.0, 0.0),
    "left": (0.0, FORCE_N, 0.0),
    "right": (0.0, -FORCE_N, 0.0),
    "up": (0.0, 0.0, FORCE_N),
    "down": (0.0, 0.0, -FORCE_N),
    "stop": (0.0, 0.0, 0.0),
}

_state = {"cmd": "stop", "manual_on": False}


def _cmd_cb(msg: StringMsg):
    data = msg.data
    if data == "manual_on":
        _state["manual_on"] = True
    elif data == "manual_off":
        _state["manual_on"] = False
        _state["cmd"] = "stop"
    elif data in COMMANDS:
        _state["cmd"] = data


def allocate_thrust(fx_body, fy_body, fz_body):
    t1 = -0.5 * KD_GEOM * (fx_body + fy_body)
    t2 = 0.5 * KD_GEOM * (fy_body - fx_body)
    t3 = 0.5 * KD_GEOM * (fx_body - fy_body)
    t4 = 0.5 * KD_GEOM * (fx_body + fy_body)
    t5 = -fz_body / 2.0
    t6 = -fz_body / 2.0
    return [t1, t2, t3, t4, t5, t6]


def main():
    node = GzNode()
    node.subscribe(StringMsg, "/sic_slam/manual_cmd", _cmd_cb)
    pubs = [
        node.advertise(f"/model/bluerov2/joint/thruster{i}_joint/cmd_thrust", Double)
        for i in range(1, 7)
    ]

    print("manual_control_node ready: listening on /sic_slam/manual_cmd, "
          "driving /model/bluerov2/joint/thrusterN_joint/cmd_thrust while Manual is on")

    try:
        while True:
            if _state["manual_on"]:
                fx_b, fy_b, fz_b = COMMANDS[_state["cmd"]]
                thrusts = allocate_thrust(fx_b, fy_b, fz_b)
                for pub, val in zip(pubs, thrusts):
                    pub.publish(Double(data=val))
            time.sleep(CONTROL_PERIOD_S)
    except KeyboardInterrupt:
        if _state["manual_on"]:
            for pub in pubs:
                pub.publish(Double(data=0.0))


if __name__ == "__main__":
    main()
