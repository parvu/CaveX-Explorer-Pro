#!/usr/bin/env python3
"""
control_server.py

Real request 2026-08-26: functional control buttons in the browser viewer.
gz-launch's WebsocketServer protocol (the one index.html already uses for
the scene/pose stream) is read-only from the browser's side -- confirmed
against its own real source (WebsocketServer.cc's OnMessage only handles
auth/protos/topics/topics-types/worlds/scene/particle_emitters/sub/image/
unsub/throttle/asset; there is no "publish a message" operation). It can't
be used to send drive/track commands back into the sim.

This is a small, separate HTTP server (stdlib only, no new dependency) that
serves the same static viewer files AND exposes a couple of control
endpoints. Deliberately reuses the exact same gz-transport StringMsg
topics the desktop GUI's ActionButtons/ManualControl plugins already use
(/cavex/manual_cmd, /cavex/track_cmd) -- manual_gui_bridge.py already
consumes both and relays them into the real ROS2/cmd_vel chain, so this
server needs zero new backend logic on that end, just gz-transport
publishers matching what those GUI plugins already send.

Run from web_viewer/: python3 control_server.py [port]
(replaces the plain `python3 -m http.server 8080` used before -- same
static files, same port by default, now also serving /api/*.)
"""
import sys
import os
import http.server
import urllib.parse

from gz.transport13 import Node as GzNode
from gz.msgs10.stringmsg_pb2 import StringMsg

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080

gz_node = GzNode()
manual_pub = gz_node.advertise('/cavex/manual_cmd', StringMsg)
track_pub = gz_node.advertise('/cavex/track_cmd', StringMsg)

MANUAL_COMMANDS = {
    'forward', 'backward', 'left', 'right', 'stop',
    'speed_up', 'speed_down', 'manual_on', 'manual_off',
}
TRACK_COMMANDS = {'deployed', 'retracted'}


class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == '/api/manual':
            self._handle_command(parsed, manual_pub, MANUAL_COMMANDS)
        elif parsed.path == '/api/tracks':
            self._handle_command(parsed, track_pub, TRACK_COMMANDS)
        else:
            super().do_GET()

    def _handle_command(self, parsed, pub, allowed):
        cmd = urllib.parse.parse_qs(parsed.query).get('cmd', [''])[0]
        if cmd not in allowed:
            self.send_response(400)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(f"unknown command {cmd!r}".encode())
            return
        pub.publish(StringMsg(data=cmd))
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def log_message(self, fmt, *args):
        if '/api/' in (args[0] if args else ''):
            super().log_message(fmt, *args)


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    server = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print(f"control_server ready on 0.0.0.0:{PORT} "
          f"(static files + /api/manual, /api/tracks)")
    server.serve_forever()


if __name__ == '__main__':
    main()
