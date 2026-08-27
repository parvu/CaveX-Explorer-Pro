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
import posixpath
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
            self._fix_up_asset_path(parsed)
            super().do_GET()

    def _fix_up_asset_path(self, parsed):
        # Real bug found live 2026-08-27: gz3d.js guesses model texture
        # paths using the old Gazebo-classic layout convention
        # (materials/textures/<file>), but this project's vendored models
        # (cave_world, etc.) keep their real texture files directly
        # alongside the mesh under meshes/ instead -- confirmed live via a
        # 404 for .../cave_world/materials/textures/CaveWall.png while the
        # real file sits at .../cave_world/meshes/CaveWall.png. Rewrites
        # self.path to the real location when it exists there, rather than
        # patching gz3d.js's own path-guessing logic in the ~2MB vendored
        # file. posixpath.normpath also collapses the "assets//cave_world"
        # double-slash gz3d.js's own path-joining produces.
        path = posixpath.normpath(parsed.path)
        if '/materials/textures/' not in path:
            return
        if os.path.isfile(self.translate_path(path)):
            return  # real file already exists at the guessed path, nothing to fix
        fallback = path.replace('/materials/textures/', '/meshes/')
        if os.path.isfile(self.translate_path(fallback)):
            self.path = fallback + (('?' + parsed.query) if parsed.query else '')

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
        if '/api/' in self.path:
            super().log_message(fmt, *args)


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    server = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print(f"control_server ready on 0.0.0.0:{PORT} "
          f"(static files + /api/manual, /api/tracks)")
    server.serve_forever()


if __name__ == '__main__':
    main()
