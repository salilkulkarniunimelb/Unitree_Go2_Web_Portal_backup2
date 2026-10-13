"""
lab_portal.py - 5G-Enabled Multi-Agent Mission Dashboard (QOD Lab).

A Hivemind Mission Dashboard-style UI for the QOD lab Go2 robots (Luna & Astro).

Runs inside the robot_hivemind ROS 2 (Humble) container and subscribes to the
lab's real topics (bridged from the Go2 over Zenoh / domain 70):

    MAP     /lidar_slam_2d_map            (nav_msgs/msg/OccupancyGrid)
    POSE    /luna/amcl_pose, /astro/amcl_pose  (geometry_msgs/msg/PoseWithCovarianceStamped)
    ODOM    /go2/restamped/robot_odom     (nav_msgs/msg/Odometry)
    CAMERA  /frontvideostream             (sensor_msgs/msg/Image)
    BATTERY /lf/lowstate                  (unitree_go/msg/LowState)

    Camera (default): the front camera is pulled from the QOD streaming
    platform's WebRTC SFU (the same channel + decode path the qod website uses),
    so the dashboard keeps showing the dog's camera even when the ROS
    /frontvideostream bridge is not publishing. Set LAB_CAMERA_SOURCE=ros to
    fall back to the legacy Go2FrontVideoData ROS-topic decoder.

Layout mirrors the Hivemind mission dashboard:
    - Workflow stepper (mapping -> scenario -> localisation -> operations)
    - Robot fleet panel (Luna / Astro cards with live status)
    - Live occupancy map (center) + live camera (right)
    - Exhibition mapping/localisation/patrol controls
    - Telemetry + status footer
    - "3D Map": a Foxglove-style 3D lidar page, served by the
      side-car web_backend/foxglove_viewer.py on port 8767 and embedded in an
      iframe, so orbit / pan / zoom work exactly like Foxglove's 3D panel.

Mission controls delegate to hardware_code's exhibitionctl.py over a restricted
SSH forced command. The exhibition controller owns mapping, verified completed-
map handover, per-robot localisation confirmation and the explicit patrol gate.

Working features stream live (map, camera, battery, pose). Features not yet
implemented (scenario/operations steps, ROS gateway, 5G link, RealSense/YOLO
streams) have been removed from the layout until they are wired up.

The page ALWAYS loads. Every section shows a "Waiting for ..." placeholder
until its topic starts publishing. Direct click-to-goal is disabled in
exhibition mode. Toggle "Set Initial Pose" and drag on the map to submit x/y/yaw
through the exhibition controller, then confirm localisation separately.

Usage:
    source /opt/ros/humble/setup.bash
    python3 lab_portal.py            # serves on 0.0.0.0:7860
"""

import math
import os
import threading
import time
import base64
import json
from collections import deque

import cv2
import numpy as np

import gradio as gr

# Dashboard-side client for hardware_code's exhibition workflow. Imported
# defensively so a deployment slip cannot turn a missing control module into a
# portal crash loop; telemetry remains available and controls report the error.
try:
    import exhibition_control
except Exception as _exc:  # noqa: BLE001 - must never take the dashboard down
    exhibition_control = None
    EXHIBITION_IMPORT_ERROR = repr(_exc)
else:
    EXHIBITION_IMPORT_ERROR = None

# ----------------------- Topic names (lab-real) -----------------------
# Topics actually publishing on the lab server (verified via ros2 topic list).
# Each robot publishes under its own namespace (/luna/, /astro/).
# - Map and camera are the live streams.
# - AMCL pose is per-robot (/luna/amcl_pose, /astro/amcl_pose) and is the
#   accurate localization source for drawing each robot on the map.
# - Odom is per-robot (/luna|astro/go2/restamped/robot_odom) and is used as a
#   fallback pose source when AMCL is not publishing on a robot.
# - No battery topic exists for either robot yet.
LUNA_TOPICS = {
    "map":     "/luna/lidar_slam_2d_map",
    "pose":    "/luna/amcl_pose",
    "odom":    "/luna/go2/restamped/robot_odom",
    "camera":  "/luna/frontvideostream",
    "source":  "luna",
    "battery": "/lf/lowstate",
    "plan":    "/luna/plan",
    "map_odom": "/luna/go2/split_compute/map_odom_correction",
}

ASTRO_TOPICS = {
    "map":     "/astro/lidar_slam_2d_map",
    "pose":    "/astro/amcl_pose",
    "odom":    "/astro/go2/restamped/robot_odom",
    "camera":  "/astro/frontvideostream",
    "source":  "astro",
    "battery": "/lf/lowstate",
    "plan":    "/astro/plan",
    "map_odom": "/astro/go2/split_compute/map_odom_correction",
}

ROBOTS = {
    "Luna": LUNA_TOPICS,
    "Astro": ASTRO_TOPICS,
}

# Luna's camera is H.264-encoded Go2FrontVideoData. We decode it with a
# persistent ffmpeg pipeline (started once per robot) fed via stdin and read
# Camera frames arrive as raw H.264 Go2FrontVideoData, fragmented across many
# small ROS messages. A persistent ffmpeg pipe cannot stay in sync on this
# stream, so we buffer bytes and decode the newest SPS-keyframe window from a
# file on a background thread. ffmpeg must be installed in the container.
CAM_W, CAM_H, CAM_BUFFER_MAX = 640, 360, 6 * 1024 * 1024
# Ignore the buffer until it holds at least this much data (enough for a
# keyframe sequence) -- avoids decoding a nearly-empty, undecodable buffer.
CAM_MIN_BYTES = 512 * 1024
# Background decode cadence (s). Governs how often a new batch is decoded.
# The Go2 source streams ~260 tiny H.264 fragments/s of only ~7 distinct display
# frames/s, so decoding faster than 0.4s only re-produces duplicate frames and
# costs CPU without raising the visible fps (which is source-limited).
CAM_DECODE_INTERVAL = 0.4
# Max frames to cache for smooth playback (one shot of motion per refill).
CAM_FRAMES_MAX = 20
# Cap the decoded segment to this many bytes -- we only need the most recent
# keyframe onward, not the whole (up to several-MB) rolling buffer.
# Sized generously so a full GOP (which may include one keyframe + many P-frames)
# fits in the window even as the buffer scrolls, keeping the camera continuous.
CAM_DECODE_WINDOW = 4 * 1024 * 1024

# ----------------------- Camera source (QOD WebRTC vs ROS) ------------
# "webrtc" (default): consume the dog's front camera from the QOD platform's
# WebRTC SFU on 127.0.0.1:8000 - the same channel and decode path the qod
# streaming platform website uses. This keeps the camera live even when the ROS
# /frontvideostream bridge is not publishing (identical frames to the website).
# "ros": legacy Go2FrontVideoData ROS-topic subscription + file decode above.
CAMERA_SOURCE = os.getenv("LAB_CAMERA_SOURCE", "webrtc").strip().lower()
QOD_SIGNALING_URL = os.getenv(
    "QOD_SIGNALING_URL", "ws://127.0.0.1:8000/ws/signaling"
)

# -------------------- QOD detection overlay -------------------------
# The qod cv-worker runs YOLO on each robot's WebRTC feed and publishes the
# boxes to Redis channel "detections:<source>"; the qod website draws them on a
# canvas. Subscribing to the same channel lets us draw the identical overlay on
# the dashboard camera. Set LAB_DETECTIONS=0 to disable.
DETECTIONS_ENABLED = os.getenv("LAB_DETECTIONS", "1").strip().lower() != "0"
REDIS_URL = os.getenv("QOD_DETECTIONS_REDIS_URL", "redis://127.0.0.1:6379")
# Only draw boxes the cv-worker already published (it filters at 0.25 -> 0.5);
# this guards the overlay against noise-tolerant thresholds.
DETECTIONS_MIN_CONF = float(os.getenv("LAB_DETECTIONS_MIN_CONF", "0.25"))
# Same class->style mapping as frontend/src/Services/detection-class-style.ts.
# NOTE: tuples are RGB, matching the (BGR-in-cv2-displayed-as-RGB) convention
# used by ROBOT_COLORS above (channel 0 = red on screen).
DETECTION_CLASS_STYLES = {
    "jerry":  dict(alias="Jerry can",      color=(255, 59, 48)),   # #ff3b30
    "spill":  dict(alias="Chemical spill", color=(34, 197, 94)),   # #22c55e
    "old_spill": dict(alias="Toxic Sludge", color=(34, 197, 94)),  # #22c55e
    "wire":   dict(alias="Exposed wire",   color=(168, 85, 247)),  # #a855f7
    "rubble": dict(alias="Debris",         color=(59, 130, 246)),  # #3b82f6
}
DEFAULT_DETECTION_STYLE = dict(alias="Object", color=(53, 255, 101))  # #35ff65

FT_FRAME = "map"

# ROS heading sources can stall (AMCL silent after start, or a one-shot /plan
# that is only re-published per goal). AMCL pose/yaw is trusted only while
# fresh; after this many seconds without a sample we fall back to the live
# odometry heading re-anchored into the map frame.
AMCL_FRESH_SECS = 3.0

# SLAM's map<-odom correction is published continuously while exploring, so it
# can be trusted for about as long as AMCL is trusted. If it goes quiet we stop
# using it and fall back to the older plan-anchored odom placement.
ODOM_FRESH_SECS = 3.0

# Per-robot visual identity. NOTE: cv2 draws BGR but Gradio displays RGB, so
# these tuples equal the browser colours (channel 0 = red on screen).
# Red = Luna, blue = Astro.
ROBOT_COLORS = {
    "Luna":  dict(fill=(255, 0, 0),  outline=(180, 0, 0),
                  path_dark=(160, 0, 0),  path_bright=(255, 0, 0)),
    "Astro": dict(fill=(0, 200, 255), outline=(0, 140, 200),
                  path_dark=(0, 0, 160), path_bright=(0, 0, 255)),
}


def _norm_angle(a):
    """Wrap an angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def _draw_plan(canvas, pts, dark, bright):
    """Draw the robot's planned route so it stands out against both the light
    explored map and the dark walls: light casing -> black under-stroke ->
    darker coloured outline -> bright coloured core (Luna red, Astro blue)."""
    cv2.polylines(canvas, [pts], False, (250, 250, 250), 14, cv2.LINE_AA)
    cv2.polylines(canvas, [pts], False, (0, 0, 0), 10, cv2.LINE_AA)
    cv2.polylines(canvas, [pts], False, dark, 7, cv2.LINE_AA)
    cv2.polylines(canvas, [pts], False, bright, 5, cv2.LINE_AA)


SERV_NAME = "0.0.0.0"
SERV_PORT = 7860

# ----------------------- Foxglove-style 3D view -----------------------
# The 3D view is a separate process (web_backend/foxglove_viewer.py) serving a
# Three.js scene plus a WebSocket point-cloud feed. The portal only embeds it,
# so nothing here can break the map/camera pages if the viewer has a bad day.
FOXGLOVE_PORT = 8767
FOXGLOVE_URL = f"http://localhost:{FOXGLOVE_PORT}"
FOXGLOVE_VIEWER_MODULE = "web_backend.foxglove_viewer"


def _foxglove_viewer_alive():
    """Is the viewer serving? Probed over HTTP, not by pid: the viewer is
    started by a supervisor thread that can be restarted under us, and a pid
    check would report a stale answer."""
    import urllib.error
    import urllib.request

    try:
        with urllib.request.urlopen(f"{FOXGLOVE_URL}/health", timeout=2) as r:
            return 200 <= r.status < 300, r.read()
    except Exception:
        return False, None


def _foxglove_status_html(note=None):
    """One-line status strip under the 3D viewer: whether the viewer process is
    up, and if so whether the selected robot's cloud is actually arriving.
    Deliberately a cheap /health GET on a 2 s timer."""
    alive, body = _foxglove_viewer_alive()
    if not alive:
        return (
            f'<div style="padding:8px 10px;border-radius:6px;'
            f'background:rgba(180,50,50,.18);border:1px solid rgba(255,120,120,.35);'
            f'font-size:13px;color:#ffb4ab;">'
            f'3D viewer not responding on port {FOXGLOVE_PORT}. '
            f'Starting it… (the rest of the dashboard is unaffected)</div>'
        )

    bits = []
    try:
        info = json.loads(body.decode())
    except Exception:
        return (
            '<div style="padding:8px 10px;font-size:13px;color:#c8c8d0;">'
            '3D Viewer online.</div>'
        )

    bits.append(f"Viewer online · port {info.get('port', FOXGLOVE_PORT)}")
    bits.append(f"{info.get('max_points_per_frame')} pts/frame @ "
                f"{info.get('target_fps')} fps")

    # The graph-optimised map is one global topic (graph_based_slam publishes it
    # un-namespaced), so it is reported once here rather than under each robot.
    mm = info.get("modified_map") or {}
    if mm.get("modified_map_msgs"):
        age = mm.get("last_modified_map_age")
        age_s = f", {age:.0f}s ago" if age is not None else ""
        bits.append(f"modified map {mm['modified_map_points']:,} pts"
                    f" on {mm.get('modified_map_topic')}{age_s}")
    elif mm.get("modified_map_topic"):
        # Names the topic so "nothing on it yet" is distinguishable from
        # "not wired up" -- identical from inside the 3D panel.
        bits.append(f"modified map: none on {mm['modified_map_topic']}")

    robots = info.get("robots") or {}
    for ns, st in robots.items():
        label = ns.capitalize()
        age = st.get("last_cloud_age")
        if st.get("cloud_msgs", 0) > 0:
            fresh = f"{age:.1f}s ago" if age is not None else "just now"
            cloud = f"{st['buffered']:,} pts ({fresh})"
        else:
            cloud = "no cloud yet"
        extra = []
        if st.get("pose_source"):
            extra.append(f"pose:{st['pose_source']}")
        if st.get("has_map"):
            extra.append("map ✓")
        if st.get("plan_points"):
            extra.append(f"path:{st['plan_points']}")
        # Dropped-without-pose points are deliberately not surfaced here. It
        # dominated the strip -- a robot that never gets a pose accrues a
        # seven-figure total that grows forever and tells the operator nothing
        # they can act on. It stays in /health (dropped_no_pose and
        # dropped_no_pose_rate) for when the counter is the actual question.
        if st.get("last_error"):
            extra.append(f"error: {st['last_error'][:80]}")
        bits.append(f"{label}: {cloud}"
                    + (f" · {' · '.join(extra)}" if extra else ""))

    tail = f" <span style='opacity:.65'>{note}</span>" if note else ""
    return (
        '<div style="padding:8px 10px;border-radius:6px;'
        'background:rgba(70,110,180,.16);border:1px solid rgba(140,180,240,.3);'
        'font-size:12px;color:#cfe0ff;">'
        + " · ".join(bits) + tail + "</div>"
    )


def _goto_tab(tab_id):
    """Click handler that selects another tab.

    Gradio switches tabs by making gr.Tabs itself an output and returning
    gr.Tabs(selected=...). A button click has no inputs to read the target
    from, so the id is bound here and the returned handler takes none.

    Deliberately a pair of buttons rather than one 2D/3D dropdown: a control
    mirroring the tab bar would be a second source of truth for which tab is
    active, and it could not represent the other two tabs at all.
    """
    return lambda: gr.Tabs(selected=tab_id)


def _foxglove_probe(robot):
    """gr.Timer callback for the 3D view's status strip."""
    return _foxglove_status_html()


def _start_foxglove_viewer():
    """Supervise web_backend/foxglove_viewer.py in its own process.

    Runs on a daemon thread started from main(). It only ever touches the
    viewer: it can never take the portal down, and it restarts the viewer after
    a crash or a port clash, which is the whole reason the tab is usable without
    anyone having to log into the container.
    """
    import subprocess
    import sys

    root = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(root, "foxglove_viewer.log")

    def loop():
        backoff = 5.0
        while True:
            if _foxglove_viewer_alive()[0]:
                time.sleep(10.0)
                continue
            try:
                with open(log_path, "ab", buffering=0) as log:
                    log.write(f"\n=== viewer start "
                              f"{time.strftime('%Y-%m-%d %H:%M:%S')} ===\n"
                              .encode())
                    proc = subprocess.Popen(
                        [sys.executable, "-m", FOXGLOVE_VIEWER_MODULE],
                        cwd=root,
                        stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                    )
                    # Wait for the child, but keep the check alive: if the
                    # portal is restarted this thread dies with it anyway.
                    proc.wait()
            except Exception as exc:  # noqa: BLE001
                print(f"[foxglove] supervisor: {exc!r}", flush=True)

            if _foxglove_viewer_alive()[0]:
                backoff = 5.0
            else:
                backoff = min(backoff * 2, 60.0)
            time.sleep(backoff)

    threading.Thread(target=loop, name="foxglove-supervisor", daemon=True).start()

# ----------------------- import rclpy + msgs (defensive) -------------
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - cv_bridge usually present in Humble
    CvBridge = None

try:
    from unitree_go.msg import LowState
except Exception:
    LowState = None

try:
    from unitree_go.msg import Go2FrontVideoData
except Exception:
    Go2FrontVideoData = None


def _placeholder(w, h, text, color=(56, 189, 248)):
    """Dark placeholder image with centred text."""
    ph = np.ones((h, w, 3), dtype=np.uint8) * 12
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(text, font, 0.6, 2)
    cv2.putText(ph, text, ((w - tw) // 2, (h + th) // 2), font, 0.6, color, 2)
    return ph


def _blank(w, h):
    """Solid dark blank frame with no text (used for the map area when no data)."""
    return np.ones((h, w, 3), dtype=np.uint8) * 12


_LOGO_URI = None


def logo_data_uri(path="assets/logo.png"):
    """Return the logo as a base64 data URI so it renders regardless of how
    Gradio serves static files (works over SSH tunnels / containers).
    Returns "" if the file is missing so the page never crashes."""
    global _LOGO_URI
    if _LOGO_URI is None:
        try:
            with open(path, "rb") as f:
                b64 = base64.b64encode(f.read()).decode()
            _LOGO_URI = f"data:image/png;base64,{b64}"
        except FileNotFoundError:
            _LOGO_URI = ""
    return _LOGO_URI


DASHBOARD_CSS = """
    html, body, .gradio-container, .gradio-container * {
        font-family: Helvetica, 'Helvetica Neue', Arial, sans-serif !important;
    }
    /* ---------- Hivemind Mission Dashboard style ----------
       Colors are driven by Gradio theme CSS variables so switching the
       theme in Settings applies everywhere (no hardcoded dark colors). */
    .gradio-container{background:var(--background-fill-primary) !important;
        color:var(--body-text-color) !important;max-width:100% !important;}
    .portal-header{display:flex;align-items:center;gap:14px;
        padding:14px 18px;border-radius:12px;
        background:linear-gradient(90deg,var(--block-background-fill),var(--block-background-fill));
        border:1px solid var(--block-border-color);
        color:var(--block-title-text-color);margin-bottom:10px;}
    .portal-header img.logo{height:44px;width:auto;border-radius:8px;
        box-shadow:0 2px 8px rgba(0,0,0,.35);}
    .portal-header .title{font-size:22px;font-weight:700;letter-spacing:.3px;
        color:var(--color-accent);}
    .portal-header .subtitle{font-size:13px;opacity:.85;margin-top:2px;
        color:var(--body-text-color-subdued);}
    .robot-selector{margin-top:10px;}
    #initpose_bridge{pointer-events:none;opacity:0;height:0;overflow:hidden;}
    #initpose_bridge textarea{opacity:0;height:0;min-height:0!important;}
    #initpose_toggle_btn{font-size:14px !important;font-weight:600 !important;
        font-family:"Helvetica", "Helvetica Neue", Arial, sans-serif !important;
        letter-spacing:.2px !important;
        padding:4px 12px !important;min-height:0 !important;height:auto !important;
        width:auto !important;border-radius:8px !important;}

    # Blocks / cards
    .gr-block,.gr-box,.gr-form{background:transparent !important;}
    .gr-group,.gr-gallery{background:var(--block-background-fill) !important;
        border:1px solid var(--block-border-color) !important;
        border-radius:12px !important;padding:12px !important;}

    /* Foxglove status strip: sits under the iframe, so it needs its own gap
       above rather than the default block spacing. */
    .foxglove-status{margin-top:10px !important;}
    /* 2D <-> 3D page hops. Secondary weight so they read as navigation, not
       as a control that changes anything on the current page. */
    .switch-view-btn{margin-top:10px !important;font-weight:600 !important;}

    /* Tab navigation — style gradio tabs to look like nav buttons */
    .tab-nav{display:flex;gap:6px !important;margin-bottom:10px !important;padding:6px !important;
        background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;}
    .tab-nav button{border-radius:10px !important;font-weight:600 !important;
        padding:10px 24px !important;font-size:14px !important;
        transition:all 0.2s ease !important;}
    .tab-nav button.selected{background:var(--color-accent) !important;
        color:var(--button-primary-text-color, #fff) !important;
        border-color:var(--color-accent) !important;}

    /* Robot fleet */
    .fleet-panel{display:flex;flex-direction:column;gap:10px;}
    .robot-card{background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;padding:12px;margin-bottom:10px;}
    .robot-card.selected{border-color:var(--color-accent);box-shadow:0 0 0 1px var(--color-accent);}
    .robot-name{font-size:15px;font-weight:600;color:var(--block-title-text-color);
        display:flex;align-items:center;gap:8px;margin-bottom:6px;}
    .robot-status-dot{width:8px;height:8px;border-radius:50%;display:inline-block;}
    .robot-status-dot.connected{background:#22c55e;box-shadow:0 0 6px #22c55e;}
    .robot-status-dot.waiting{background:#f59e0b;box-shadow:0 0 6px #f59e0b;}
    .telem-text{font-size:12px;color:var(--body-text-color-subdued);margin-top:4px;}

    /* Live 3D mapping panel */
    .map-panel{background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;padding:10px;}
    .map-panel-title{font-size:15px;font-weight:600;color:var(--block-title-text-color);
        display:flex;align-items:center;gap:8px;margin-bottom:6px;}

    /* Camera panel */
    .camera-panel{background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;padding:10px;}

    /* Status footer */
    .status-footer{display:flex;gap:14px;flex-wrap:wrap;
        background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;padding:10px 14px;margin-top:10px;font-size:12px;
        color:var(--body-text-color-subdued);}
    .status-item{display:flex;align-items:center;gap:6px;}
    .status-item .lv{width:8px;height:8px;border-radius:50%;display:inline-block;}
    .status-item .lv.green{background:#22c55e;}
    .status-item .lv.yellow{background:#f59e0b;}
    .status-item .lv.gray{background:#475569;}
    .status-item .lv.blue{background:#3b82f6;}

    .grid-2col{display:grid;grid-template-columns:1fr 1fr;gap:10px;}
    .grid-3col{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;}

    /* Future placeholder buttons */
    .future-placeholder{display:flex;align-items:center;justify-content:center;
        background:var(--input-background-fill);border:1px dashed var(--input-border-color);
        border-radius:8px;color:var(--body-text-color-subdued);font-size:12px;
        padding:8px 12px;min-height:36px;}

    .section-label{font-size:13px;color:var(--color-accent);font-weight:600;
        text-transform:uppercase;letter-spacing:.5px;margin:12px 0 8px;}
    .section-label:first-child{margin-top:0;}

    /* Mission buttons (future) */
    .mission-buttons{display:flex;gap:8px;flex-wrap:wrap;}
    .mission-buttons .gr-button{background:var(--input-background-fill);color:var(--color-accent);
        border:1px solid var(--input-border-color);border-radius:8px !important;}
    .mission-buttons .gr-button:hover{background:var(--border-color-primary);color:var(--body-text-color);}
"""

# Foxglove-style click-and-drag to set the initial pose. The drag endpoints are
# captured (in image-pixel coords) and forwarded to a Python handler through a
# small hidden Textbox bridge (initpose_bridge) by dispatching a DOM 'input'
# event, which Gradio's `.input` listener picks up. When the "Set Initial Pose"
# toggle is off, the backend ignores the payload.
INIT_POSE_JS = """<script>
(function () {
    var ACTIVE = true;
    var dragging = false;
    var sX = 0, sY = 0, sPx = 0, sPy = 0;
    var svg = null, svgLine = null, svgHead = null, svgDot = null;

    // Gradio mounts its UI inside a shadow root under <gradio-app>. Resolve an
    // element against that shadow root (falling back to the light DOM).
    function resolveEl(sel) {
        var root = document.querySelector('gradio-app');
        if (root && root.shadowRoot) {
            var el = root.shadowRoot.querySelector(sel);
            if (el) return el;
        }
        return document.querySelector(sel);
    }

    function mapImg() {
        var img = resolveEl('#map_image');
        if (!img) return null;
        return img.tagName === 'IMG' ? img : img.querySelector('img');
    }

    function inImg(e, img) {
        var r = img.getBoundingClientRect();
        return e.clientX >= r.left && e.clientX <= r.right &&
               e.clientY >= r.top  && e.clientY <= r.bottom;
    }

    function toImgPts(e, img) {
        var r = img.getBoundingClientRect();
        if (!r.width || !r.height) return [0, 0];
        var x = Math.round((e.clientX - r.left) / r.width  * (img.naturalWidth  || 1));
        var y = Math.round((e.clientY - r.top)  / r.height * (img.naturalHeight || 1));
        return [x, y];
    }

    // A full-viewport SVG overlay that draws a Foxglove-style pose arrow while
    // dragging: a dot at the press point + a line with an arrowhead at the tip.
    function makeArrow() {
        if (svg) return svg;
        var NS = 'http://www.w3.org/2000/svg';
        svg = document.createElementNS(NS, 'svg');
        svg.setAttribute('style',
            'position:fixed;left:0;top:0;width:100vw;height:100vh;' +
            'z-index:99999;pointer-events:none;overflow:visible;' +
            'touch-action:none;');
        svgDot = document.createElementNS(NS, 'circle');
        svgDot.setAttribute('r', '7');
        svgDot.setAttribute('fill', 'rgba(34,197,94,.95)');
        svgDot.setAttribute('stroke', 'rgba(0,80,40,.6)');
        svgDot.setAttribute('stroke-width', '2');
        svgLine = document.createElementNS(NS, 'line');
        svgLine.setAttribute('stroke', 'rgba(34,197,94,.95)');
        svgLine.setAttribute('stroke-width', '3.5');
        svgLine.setAttribute('stroke-linecap', 'round');
        svgHead = document.createElementNS(NS, 'polygon');
        svgHead.setAttribute('fill', 'rgba(34,197,94,.95)');
        svgHead.setAttribute('stroke', 'rgba(0,80,40,.6)');
        svgHead.setAttribute('stroke-width', '1');
        svg.appendChild(svgDot);
        svg.appendChild(svgLine);
        svg.appendChild(svgHead);
        document.body.appendChild(svg);
        return svg;
    }

    function drawArrow(e) {
        var dx = e.clientX - sX, dy = e.clientY - sY;
        var len = Math.hypot(dx, dy);
        var ang = Math.atan2(dy, dx);
        svgDot.setAttribute('cx', sX);
        svgDot.setAttribute('cy', sY);
        svgLine.setAttribute('x1', sX);
        svgLine.setAttribute('y1', sY);
        // If the drag is essentially a click (too short), just show the dot.
        if (len < 4) {
            svgLine.setAttribute('x2', sX);
            svgLine.setAttribute('y2', sY);
            svgHead.setAttribute('points', sX + ',' + sY + ' ' + sX + ',' + sY + ' ' + sX + ',' + sY);
            return;
        }
        var tipX = e.clientX, tipY = e.clientY;
        svgLine.setAttribute('x2', tipX);
        svgLine.setAttribute('y2', tipY);
        // arrowhead: a triangle whose base is behind the tip, perp to the line
        var hs = 15, hw = 7.5;
        var bx = tipX - hs * Math.cos(ang), by = tipY - hs * Math.sin(ang);
        var cp = Math.PI / 2;
        var q1x = bx + hw * Math.cos(ang + cp), q1y = by + hw * Math.sin(ang + cp);
        var q2x = bx + hw * Math.cos(ang - cp), q2y = by + hw * Math.sin(ang - cp);
        svgHead.setAttribute('points',
            tipX + ',' + tipY + ' ' + q1x + ',' + q1y + ' ' + q2x + ',' + q2y);
    }

    document.addEventListener('pointerdown', function (e) {
        var img = mapImg();
        if (!img || !ACTIVE || !inImg(e, img)) return;
        // Only the primary (left) mouse button — and only when the toggle is ON.
        if (e.pointerType === 'mouse' && e.button !== 0) return;
        var toggle = resolveEl('#initpose_toggle input');
        if (!toggle || !toggle.checked) return;
        // Stop the browser/Gradio default (map pan / text-select) that would
        // otherwise hijack a left-button drag, so left-click drag works.
        e.preventDefault();
        var p = toImgPts(e, img);
        sPx = p[0]; sPy = p[1];
        sX = e.clientX; sY = e.clientY;
        dragging = true;
        makeArrow();
        // Capture pointer events on the overlay so the drag does NOT pan the
        // underlying map (Gradio's image pan must not fire while drawing).
        svg.style.pointerEvents = 'auto';
        svg.style.display = '';
        drawArrow(e);
    });

    document.addEventListener('pointermove', function (e) {
        if (!dragging || !svg) return;
        e.preventDefault();
        drawArrow(e);
    });

    document.addEventListener('pointerup', function (e) {
        if (!dragging) return;
        dragging = false;
        if (svg) {
            svg.style.display = 'none';
            // Release the overlay so normal map panning/clicking works again.
            svg.style.pointerEvents = 'none';
        }
        var img = mapImg();
        if (!img || !inImg(e, img)) return;
        var p2 = toImgPts(e, img);
        var payload = sPx + ',' + sPy + ',' + p2[0] + ',' + p2[1];
        // Only publish when the "Set Initial Pose" toggle is checked.
        var toggle = resolveEl('#initpose_toggle input');
        var mode = toggle ? !!toggle.checked : false;
        try {
            fetch('/gradio_api/call/drag_initial', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ data: [payload, mode] })
            }).catch(function () {});
        } catch (err) {}
    });
})();
</script>"""


class RobotState:
    """Per-robot data + callbacks + rendering.

    One instance per robot (Luna, Astro). Each owns its own topic names and
    live data, so the dashboard can switch between robots without ever
    tearing down or recreating subscriptions.
    """

    def __init__(self, node, name, topics):
        self.node = node
        self.name = name
        self.topics = topics
        self._bridge = CvBridge() if CvBridge else None

        # --- map ---
        self.map_info = None
        self.map = None
        self.cached_map_img = None
        self.cached_scale = None
        self.cached_meta = None
        self.map_stamp = 0.0
        # --- pose / odom ---
        self.robot_pose = None
        self.pose_from_amcl = False   # True once AMCL (accurate) pose received
        self.yaw = 0.0
        self.odom_path = deque(maxlen=2000)
        self.pose_latest = "Waiting for pose..."
        # --- Nav2 planned path (published by the robot's global planner on
        #     /luna|astro/plan when a navigation goal is being executed) ---
        self.planner_path = None          # list of (wx, wy) in the "map" frame
        self.planner_path_stamp = 0.0
        # --- map-frame body anchoring ------------------------------------
        # The lab's AMCL (/luna|astro/amcl_pose) is NOT publishing, so the
        # odom fallback is used for the robot body. Odom lives in the odom
        # frame while the map + Nav2 plan are in the "map" frame, so drawing
        # the odom body with the map transform misplaces it (body off the
        # route). The Nav2 plan starts at the robot's current map-frame pose,
        # so we derive an odom->map offset from the plan's first point at each
        # replan and re-anchor the odom body into the map frame.
        self.last_odom = None             # latest (x, y) in the odom frame
        self.odom_to_map = None           # (dx, dy) odom -> map offset
        self.odom_yaw = None              # latest odom-frame heading (always kept)
        self.odom_to_map_yaw = None       # learned odom -> map rotation offset
        self.plan_start_yaw = None        # heading at the plan start (map frame)
        self._last_amcl_wall = 0.0        # last AMCL sample (wall clock)
        # --- SLAM map <- odom (exploring only) --------------------------------
        # While exploring, AMCL is not publishing AND Nav2 is not planning, so
        # there is no /plan to anchor the odom body against. The odom frame is
        # then drawn as if it were 'map', which is why the body and heading sat
        # off the route while the 3D view tracked it correctly. The lidar SLAM
        # stack publishes the real map<-odom transform on
        # /<ns>/go2/split_compute/map_odom_correction; composing it with odom
        # reproduces exactly what the 3D viewer already does.
        self.map_from_odom = None        # (x, y, yaw) map<-odod translation/rotation
        self.map_from_odom_stamp = 0.0
        # --- initial pose (set via map drag) ---
        self.initial_pose = None          # (wx, wy, yaw) pending/confirmed initial pose
        self.last_initial = None
        # --- camera ---
        self.color_frame = None
        self.cam_stamp = 0.0
        self.cam_frame_ts = 0.0
        self._cam_buf = bytearray()
        self._cam_lock = threading.Lock()
        self._cam_frames = deque(maxlen=CAM_FRAMES_MAX)  # smooth-playback buffer
        self._last_decode = 0.0
        self._stop_cam = False
        self._qod_detections = None  # QodDetectionsSubscriber (webrtc mode only)
        # Camera decode strategy: the Go2 H.264 stream is fragmented across many
        # small ROS messages, so a persistent ffmpeg pipe can never stay in sync
        # (constant "no frame!" / "non-existing PPS"). Instead we buffer bytes,
        # trim the tail to the latest SPS keyframe sequence, and decode that
        # window from a file on a background thread. Reliable, keeps the icon
        # green, and delivers a live multi-fps camera.
        if CAMERA_SOURCE == "webrtc":
            # Pull the camera from the QOD SFU instead of ROS (same feed the qod
            # website shows). Decoded BGR frames arrive via _on_qod_frame and are
            # queued into the same _cam_frames deck draw_camera() plays back.
            self._qod_consumer = None
            self._qod_detections = None
            qod_source = self.topics.get("source")
            if qod_source:
                try:
                    from web_backend.qod_consumer import QodCameraConsumer
                    self._qod_consumer = QodCameraConsumer(
                        source_id=qod_source,
                        name=self.name,
                        signaling_url=QOD_SIGNALING_URL,
                        on_frame=self._on_qod_frame,
                    )
                    self._qod_consumer.start()
                    self.node.get_logger().info(
                        f"[{self.name}] camera via QOD WebRTC (source={qod_source}, "
                        f"{QOD_SIGNALING_URL})"
                    )
                except Exception as e:
                    self.node.get_logger().error(
                        f"[{self.name}] failed to start QOD camera consumer: {e}"
                    )
                if DETECTIONS_ENABLED:
                    try:
                        from web_backend.qod_detections import (
                            QodDetectionsSubscriber,
                        )
                        self._qod_detections = QodDetectionsSubscriber(
                            source_id=qod_source,
                            name=self.name,
                            redis_url=REDIS_URL,
                        )
                        self._qod_detections.start()
                    except Exception as e:
                        self.node.get_logger().error(
                            f"[{self.name}] failed to start QOD detections "
                            f"subscriber: {e}"
                        )
        else:
            threading.Thread(target=self._camera_decode_loop, daemon=True).start()
        # --- battery / imu ---
        self.battery = None
        self.voltage = None
        self.current = None
        self.power = None
        self.roll = None
        self.pitch = None
        self.imu_yaw = None
        self.motor_status = None
        self.bat_stamp = 0.0


        # Camera playback FPS diagnostic (rough): count frames popped vs wall time.
        self._cam_played = 0
        self._cam_played_ts = time.time()
        self.cam_fps = 0.0

        # Cached H.264 parameter sets (SPS + PPS). The Go2 stream only emits these
        # once at startup, then continuous P-frames. The rolling buffer eventually
        # scrolls past the SPS, leaving only reference P-frames that cannot decode
        # on their own -> the camera "works a few moments then goes stale". We cache
        # the SPS/PPS and prepend them to every decode window so the stream stays
        # decodable forever.
        self._sps = b""
        self._pps = b""

    # ---------------------- callbacks ----------------------
    def map_cb(self, msg):
        self.map_info = msg.info
        mp = np.array(msg.data, dtype=np.float32).reshape(msg.info.height, msg.info.width)
        self.map = np.flipud(mp)
        h, w = self.map.shape
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        res = msg.info.resolution

        disp = np.ones((h, w), dtype=np.float32)
        disp[self.map == -1] = 0.5
        disp[self.map == 0] = 1.0
        disp[self.map > 0] = 0.0
        gray = (disp * 255).astype(np.uint8)
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        canvas = 800
        scale = canvas / max(h, w)
        img = cv2.resize(img, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_NEAREST)
        out = np.ones((canvas, canvas, 3), dtype=np.uint8) * 255
        ox = (canvas - img.shape[1]) // 2
        oy = (canvas - img.shape[0]) // 2
        out[oy:oy + img.shape[0], ox:ox + img.shape[1]] = img

        self.cached_map_img = out
        self.cached_scale = scale
        self.cached_meta = (origin_x, origin_y, h, ox, oy, res)
        self.map_stamp = self.node.get_clock().now().nanoseconds / 1e9

    def pose_cb(self, msg):
        # The per-robot AMCL pose topics (/luna/amcl_pose, /astro/amcl_pose)
        # are geometry_msgs/msg/PoseWithCovarianceStamped, where the pose lives
        # at msg.pose.pose. The older PoseStamped path keeps msg.pose at the
        # top level. Normalize so both feed the same drawing/telemetry code.
        pose = getattr(msg, "pose", None)
        if pose is None:
            return
        p = getattr(pose, "pose", pose)  # PoseWithCovarianceStamped -> .pose.pose
        self.robot_pose = p
        self.pose_from_amcl = True       # AMCL is the accurate per-robot source
        self._last_amcl_wall = time.time()
        q = p.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        # Learn the odom -> map rotation from AMCL (map-frame heading minus the
        # near-simultaneous odom-frame heading) too, so a live re-anchored
        # heading stays available after AMCL goes quiet / between plans.
        if self.odom_yaw is not None:
            self.odom_to_map_yaw = _norm_angle(self.yaw - self.odom_yaw)
        self.pose_latest = (
            f"x: {p.position.x:.3f}   "
            f"y: {p.position.y:.3f}   "
            f"yaw: {self.yaw:.3f} rad"
        )

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.last_odom = (p.position.x, p.position.y)
        self.odom_path.append(
            (p.position.x, p.position.y)
        )
        q = p.orientation
        try:
            # Always keep the latest odom-frame heading so we can re-anchor it
            # into the map frame with the learned rotation offset (this is what
            # keeps the heading arrow live while travelling).
            self.odom_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
        except Exception:
            pass
        # The per-robot odom topic is a fallback pose source. Once the accurate
        # AMCL pose has been received, keep using AMCL (odom would otherwise
        # drift / overwrite the localized position and misplace the robot dot).
        if self.pose_from_amcl:
            return
        try:
            self.robot_pose = p
            self.yaw = self.odom_yaw
            self.pose_latest = (
                f"x: {p.position.x:.3f}   "
                f"y: {p.position.y:.3f}   "
                f"yaw: {self.yaw:.3f} rad"
            )
        except Exception:
            pass

    def plan_cb(self, msg):
        """Store the latest Nav2 global plan (/luna|astro/plan, nav_msgs/msg/Path).
        This is the path the robot is about to take; it is re-published each time
        a new navigation goal is planned, so we just remember the newest one."""
        try:
            self.planner_path = [
                (p.pose.position.x, p.pose.position.y) for p in msg.poses
            ]
            self.planner_path_stamp = self.node.get_clock().now().nanoseconds / 1e9
            # Re-anchor the odom body into the map frame: the first plan pose is
            # the robot's current map-frame position, so the difference against
            # the (nearly simultaneous) odom reading is the odom->map offset.
            if msg.poses:
                fp = msg.poses[0].pose
                fx, fy = fp.position.x, fp.position.y
                if self.last_odom is not None:
                    self.odom_to_map = (fx - self.last_odom[0],
                                        fy - self.last_odom[1])
                # Prefer the plan's stated heading (map frame) for the body arrow.
                try:
                    q = fp.orientation
                    self.plan_start_yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                    )
                except Exception:
                    pass
                # Learn the odom -> map rotation: the plan's first pose heading
                # is the robot's map-frame heading at plan time, while odom_yaw
                # is the same physical heading in the odom frame (odom and map
                # axes differ by a fixed rotation). This offset lets us keep a
                # live map-frame heading between plans / while AMCL is silent.
                if (self.plan_start_yaw is not None
                        and self.odom_yaw is not None):
                    self.odom_to_map_yaw = _norm_angle(
                        self.plan_start_yaw - self.odom_yaw
                    )
                if self.pose_from_amcl:
                    self.pose_latest = (
                        f"x: {self.robot_pose.position.x:.3f}   "
                        f"y: {self.robot_pose.position.y:.3f}   "
                        f"yaw: {self.yaw:.3f} rad"
                    )
                else:
                    self.pose_latest = (
                        f"x: {fx:.3f}   "
                        f"y: {fy:.3f}   "
                        f"yaw: {self.plan_start_yaw:.3f} rad "
                        f"(from plan)"
                    )
            self.node.get_logger().info(
                f"[{self.name}] /plan received: {len(self.planner_path)} pts "
                f"frame={getattr(msg.header, 'frame_id', '?')!r}"
                f" odom->map={self.odom_to_map}"
            )
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] plan_cb failed: {e}")

    def _live_map_yaw(self):
        """Best continuously-updating map-frame heading.

        Re-anchor the latest odom heading into the map frame using the learned
        rotation offset (from a plan start pose or AMCL while it was fresh), so
        the heading arrow turns in real time as the robot drives - even between
        /plan re-publishes or while AMCL is silent."""
        if self.odom_to_map_yaw is not None and self.odom_yaw is not None:
            return _norm_angle(self.odom_yaw + self.odom_to_map_yaw)
        if self.plan_start_yaw is not None:
            return self.plan_start_yaw
        return self.yaw

    def map_odom_cb(self, msg):
        """SLAM's map <- odom correction, published while exploring.

        This is a geometry_msgs/msg/TransformStamped whose transform composes
        with the odom pose to give the map-frame pose. Only the planar part
        matters here (the 2D map is x/y); roll/pitch are dropped.
        """
        try:
            t = msg.transform.translation
            q = msg.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.map_from_odom = (t.x, t.y, yaw)
            self.map_from_odom_stamp = time.time()
        except Exception as e:
            self.node.get_logger().error(
                f"[{self.name}] map_odom_cb failed: {e}")

    def _odom_in_map(self):
        """Odom pose lifted into the map frame by SLAM's correction, or None.

        Same composition the 3D viewer uses: translate the odom position by the
        correction and add the correction's rotation to the odom heading.
        """
        if self.map_from_odom is None or self.last_odom is None:
            return None
        if time.time() - self.map_from_odom_stamp > ODOM_FRESH_SECS:
            return None
        dx, dy, dyaw = self.map_from_odom
        ox, oy = self.last_odom
        c, s = math.cos(dyaw), math.sin(dyaw)
        yaw = _norm_angle((self.odom_yaw if self.odom_yaw is not None
                           else self.yaw) + dyaw)
        return (ox * c - oy * s + dx, ox * s + oy * c + dy, yaw)

    def map_pose(self):
        """Best available robot position expressed in the 'map' frame.

        Returns (x, y, yaw) or None. Order of preference:
          1. AMCL pose, as long as it is still publishing (fresh samples). This
             is the normal path whenever the robot is driving to a goal, so
             navigating behaviour is unchanged by the exploring fallback below.
          2. Odom lifted into the map frame by SLAM's map<-odom correction.
             Used while exploring, where AMCL is silent and Nav2 publishes no
             plan, and therefore no other map-frame anchor exists.
          3. Odom re-anchored into the map frame via the offset taken from the
             Nav2 plan's start pose (used while navigating without AMCL / when
             AMCL stalls, and kept after arrival so the body does not jump
             frames). The heading here is LIVE (re-anchored odom yaw).
          4. Nav2 plan start pose (map frame).
          5. Raw odom (unreliable frame, last resort before any map data)."""
        amcl_fresh = (self.pose_from_amcl
                      and time.time() - self._last_amcl_wall <= AMCL_FRESH_SECS)
        if amcl_fresh and self.robot_pose is not None:
            return (self.robot_pose.position.x,
                    self.robot_pose.position.y, self.yaw)
        lifted = self._odom_in_map()
        if lifted is not None:
            return lifted
        if (self.odom_to_map is not None and self.last_odom is not None):
            yaw = self._live_map_yaw()
            return (self.last_odom[0] + self.odom_to_map[0],
                    self.last_odom[1] + self.odom_to_map[1], yaw)
        if self.planner_path:
            fx, fy = self.planner_path[0]
            yaw = self._live_map_yaw()
            return (fx, fy, yaw)
        if self.robot_pose is not None:
            return (self.robot_pose.position.x,
                    self.robot_pose.position.y, self.yaw)
        return None

    @staticmethod
    def _nal_units(raw):
        """Yield (offset, start_code_size) for each Annex-B start code in raw."""
        i = 0
        n = len(raw) - 4
        while i < n:
            if raw[i] == 0 and raw[i + 1] == 0:
                if raw[i + 2] == 1:
                    yield i, 3
                    i += 3
                elif raw[i + 2] == 0 and raw[i + 3] == 1:
                    yield i, 4
                    i += 4
                else:
                    i += 1
            else:
                i += 1

    def _capture_headers(self, raw):
        """Cache any SPS(7)/PPS(8) NAL units found in raw (they are rare, so we
        only re-scan for them when they appear near a keyframe / on startup)."""
        units = list(self._nal_units(raw))
        for k, (pos, sz) in enumerate(units):
            hdr = pos + sz
            if hdr >= len(raw):
                continue
            nal_type = raw[hdr] & 0x1F
            end = units[k + 1][0] if k + 1 < len(units) else len(raw)
            if nal_type == 7:
                self._sps = raw[pos:end]
            elif nal_type == 8:
                self._pps = raw[pos:end]

    def camera_cb(self, msg):
        """Append H.264 bytes from Go2FrontVideoData to the rolling buffer and
        cache SPS/PPS so the stream stays decodable as the buffer scrolls."""
        try:
            data = bytes(msg.data)
            with self._cam_lock:
                self._cam_buf.extend(data)
                # keep a bounded trailing window (enough for a full IDR+frames)
                if len(self._cam_buf) > CAM_BUFFER_MAX:
                    del self._cam_buf[: len(self._cam_buf) - CAM_BUFFER_MAX]
                # Occasionally scan for SPS/PPS (cheap: only the newest bytes).
                if len(self._cam_buf) < 1024 * 1024 or len(self._cam_buf) % (512 * 1024) < len(data):
                    self._capture_headers(data)
            self.cam_stamp = self.node.get_clock().now().nanoseconds / 1e9
        except Exception as e:
            self.node.get_logger().error(f"Camera buffer failed: {e}")

    def _on_qod_frame(self, frame):
        """Queue a decoded frame from the QOD WebRTC consumer for smooth
        playback. draw_camera() pops from the same _cam_frames deck used by the
        ROS decode path, so the UI code is shared. The consumer hands us BGR;
        Gradio's gr.Image component displays RGB, so convert here once (this is
        what caused the blue tint)."""
        try:
            if frame.ndim == 3 and frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w = frame.shape[:2]
            if w > CAM_W or h > CAM_H:
                frame = cv2.resize(frame, (CAM_W, CAM_H),
                                   interpolation=cv2.INTER_AREA)
            with self._cam_lock:
                self._cam_frames.append(frame)
                self.color_frame = frame
            self.cam_frame_ts = time.time()
            self.cam_stamp = time.time()
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] QOD frame failed: {e}")

    @staticmethod
    def _trim_to_keyframe(raw):
        """Trim a raw H.264 Annex-B buffer so it starts at the most recent
        COMPLETE SPS->PPS->IDR sequence (NAL types 7 -> 8 -> 5) and ends just
        before the NEXT SPS. Starting at a bare SPS with no following PPS/IDR
        (or mid-GOP) is what produces the ffmpeg 'top block unavailable for
        requested intra mode' errors and a near-zero frame yield, which makes
        the camera look frozen. Scanning the whole buffer for the last clean
        GOP start gives the decoder a perfect SPS+PPS+IDR+P.. sequence to sync."""
        def start_codes(data):
            i = 0
            n = len(data) - 4
            pos = []
            while i < n:
                if data[i] == 0 and data[i + 1] == 0:
                    if data[i + 2] == 1:
                        pos.append((i, 3))
                        i += 3
                    elif data[i + 2] == 0 and data[i + 3] == 1:
                        pos.append((i, 4))
                        i += 4
                    else:
                        i += 1
                else:
                    i += 1
            return pos

        codes = start_codes(raw)
        if not codes:
            return None

        def nal_type_at(idx):
            pos, sz = codes[idx]
            hdr = pos + sz
            if hdr >= len(raw):
                return None
            return raw[hdr] & 0x1F

        # Find the newest REFERENCE frame (NAL 5 = IDR, or NAL 1 slice) to cut at.
        # We need a real I/mixed slice so the decoder has a reference to start.
        # Prefer an IDR(5); else any NAL-1 slice after the newest SPS->PPS pair.
        ref = None
        for i in range(len(codes)):
            t = nal_type_at(i)
            if t == 5:  # IDR - guaranteed clean reference
                ref = i
        if ref is None:
            # Fall back: last NAL-1 slice that could start a new frame.
            for i in range(len(codes)):
                if nal_type_at(i) == 1:
                    ref = i
        if ref is None:
            # Fall back to the newest SPS->PPS pair (stream may have no I slice yet)
            for i in range(len(codes) - 1):
                if nal_type_at(i) == 7 and nal_type_at(i + 1) == 8:
                    ref = i
        if ref is None:
            return None

        # Prefer starting at the newest COMPLETE SPS->PPS->IDR(5) sequence so the
        # returned segment is self-contained and does not rely on a cached
        # parameter set that the camera may have renegotiated (a stale SPS+PPS
        # prepended to newer slices is what makes cv2 yield zero frames forever).
        best = codes[ref][0]
        for i in range(len(codes) - 2):
            if (nal_type_at(i) == 7 and nal_type_at(i + 1) == 8
                    and nal_type_at(i + 2) == 5 and codes[i][0] <= best):
                best = codes[i][0]
                break

        start = best
        # Cut before the NEXT SPS (a new parameter-set = a new clean access unit),
        # so the returned segment is a single self-contained GOP.
        end = len(raw)
        for j in range(ref + 1, len(codes)):
            if nal_type_at(j) == 7:
                end = codes[j][0]
                break
        return raw[start:end]

    def _camera_decode_loop(self):
        """Background decode thread: repeatedly decode the newest H.264 keyframe
        segment and fill a buffer of frames. draw_camera() plays those frames at
        full speed for smooth video while this thread refills in the background."""
        while not self._stop_cam:
            try:
                if self.cam_stamp:
                    batch = self._decode_camera_batch()
                    if batch:
                        with self._cam_lock:
                            self._cam_frames.extend(batch)
                        self.cam_frame_ts = time.time()
                    else:
                        # Diagnostic: surface WHY no frames decoded (throttled).
                        now = time.time()
                        if now - getattr(self, "_diag_last", 0) > 5.0 and self.cam_fps < 0.5:
                            self._diag_last = now
                            with self._cam_lock:
                                buf = len(self._cam_buf)
                            raw = bytes(self._cam_buf[-CAM_DECODE_WINDOW:]) if buf else b""
                            trim = self._trim_to_keyframe(raw) if raw else None
                            self.node.get_logger().warning(
                                f"[{self.name}] cam decode empty: buf={buf}B "
                                f"window={len(raw)}B trim={'yes' if trim else 'NO-KEYFRAME'}"
                            )
            except Exception as e:
                self.node.get_logger().error(f"Camera decode failed: {e}")
            threading.Event().wait(CAM_DECODE_INTERVAL)

    def _decode_camera_batch(self):
        """Decode the newest bytes into a list of frames, staying continuous.

        Strategy: instead of requiring a keyframe to be present in the rolling
        window (which fails once the buffer scrolls past the one-time SPS/IDR,
        causing the camera to freeze permanently), we refresh the cached SPS+PPS
        from the newest bytes each decode and, when available, split at the
        newest SPS->PPS->IDR sequence so the decoder always has a valid reference
        to start from. If a decode yields zero frames, the cached parameter sets
        may be stale (the camera renegotiated its encoder) -- we clear them and
        retry once against a freshly assembled segment so the stream self-heals.
        """
        if not self._cam_buf:
            return []
        with self._cam_lock:
            sps, pps = self._sps, self._pps
            raw = bytes(self._cam_buf[-CAM_DECODE_WINDOW:])
        if len(raw) < CAM_MIN_BYTES:
            return []

        def assemble(s, p):
            r = self._trim_to_keyframe(raw)
            if r is None:
                return None
            if s:
                # Prepend the (refreshed) parameter sets so the decoder always
                # has them, even after the keyframe scrolled out of the buffer.
                return s + p + r
            return r

        seg = assemble(sps, pps)
        if seg is None:
            return []
        frames = self._decode_segment(seg)
        if frames:
            return frames

        # Zero frames from a valid keyframe segment -> cached params are stale.
        # Refresh SPS/PPS from the newest bytes and retry once.
        with self._cam_lock:
            self._capture_headers(bytes(self._cam_buf[-CAM_DECODE_WINDOW:]))
            sps, pps = self._sps, self._pps
        seg = assemble(sps, pps)
        if seg is None:
            return []
        return self._decode_segment(seg)

    def _decode_segment(self, raw):
        """Write raw H.264 bytes to a temp file and decode up to CAM_FRAMES_MAX
        frames. Returns [] on failure so callers can trigger self-healing."""
        tmp = f"/tmp/{self.name}_cam.h264"
        with open(tmp, "wb") as f:
            f.write(raw)
        cap = cv2.VideoCapture(tmp)
        frames = []
        while len(frames) < CAM_FRAMES_MAX:
            ret, f = cap.read()
            if not ret:
                break
            h, w = f.shape[:2]
            if w > CAM_W or h > CAM_H:
                f = cv2.resize(f, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)
            frames.append(f)
        cap.release()
        return frames

    def decode_camera(self):
        """Return the latest decoded frame instantly (decode runs in a thread).
        Never blocks the UI: there is always a cached frame or the placeholder."""
        return self.color_frame

    def battery_cb(self, msg):
        self.battery = msg.bms_state.soc
        self.motor_status = [m.temperature for m in msg.motor_state if m.mode == 1]
        self.roll = msg.imu_state.rpy[0]
        self.pitch = msg.imu_state.rpy[1]
        self.imu_yaw = msg.imu_state.rpy[2]
        self.voltage = msg.power_v
        self.current = msg.power_a
        self.power = self.voltage * self.current
        self.bat_stamp = self.node.get_clock().now().nanoseconds / 1e9

    # ---------------------- rendering ----------------------
    def draw_map(self):
        if self.cached_map_img is None:
            return _blank(640, 480)

        canvas = self.cached_map_img.copy()
        scale = self.cached_scale
        origin_x, origin_y, h, ox, oy, res = self.cached_meta

        def to_px(wx, wy):
            mx = (wx - origin_x) / res
            my = (wy - origin_y) / res
            my = h - 1 - my
            return int(mx * scale) + ox, int(my * scale) + oy

        mp = self.map_pose()
        if mp is not None:
            body_x, body_y, body_yaw = mp
            # Nav2 planned path (the route the robot is about to take). Color
            # matches the robot: Luna = red, Astro = blue (same as its dot).
            colors = ROBOT_COLORS[self.name]
            if self.planner_path and len(self.planner_path) >= 2:
                pts = np.array(
                    [to_px(wx, wy) for wx, wy in self.planner_path], dtype=np.int32
                )
                _draw_plan(canvas, pts, colors["path_dark"], colors["path_bright"])

            px, py = to_px(body_x, body_y)
            dot = colors["fill"]
            cv2.circle(canvas, (px, py), 10, dot, -1)
            ex = int(px + 30 * math.cos(body_yaw))
            ey = int(py - 30 * math.sin(body_yaw))
            cv2.arrowedLine(canvas, (px, py), (ex, ey), (0, 0, 0), 3)

            canvas = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)
            hc, wc = canvas.shape[:2]
            cv2.putText(canvas, "N", (wc // 2, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
            cv2.putText(canvas, "S", (wc // 2, hc - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
        return canvas

    def _draw_detections(self, frame):
        """Draw the qod cv-worker's object detections on the camera frame -
        the exact same stream (Redis channel detections:<source>) and class
        styling the qod website uses, so both UIs match. Boxes arrive
        normalized 0..1 relative to the original 1280x720 feed; the displayed
        frame keeps the same aspect ratio, so we scale linearly to its size."""
        sub = getattr(self, "_qod_detections", None)
        payload = sub.detections() if sub is not None else []
        if not payload:
            return frame
        h, w = frame.shape[:2]
        outer = max(2, w // 170)   # dark under-stroke
        inner = max(1, outer - 1)  # coloured core - same look as the site
        font = cv2.FONT_HERSHEY_SIMPLEX
        for det in payload:
            try:
                cls = str(det.get("class") or "").strip().lower()
                conf = float(det.get("confidence") or 0.0)
                if conf < DETECTIONS_MIN_CONF:
                    continue
                bbox = det.get("bbox") or {}
                x = float(bbox["x"])
                y = float(bbox["y"])
                bw = float(bbox["width"])
                bh = float(bbox["height"])
                # The site accepts normalized (0..1) boxes and also raw pixel
                # boxes; we only draw normalized ones, like the YOLO publisher.
                if not (0 <= x <= 1.1 and 0 <= y <= 1.1 and
                        0 <= bw <= 1.1 and 0 <= bh <= 1.1):
                    continue
                style = DETECTION_CLASS_STYLES.get(cls)
                if style is None:
                    # Same fallback as the site: an unstyled class keeps its
                    # raw name (the model also emits classes with no style
                    # entry, e.g. "old_spill"); only a nameless class becomes
                    # the generic "Object".
                    style = dict(DEFAULT_DETECTION_STYLE)
                    if cls:
                        style["alias"] = cls
                label = f"{style['alias']} {conf * 100:.0f}%"
                px = int(x * w)
                py = int(y * h)
                pw = max(1, int(bw * w))
                ph = max(1, int(bh * h))
                pt1 = (px, py)
                pt2 = (px + pw, py + ph)
                cv2.rectangle(frame, pt1, pt2, (0, 0, 0), outer)
                cv2.rectangle(frame, pt1, pt2, style["color"], inner)
                (tw, th), baseline = cv2.getTextSize(
                    label, font, 0.45, 1
                )
                lx = px
                ly = max(0, py - (th + baseline + 4))
                cv2.rectangle(
                    frame, (lx, ly), (lx + tw + 6, ly + th + 6),
                    (0, 0, 0), -1,
                )
                cv2.putText(
                    frame, label, (lx + 3, ly + th + 2), font, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA,
                )
            except Exception:
                continue
        return frame

    def draw_camera(self):
        # Play smoothly from the decoded frame queue (refilled in the
        # background). When it runs dry, hold the last frame until the next
        # batch is decoded -- gives fluid motion instead of one stamp per tick.
        frame = None
        now = time.time()
        if now - self._cam_played_ts >= 1.0:
            self.cam_fps = self._cam_played / (now - self._cam_played_ts)
            self._cam_played = 0
            self._cam_played_ts = now
        with self._cam_lock:
            if self._cam_frames:
                frame = self._cam_frames.popleft()
                self.color_frame = frame
                self._cam_played += 1
        if frame is None:
            frame = self.color_frame
        if frame is None:
            return _placeholder(640, 360, f"WAITING FOR {self.name.upper()} CAMERA...")
        h, w = frame.shape[:2]
        if w > 640 or h > 360:
            frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
        return self._draw_detections(frame)

    # ---------------------- text getters ----------------------
    def get_battery_data(self):
        if self.battery is None:
            return f"Waiting for {self.name} battery..."
        if self.battery >= 70:
            col = "🟢"
        elif self.battery >= 30:
            col = "🟡"
        elif self.battery >= 20:
            col = "🔴"
        else:
            col = "🚨"
        text = f"🔋 {col} {self.battery} %"
        if self.power is not None:
            text += f"  🔌 {self.power:.1f} W ({self.voltage:.1f}V, {self.current:.1f}A)"
        return text

    def get_motor_data(self):
        if self.motor_status is None:
            return f"Waiting for {self.name} motor temps..."
        legs = ["FL", "FR", "RL", "RR"]
        out = []
        for i, leg in enumerate(legs):
            base = i * 3
            out.append(
                f"{leg} hip {self.motor_status[base]:.0f}°  "
                f"thigh {self.motor_status[base+1]:.0f}°  "
                f"calf {self.motor_status[base+2]:.0f}°"
            )
        return "\n".join(out)

    def get_orientation_data(self):
        if self.roll is None:
            return f"Waiting for {self.name} IMU..."
        text = (f"Roll {self.roll:.3f}   Pitch {self.pitch:.3f}   "
                f"Yaw {self.imu_yaw:.3f} rad")
        if abs(self.roll) > 0.5 or abs(self.pitch) > 0.5:
            text += "\n🚨 Robot UNSTABLE!"
        return text

    def get_pose_data(self):
        mp = self.map_pose()
        if mp is None:
            return self.pose_latest
        bx, by, byaw = mp
        pos = (f"x: {bx:.3f}   y: {by:.3f}   yaw: {byaw:.3f} rad")
        return pos

    def get_connection_status(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        parts = []
        for name, ts in (
            ("MAP", self.map_stamp), ("CAM", self.cam_stamp),
            ("BAT", self.bat_stamp),
        ):
            if ts > 0 and now - ts < 5.0:
                parts.append(f"🟢 {name}")
            elif ts > 0:
                parts.append(f"🟡 {name} (stale)")
            else:
                parts.append(f"⚪ {name} (waiting)")
        cam = f"camera {self.cam_fps:.1f} fps" if self.cam_fps > 0.5 else "camera no-frames"
        return "   ".join(parts) + f"   ·  {cam}"

    # ---------------------- initial pose ----------------------
    def _display_to_world(self, x, y):
        """Convert displayed-map image pixel coords (x = horizontal, y = vertical,
        matching Gradio's select evt.index = [x, y]) to world (wx, wy).
        Mirrors the inverse transform used by draw_map()."""
        if self.cached_meta is None:
            return None, None
        origin_x, origin_y, h, ox, oy, res = self.cached_meta
        canvas = self.cached_map_img.shape[0]
        # Undo the 90-degree clockwise display rotation.
        px = y
        py = canvas - 1 - x
        # undo scaling / centering, then the y-flip
        mx = (px - ox) / self.cached_scale
        myp = (py - oy) / self.cached_scale
        wx = mx * res + origin_x
        wy = (h - 1 - myp) * res + origin_y
        return wx, wy

    def set_initial_pose_marker(self, wx, wy, yaw):
        """Remember a submitted exhibition pose so it is drawn on the map."""
        self.initial_pose = (float(wx), float(wy), float(yaw))
        self.last_initial = (float(wx), float(wy), float(yaw))

    def pose_from_drag(self, press, release):
        """press/release = (x, y) displayed-pixel coords of a click-drag.
        Position = press point, yaw = direction of the drag. Mirrors Foxglove's
        "2D Publish Pose" tool."""
        if self.cached_meta is None:
            raise ValueError(f"No {self.name} map yet (drag on the map once it loads)")
        x1, y1 = press
        x2, y2 = release
        wx, wy = self._display_to_world(x1, y1)
        if wx is None:
            raise ValueError(f"No {self.name} map yet")
        # world-frame yaw from the drag vector (see transform derivation)
        yaw = math.atan2(x2 - x1, y2 - y1)
        return float(wx), float(wy), float(yaw)
class LabRobotNode(Node):
    """Subscribes to map/pose/odom/camera/battery for every robot and exposes
    the currently-selected robot for rendering."""

    def __init__(self):
        super().__init__("lab_portal_node")
        self.robots = {}
        for name, topics in ROBOTS.items():
            robot = RobotState(self, name, topics)

            # All subscriptions are optional: ROS waits silently for any topic.
            # The map publishers (map_server / completed-map relays) use
            # TRANSIENT_LOCAL durability, so we MUST subscribe with the same
            # durability, otherwise no map message is ever delivered.
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, topics["map"], robot.map_cb,
                                     qos_profile=map_qos)
            # Per-robot AMCL pose (/luna/amcl_pose, /astro/amcl_pose) is
            # geometry_msgs/msg/PoseWithCovarianceStamped and gives the accurate
            # localized position for drawing each robot on the map.
            self.create_subscription(PoseWithCovarianceStamped, topics["pose"],
                                     robot.pose_cb, 10)
            # Live odometry. The robot odom topics here are bridged by
            # zenoh_bridge_ros2dds, which publishes BEST_EFFORT; a RELIABLE
            # subscription is QoS-incompatible and silently receives nothing
            # (dead robot body/heading on the map). Use BEST_EFFORT + deep
            # history so odom - and therefore the live heading - always flows.
            odom_qos = QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(Odometry, topics["odom"], robot.odom_cb,
                                     qos_profile=odom_qos)
            # Nav2's global planner publishes the path currently being followed
            # on /luna|astro/plan; draw it on the map so the operator can see the
            # route the robot is about to take before/while it drives.
            self.create_subscription(Path, topics["plan"], robot.plan_cb, 10)
            # SLAM's map<-odom correction, published while exploring. This is the
            # only map-frame anchor available then: AMCL is silent and Nav2
            # publishes no plan, so without it the odom body is drawn in the
            # wrong frame. TRANSIENT_LOCAL like the other latched SLAM topics.
            map_odom_topic = topics.get("map_odom")
            if map_odom_topic:
                self.create_subscription(
                    TransformStamped, map_odom_topic, robot.map_odom_cb,
                    qos_profile=map_qos)
            if Go2FrontVideoData is not None and CAMERA_SOURCE != "webrtc":
                # Camera arrives as high-rate (~250Hz+) fragmented H.264. A
                # RELIABLE depth-10 subscription drops messages under the burst
                # load, which slices gaps into the H.264 stream and makes it
                # impossible to decode any frame. Use BEST_EFFORT + a deep
                # history so the newest contiguous bytes arrive without gap.
                # (Skipped in "webrtc" mode: the camera comes from the QOD SFU.)
                cam_qos = QoSProfile(
                    depth=100,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                )
                self.create_subscription(
                    Go2FrontVideoData, topics["camera"], robot.camera_cb,
                    qos_profile=cam_qos,
                )
            if LowState is not None:
                self.create_subscription(LowState, topics["battery"], robot.battery_cb, 10)

            self.robots[name] = robot
            self.get_logger().info(
                f"[{name}] Subscribed to {topics['map']}, {topics['pose']}, "
                f"{topics['odom']}, {topics['camera']}, {topics['battery']}"
            )

        self.current = "Luna"

        threading.Thread(target=self._diag_loop, daemon=True).start()

    def _diag_loop(self):
        """Periodic heartbeat: log what each robot is actually receiving so we
        can confirm map/pose/camera delivery. Diagnostic only."""
        while True:
            try:
                for name, robot in self.robots.items():
                    live_yaw = robot._live_map_yaw()
                    if (robot.pose_from_amcl
                            and time.time() - robot._last_amcl_wall
                            <= AMCL_FRESH_SECS):
                        src = "amcl"
                    elif robot._odom_in_map() is not None:
                        src = "slam"
                    elif robot.odom_to_map_yaw is not None:
                        src = "odom+off"
                    elif robot.plan_start_yaw is not None:
                        src = "plan"
                    else:
                        src = "odom"
                    self.get_logger().info(
                        f"[DIAG] {name}: map={robot.cached_map_img is not None} "
                        f"pose={'y' if robot.robot_pose is not None else 'n'} "
                        f"plan={'y' if robot.planner_path and len(robot.planner_path) >= 2 else 'n'} "
                        f"plan_pts={len(robot.planner_path) if robot.planner_path else 0} "
                        f"pose_txt={robot.pose_latest!r} "
                        f"hdg={src} "
                        f"map_yaw={(live_yaw if live_yaw is not None else float('nan')):.2f} "
                        f"cam_frame={'y' if robot.color_frame is not None else 'n'} "
                        f"cam_ts={robot.cam_frame_ts:.1f} cam_fps={robot.cam_fps:.2f} "
                        f"det={len(robot._qod_detections.detections()) if getattr(robot, '_qod_detections', None) else 'n/a'}"
                    )
            except Exception as e:
                self.get_logger().error(f"[DIAG] failed: {e}")
            threading.Event().wait(5.0)

    # ---- the active robot's data ----
    @property
    def current_robot(self):
        return self.robots[self.current]

    def set_robot(self, name):
        if name in self.robots:
            self.current = name
            return f"Showing: {name}"
        return f"Unknown robot: {name}"

    def draw_map(self):
        return self.current_robot.draw_map()

    def draw_camera(self):
        return self.current_robot.draw_camera()

    def get_battery_data(self):
        return self.current_robot.get_battery_data()

    def get_motor_data(self):
        return self.current_robot.get_motor_data()

    def get_orientation_data(self):
        return self.current_robot.get_orientation_data()

    def get_pose_data(self):
        return self.current_robot.get_pose_data()

    def get_connection_status(self):
        return self.current_robot.get_connection_status()

    def pose_from_drag(self, press, release):
        return self.current_robot.pose_from_drag(press, release)

    # ---- patrolling page ----
    def draw_patrol_map(self):
        """Render both robots, their paths and goals on a single occupancy
        grid.  The base map is taken from whichever robot has a live occupancy
        grid; every robot's pose, Nav2 plan and goal marker are overlaid."""
        # Pick the base map from whichever robot has one.
        base = None
        label_positions = []
        for r in self.robots.values():
            if r.cached_map_img is not None:
                base = r
                break
        if base is None:
            return _blank(640, 480)

        canvas = base.cached_map_img.copy()
        scale = base.cached_scale
        ox, oy, h, pox, poy, res = base.cached_meta

        def to_px(wx, wy):
            mx = (wx - ox) / res
            my = (wy - oy) / res
            my = h - 1 - my
            return int(mx * scale) + pox, int(my * scale) + poy

        # Draw each robot's plan + goal (same logic as single-robot draw_map).
        for name, robot in self.robots.items():
            mp = robot.map_pose()
            if mp is None:
                continue
            body_x, body_y, body_yaw = mp
            colors = ROBOT_COLORS.get(name, ROBOT_COLORS["Luna"])
            # Nav2 planned path (the route the robot is about to take), drawn in
            # the robot's colour with a light casing + black under-stroke.
            if robot.planner_path and len(robot.planner_path) >= 2:
                pts = np.array(
                    [to_px(wx, wy) for wx, wy in robot.planner_path],
                    dtype=np.int32,
                )
                _draw_plan(canvas, pts, colors["path_dark"], colors["path_bright"])

            fill = colors.get("fill", (255, 0, 0))
            outline = colors.get("outline", (180, 0, 0))
            px, py = to_px(body_x, body_y)
            cv2.circle(canvas, (px, py), 10, fill, -1)
            cv2.circle(canvas, (px, py), 10, outline, 2)
            ex = int(px + 30 * math.cos(body_yaw))
            ey = int(py - 30 * math.sin(body_yaw))
            cv2.arrowedLine(canvas, (px, py), (ex, ey), (0, 0, 0), 3)
            label_positions.append((px, py, name))

        canvas = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)
        hc, wc = canvas.shape[:2]

        # Robot labels.
        for px, py, name in label_positions:
            # 90-degree CW rotation: dst(row = src_col, col = H-1-src_row).
            # Source row = py, col = px -> dst col = H-1-py, dst row = px.
            # cv2.putText's (x, y) is (col, baseline-row).
            lx = wc - 1 - py + 5
            ly = px - 5
            cv2.putText(canvas, name, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(canvas, name, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.putText(canvas, "N", (wc // 2, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 2)
        cv2.putText(canvas, "S", (wc // 2, hc - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 2)

        return canvas

    def patrol_draw_cameras(self):
        """Return both robots' camera streams for the fleet camera panel."""
        luna = self.robots.get("Luna")
        astro = self.robots.get("Astro")
        return (
            luna.draw_camera() if luna else _placeholder(640, 360, "LUNA OFFLINE"),
            astro.draw_camera() if astro else _placeholder(640, 360, "ASTRO OFFLINE"),
        )

    def patrol_get_status(self):
        """Per-robot telemetry (connection + pose) for the fleet footer."""
        luna = self.robots.get("Luna")
        astro = self.robots.get("Astro")
        if luna is None or astro is None:
            return ("LUNA unavailable", "—", "ASTRO unavailable", "—")
        return (
            luna.get_connection_status(),
            luna.get_pose_data(),
            astro.get_connection_status(),
            astro.get_pose_data(),
        )


def main():
    rclpy.init(args=None)
    node = LabRobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # Side-car for the "3D Map" tab. Started here (rather than from
    # boot.sh / lab_start.sh) so one deploy unit covers everything, but in its
    # own process and behind its own health check, so it can neither delay nor
    # take down this dashboard.
    try:
        _start_foxglove_viewer()
    except Exception as _exc:  # noqa: BLE001
        print(f"[foxglove] could not start viewer supervisor: {_exc!r}", flush=True)

    theme = gr.themes.Soft(
        primary_hue=gr.themes.colors.blue,
        secondary_hue=gr.themes.colors.blue,
        neutral_hue=gr.themes.colors.slate,
        font="Helvetica, 'Helvetica Neue', Arial, sans-serif",
    )

    logo_uri = logo_data_uri()
    logo_tag = f'<img class="logo" src="{logo_uri}" alt="logo"/>' if logo_uri else ""
    header_html = f"""
    <div class="portal-header">
        {logo_tag}
        <div>
            <div class="title">5G-Enabled Multi-Agent Mission Dashboard</div>
            <div class="subtitle">E-12-DMAT-025 · Luna &amp; Astro · live telemetry from the lab</div>
        </div>
    </div>
    """

    with gr.Blocks(title="5G-Enabled Multi-Agent Mission Dashboard", theme=theme, css=DASHBOARD_CSS, head=INIT_POSE_JS) as demo:
        gr.HTML(header_html)

        with gr.Tabs() as page_tabs:
            # ================================================================
            # TAB 1: MAPPING (existing single-robot dashboard)
            # ================================================================
            with gr.Tab("Mapping", id="mapping"):

                # ================================================================
                # TOP ROW: ROBOT FLEET (LEFT) + MAP (CENTER) + CAMERA (RIGHT)
                # ================================================================
                with gr.Row():
                    # ---------- LEFT: ROBOT FLEET ----------
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        gr.Markdown("#### ROBOT FLEET")
                        robot_dd = gr.Dropdown(
                            choices=list(ROBOTS.keys()),
                            value=node.current,
                            label="Select Robot",
                            interactive=True,
                            elem_id="robot_selector",
                            elem_classes=["robot-selector"],
                        )

                        # Luna card
                        with gr.Group(elem_classes=["robot-card"]):
                            with gr.Row():
                                gr.HTML('<div style="display:flex;justify-content:space-between;align-items:center;"><h3 style="margin:0;">Luna</h3><div>🟢 Connected</div></div>')
                            luna_map_status = gr.Textbox(label="Map", lines=1, interactive=False)
                            luna_pose_status = gr.Textbox(label="Pose", lines=1, interactive=False)
                            luna_cam_status = gr.Textbox(label="Camera", lines=1, interactive=False)

                        # Astro card
                        with gr.Group(elem_classes=["robot-card"]):
                            with gr.Row():
                                gr.HTML('<div style="display:flex;justify-content:space-between;align-items:center;"><h3 style="margin:0;">Astro</h3><div>🟢 Connected</div></div>')
                            astro_map_status = gr.Textbox(label="Map", lines=1, interactive=False)
                            astro_pose_status = gr.Textbox(label="Pose", lines=1, interactive=False)
                            astro_cam_status = gr.Textbox(label="Camera", lines=1, interactive=False)

                    # ---------- CENTER: LIVE 3D MAPPING ----------
                    with gr.Column(scale=2, elem_classes=["map-panel"]):
                        gr.HTML('<h2 style="text-align:center;margin:0 0 8px;">Live Mapping</h2>')

                        # "Set Initial Pose" is hidden by default and only revealed
                        # via the toggle button below, so the map stays clean for
                        # normal map viewing. Hiding also un-checks the toggle so
                        # the drag gesture is never silently active while hidden.
                        initpose_btn = gr.Button(
                            "Set Initial Pose", interactive=False,
                            elem_id="initpose_toggle_btn")
                        initpose_visible = gr.State(value=False)

                        with gr.Column(visible=False, elem_id="initpose_section") as initpose_section:
                            init_toggle = gr.Checkbox(
                                label="Set Initial Pose — drag on the map (position = start, "
                                      "orientation = drag direction)",
                                value=False,
                                elem_id="initpose_toggle",
                            )

                        map_img = gr.Image(show_label=False,
                                           type="numpy", elem_id="map_image", height=520)

                        # Jump to the 3D view of the same world. Navigation only:
                        # no data path,
                        # subscription or timer is touched, and the 2D map keeps
                        # running untouched on its own tab.
                        switch_to_3d_btn = gr.Button(
                            "Switch to 3D", variant="secondary",
                            elem_classes=["switch-view-btn"])
                        switch_to_3d_btn.click(
                            _goto_tab("foxglove-ui"),
                            inputs=None, outputs=page_tabs)

                        map_interaction_out = gr.Textbox(
                            label="Map Interaction", lines=1, interactive=False)

                        with gr.Row():
                            init_status = gr.Textbox(label="🟢 Initial Pose Status", lines=1,
                                                     interactive=False, scale=2)
                            init_bridge = gr.Textbox(visible=True, show_label=False,
                                                     elem_id="initpose_bridge", scale=0)

                        gr.HTML('<div style="text-align:center;">🔴 Luna · 🟦 Astro</div>')

                    # ---------- RIGHT: LIVE CAMERA ----------
                    with gr.Column(scale=1, elem_classes=["camera-panel"]):
                        gr.HTML('<h2 style="text-align:center;margin:0 0 8px;">Live Camera</h2>')
                        cam_img = gr.Image(label="Camera: selected robot",
                                           type="numpy", height=300)
                        cam_fps_out = gr.Textbox(label="Stream FPS", lines=1, interactive=False)

                # Exhibition workflow controls. hardware_code/exhibitionctl.py is
                # the authority for state, process ownership and motion gates.
                with gr.Row():
                    with gr.Column(scale=1, min_width=320):
                        gr.Markdown("#### MAPPING")
                        mapping_start_btn = gr.Button(
                            "Start Mapping", variant="primary", interactive=False)
                        mapping_stop_btn = gr.Button(
                            "Stop Mapping", variant="stop", interactive=False)
                        mapping_partial_btn = gr.Button(
                            "Stop & Save Partial Map", interactive=False)

                    with gr.Column(scale=1, min_width=320):
                        gr.Markdown("#### LOCALISATION")
                        map_dd = gr.Dropdown(
                            label="Completed Map (latest five)",
                            choices=[], value=None, interactive=True,
                            elem_classes=["map-selector"])
                        team_dd = gr.Dropdown(
                            label="Robot Team",
                            choices=[("Both robots", "both"), ("Luna only", "luna"),
                                     ("Astro only", "astro")],
                            value="both", interactive=True)
                        map_refresh_btn = gr.Button(
                            "Refresh Maps", variant="secondary")
                        localisation_prepare_btn = gr.Button(
                            "Prepare Localisation", variant="primary", interactive=False)
                        cleanup_btn = gr.Button(
                            "End Patrol & Clean Up", variant="stop", interactive=False)

                with gr.Row():
                    confirm_luna_btn = gr.Button(
                        "Confirm Luna Localisation", interactive=False)
                    confirm_astro_btn = gr.Button(
                        "Confirm Astro Localisation", interactive=False)

                mission_status_out = gr.Textbox(
                    label="Exhibition Workflow", lines=9, interactive=False)
                action_result_out = gr.JSON(label="Last Controller Result")

                # ================================================================
                # BOTTOM ROW: POSE + STATUS FOOTER
                # Battery / Orientation / Motor Temps were removed at the
                # operator's request. Their callbacks and the ROS
                # subscriptions that feed them are left in place: other
                # panels and the /health diagnostics still read the same
                # values, so only the Mapping page display is gone.
                # ================================================================
                pose_out = gr.Textbox(label="📍 Pose", lines=2)

                conn_out = gr.Textbox(label="Connection Status", lines=2, interactive=False)

                # --- tickers (each independently safe; missing data => placeholder) ---
                # Map is redrawn at 2 Hz: a full occupancy-grid image shipped to the
                # browser on every tick is the largest CPU/bandwidth driver. Slowing it
                # to 0.5s frees CPU so the camera stream (below) can refresh smoothly
                # instead of being starved by a 10 Hz map redraw.
                map_timer = gr.Timer(0.5)
                map_timer.tick(lambda: node.draw_map(), outputs=map_img)

                # Camera pulls decoded frames faster than before. Frames are produced by
                # a background decode thread, so a quick 0.1s pull yields fluid playback
                # without adding decode work -- it just drains the ready frame queue.
                cam_timer = gr.Timer(0.1)
                cam_timer.tick(lambda: node.draw_camera(), outputs=cam_img)

                status_timer = gr.Timer(1.0)
                status_timer.tick(
                    lambda: (
                        node.get_connection_status(),
                        node.get_pose_data(),
                        f"{node.current_robot.cam_fps:.1f} fps",
                    ),
                    outputs=[conn_out, pose_out, cam_fps_out],
                )

                # Populate per-robot status cards
                robot_status_timer = gr.Timer(2.0)

                def robot_status():
                    texts = []
                    for name in node.robots:
                        robot = node.robots[name]
                        now = node.get_clock().now().nanoseconds / 1e9
                        map_ok = "🟢 live" if robot.map_stamp > 0 and now - robot.map_stamp < 5.0 else ("🟡 stale" if robot.map_stamp > 0 else "⚪ waiting")
                        pose_ok = "🟢 live" if robot.robot_pose is not None else "⚪ waiting"
                        cam_ok = "🟢 live" if robot.cam_fps > 0.5 else "⚪ waiting"
                        texts.append(map_ok)
                        texts.append(pose_ok)
                        texts.append(cam_ok)
                    return texts

                robot_status_timer.tick(
                    robot_status,
                    outputs=[luna_map_status, luna_pose_status, luna_cam_status,
                             astro_map_status, astro_pose_status, astro_cam_status],
                )

                def on_map_select(mode, evt: gr.SelectData):
                    del evt
                    if mode:
                        return ("Initial Pose mode ON — drag from position toward the "
                                "desired heading, then wait for controller confirmation.")
                    return "Direct navigation goals are disabled in exhibition mode."

                def on_drag_payload(payload, mode):
                    if not mode:
                        return "Initial pose drag ignored — toggle \"Set Initial Pose\" ON."
                    if exhibition_control is None:
                        return f"Exhibition controls unavailable: {EXHIBITION_IMPORT_ERROR}"
                    if not payload or "," not in payload:
                        return "No drag data received."
                    try:
                        robot_label = node.current
                        x1, y1, x2, y2 = [int(x) for x in payload.split(",")]
                        x, y, yaw = node.pose_from_drag((x1, y1), (x2, y2))
                        result = exhibition_control.set_initial_pose(
                            robot_label, x, y, yaw)
                        node.robots[robot_label].set_initial_pose_marker(x, y, yaw)
                    except Exception as exc:
                        return f"Initial pose failed: {exc}"
                    return (f"{robot_label}: pose submitted at ({x:.3f}, {y:.3f}), "
                            f"yaw {yaw:.3f} rad. Confirm localisation separately. "
                            f"Workflow: {result.get('workflow_state', 'unknown')}")

                def _control_missing():
                    return {
                        "state": "error",
                        "error": f"exhibition_control unavailable: {EXHIBITION_IMPORT_ERROR}",
                    }

                def _run(action, *args):
                    if exhibition_control is None:
                        return _control_missing()
                    try:
                        return action(*args)
                    except Exception as exc:
                        return {"state": "error", "error": str(exc)}

                def _run_named(name, *args):
                    if exhibition_control is None:
                        return _control_missing()
                    return _run(getattr(exhibition_control, name), *args)

                def _mission_summary(state):
                    workflow = state.get("workflow_state") or state.get("state", "unknown")
                    explorer = state.get("selected_explorer") or "none"
                    team = ", ".join(state.get("patrol_robots") or []) or "none"
                    handover = state.get("map_handover") or {}
                    map_info = handover.get("yaml") or (state.get("map") or {}).get("yaml") or "none"
                    localisation = state.get("localisation") or {}
                    ready = []
                    for name in ("luna", "astro"):
                        item = localisation.get(name) or {}
                        ready.append(
                            f"{name.title()}: pose={'yes' if item.get('pose_submitted') else 'no'}, "
                            f"confirmed={'yes' if item.get('confirmed') else 'no'}"
                        )
                    processes = state.get("robots") or {}
                    process_lines = []
                    for name in ("luna", "astro"):
                        item = processes.get(name) or {}
                        process_lines.append(
                            f"{name.title()} process: {item.get('mode') or 'none'} / "
                            f"{item.get('health') or item.get('state') or 'unknown'}"
                        )
                    error = state.get("last_error") or state.get("error") or "none"
                    return (f"State: {workflow}\nExplorer: {explorer}\nTeam: {team}\n"
                            f"Map: {map_info}\n" + "\n".join(process_lines + ready)
                            + f"\nError: {error}")

                def on_mission_tick():
                    state = _run(exhibition_control.mission_status) \
                        if exhibition_control is not None else _control_missing()
                    workflow = state.get("workflow_state") or state.get("state")
                    mapping = workflow in {
                        "starting_exploration", "exploring", "completion_pending",
                        "saving_map", "map_ready",
                    }
                    selected = state.get("patrol_robots") or []
                    localising = workflow in {"awaiting_localisation", "ready_to_patrol"}
                    cleanup = workflow in {
                        "preparing_localisation", "awaiting_localisation",
                        "ready_to_patrol", "patrolling", "error",
                    }
                    return (
                        _mission_summary(state),
                        gr.update(interactive=workflow in {"idle", "map_ready"}),
                        gr.update(interactive=mapping),
                        gr.update(interactive=workflow in {
                            "starting_exploration", "exploring", "completion_pending"}),
                        gr.update(interactive=workflow in {
                            "idle", "map_ready", "awaiting_localisation", "ready_to_patrol"}),
                        gr.update(interactive=cleanup),
                        gr.update(interactive=localising and "luna" in selected),
                        gr.update(interactive=localising and "astro" in selected),
                        gr.update(interactive=localising),
                    )

                def on_refresh_maps(current):
                    if exhibition_control is None:
                        return gr.update(choices=[], value=None), _control_missing()
                    try:
                        maps = exhibition_control.list_maps()
                    except Exception as exc:
                        return gr.update(choices=[], value=None), {
                            "state": "error", "error": str(exc)}
                    valid = [item for item in maps if item.get("valid")][:5]
                    choices = [
                        (item.get("id") or item["yaml"].rsplit("/", 1)[-1],
                         item["yaml"].rsplit("/", 1)[-1])
                        for item in valid
                    ]
                    values = [value for _, value in choices]
                    selected = current if current in values else (values[0] if values else None)
                    return gr.update(choices=choices, value=selected), {
                        "state": "maps", "count": len(choices)}

                def on_prepare(map_name, team):
                    if not map_name:
                        return {"state": "error", "error": "select a completed map"}
                    return _run_named("prepare_localisation", map_name, team)

                def on_confirm(robot):
                    return _run_named("confirm_localisation", robot)

                map_img.select(
                    on_map_select, inputs=init_toggle, outputs=map_interaction_out)

                # Reveal / hide the "Set Initial Pose" option. Hiding also forces
                # the checkbox off so the hidden toggle can't leave the drag
                # in initial-pose mode while the option is invisible.
                def toggle_initpose_section(visible):
                    new_visible = not visible
                    label = ("Hide Initial Pose Setting" if new_visible
                             else "Set Initial Pose")
                    if new_visible:
                        return new_visible, gr.update(visible=True), gr.update(value=label), gr.skip()
                    return new_visible, gr.update(visible=False), gr.update(value=label), gr.update(value=False)

                initpose_btn.click(
                    toggle_initpose_section,
                    inputs=[initpose_visible],
                    outputs=[initpose_visible, initpose_section, initpose_btn, init_toggle],
                )
                # Exposed as a public Gradio API endpoint ("drag_initial") so the browser
                # drag JS can POST the payload directly (POST /gradio_api/call/drag_initial
                # with {"data": [payload, mode]}). This deterministic bridge works
                # regardless of Gradio's component-event semantics. The endpoint is bound to
                # init_bridge.input so it also works on the legacy event path.
                init_bridge.input(on_drag_payload,
                                  inputs=[init_bridge, init_toggle],
                                  outputs=init_status, api_name="drag_initial")

                robot_dd.change(node.set_robot, robot_dd, None)
                mapping_start_btn.click(
                    lambda robot: _run_named("start_mapping", robot),
                    inputs=robot_dd, outputs=action_result_out,
                    api_name="start_mapping")
                mapping_stop_btn.click(
                    lambda robot: _run_named("stop_mapping", robot, False),
                    inputs=robot_dd, outputs=action_result_out,
                    api_name="stop_mapping")
                mapping_partial_btn.click(
                    lambda robot: _run_named("stop_mapping", robot, True),
                    inputs=robot_dd, outputs=action_result_out,
                    api_name="stop_mapping_partial")
                localisation_prepare_btn.click(
                    on_prepare, inputs=[map_dd, team_dd], outputs=action_result_out,
                    api_name="prepare_localisation")
                cleanup_btn.click(
                    lambda: _run_named("stop_patrol"),
                    outputs=action_result_out, api_name="stop_patrol")
                confirm_luna_btn.click(
                    lambda: on_confirm("luna"), outputs=action_result_out,
                    api_name="confirm_localisation_luna")
                confirm_astro_btn.click(
                    lambda: on_confirm("astro"), outputs=action_result_out,
                    api_name="confirm_localisation_astro")
                map_refresh_btn.click(
                    on_refresh_maps, inputs=map_dd,
                    outputs=[map_dd, action_result_out],
                    api_name="refresh_maps")
                demo.load(
                    on_refresh_maps, inputs=map_dd,
                    outputs=[map_dd, action_result_out],
                    api_name="load_maps")
                mission_timer = gr.Timer(3.0)
                mission_timer.tick(
                    on_mission_tick,
                    outputs=[mission_status_out, mapping_start_btn, mapping_stop_btn,
                             mapping_partial_btn, localisation_prepare_btn,
                             cleanup_btn, confirm_luna_btn, confirm_astro_btn,
                             initpose_btn],
                    api_name="mission_status")

            # ================================================================
            # TAB 2: FLEET VIEW (both robots on one map)
            # ================================================================
            with gr.Tab("Fleet View", id="patrolling"):
                # Read-only fleet overview. Motion is owned by the exhibition
                # controller and polygon patrol planner, never a map click.
                with gr.Row():
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        gr.Markdown("#### EXHIBITION MODE")
                        gr.Markdown(
                            "Fleet map interaction is read-only. Use the Mapping "
                            "tab for initial pose and the Patrolling tab for motion.")

                    # CENTER: Fleet Patrolling Map (both robots)
                    with gr.Column(scale=2, elem_classes=["map-panel"]):
                        gr.HTML('<h2 style="text-align:center;margin:0 0 8px;">Fleet Patrolling Map</h2>')
                        patrol_map_img = gr.Image(
                            show_label=False, type="numpy", elem_id="patrol_map_image", height=520)
                        gr.HTML('<div style="text-align:center;">🔴 Luna · 🟦 Astro</div>')

                    # RIGHT: Fleet Camera — streams from BOTH robots
                    with gr.Column(scale=1, elem_classes=["camera-panel"]):
                        gr.HTML('<h2 style="text-align:center;margin:0 0 8px;">Fleet Camera</h2>')
                        patrol_cam_luna = gr.Image(
                            label="Luna — Forward Camera", type="numpy", height=250)
                        patrol_cam_astro = gr.Image(
                            label="Astro — Forward Camera", type="numpy", height=250)

                # ---------- SECTION 2: FLEET TELEMETRY (BOTH ROBOTS) ----------
                with gr.Row():
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        patrol_conn_luna = gr.Textbox(
                            label="Luna — Connection Status", lines=2, interactive=False)
                        patrol_pose_luna = gr.Textbox(
                            label="Luna — 📍 Pose", lines=2, interactive=False)
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        patrol_conn_astro = gr.Textbox(
                            label="Astro — Connection Status", lines=2, interactive=False)
                        patrol_pose_astro = gr.Textbox(
                            label="Astro — 📍 Pose", lines=2, interactive=False)

                # ---------- TICKERS (run continuously; cheap when tab hidden) ----------
                patrol_map_timer = gr.Timer(0.5)
                patrol_map_timer.tick(lambda: node.draw_patrol_map(),
                                      outputs=patrol_map_img)

                patrol_cam_timer = gr.Timer(0.1)
                patrol_cam_timer.tick(lambda: node.patrol_draw_cameras(),
                                      outputs=[patrol_cam_luna, patrol_cam_astro])

                patrol_status_timer = gr.Timer(1.0)
                patrol_status_timer.tick(
                    lambda: node.patrol_get_status(),
                    outputs=[patrol_conn_luna, patrol_pose_luna,
                             patrol_conn_astro, patrol_pose_astro],
                )

            # ================================================================
            # TAB 3: UI PATOL (embedded polygon-constrained patrol planner)
            # The patrol planner runs in the same robot_hivemind container on
            # port 8766 (host networking). The dashboard embeds it in an
            # iframe; the port is forwarded by the SSH tunnel just like 7860.
            # ================================================================
            with gr.Tab("Patrolling", id="patrol-ui"):
                gr.Markdown("#### POLYGON-CONSTRAINED PATROL PLANNER")
                gr.Markdown(
                    "Prepare localisation and confirm every selected robot first. "
                    "Draw the patrol region below, then use the explicit motion gate."
                )
                with gr.Row():
                    patrol_start_btn = gr.Button(
                        "Start Patrol", variant="primary", interactive=False)
                    patrol_stop_btn = gr.Button(
                        "End Patrol & Clean Up", variant="stop", interactive=False)
                patrol_workflow_out = gr.Textbox(
                    label="Patrol Readiness", lines=9, interactive=False)
                patrol_action_out = gr.JSON(label="Last Patrol Result")
                patrol_start_btn.click(
                    lambda: _run_named("start_patrol"), outputs=patrol_action_out,
                    api_name="start_patrol")
                patrol_stop_btn.click(
                    lambda: _run_named("stop_patrol"), outputs=patrol_action_out,
                    api_name="end_patrol")

                def on_patrol_tick():
                    state = _run_named("mission_status")
                    selected = state.get("patrol_robots") or []
                    localisation = state.get("localisation") or {}
                    all_confirmed = bool(selected) and all(
                        (localisation.get(name) or {}).get("confirmed") is True
                        for name in selected
                    )
                    can_start = (
                        state.get("workflow_state") == "ready_to_patrol"
                        and all_confirmed
                    )
                    can_stop = state.get("workflow_state") in {
                        "preparing_localisation", "awaiting_localisation",
                        "ready_to_patrol", "patrolling", "error",
                    }
                    return (
                        _mission_summary(state),
                        gr.update(interactive=can_start),
                        gr.update(interactive=can_stop),
                    )

                patrol_control_timer = gr.Timer(3.0)
                patrol_control_timer.tick(
                    on_patrol_tick,
                    outputs=[patrol_workflow_out, patrol_start_btn, patrol_stop_btn])
                gr.HTML(
                    '<iframe src="http://localhost:8766/" '
                    'style="width:100%; height:900px; border:0; '
                    'border-radius:8px; background:#0b0b12;"></iframe>'
                )

            # ================================================================
            # TAB 4: FOXGLOVE VIEW (3D lidar map, orbit/pan/zoom)
            # A separate viewer process (web_backend/foxglove_viewer.py) serves
            # a Three.js scene plus a WebSocket point-cloud feed on port 8767
            # (host networking, same container). The dashboard only embeds it
            # in an iframe, like the patrol planner on 8766, and the port is
            # forwarded by the SSH tunnel like the others.
            #
            # Deliberately a separate process: the portal is on a boot.sh
            # watchdog because it dies on its own often enough to matter, and a
            # crash in a WebGL/WebSocket path must never take the working map
            # and camera pages down with it. If the viewer is down the iframe
            # just says so and every other tab is unaffected.
            # ================================================================
            with gr.Tab("3D Map", id="foxglove-ui"):
                # The intro heading and blurb were removed at the operator's
                # request. The tab label above is the only remaining label for
                # this view.
                foxglove_robot_dd = gr.Dropdown(
                    choices=list(ROBOTS.keys()),
                    value="Luna",
                    label="Robot",
                    interactive=True,
                    elem_classes=["foxglove-robot"],
                )
                foxglove_view = gr.HTML(
                    value='<iframe id="foxglove_frame" '
                          'src="http://localhost:8767/?robot=Luna" '
                          'style="width:100%; height:820px; border:0; '
                          'border-radius:8px; background:#0b0b12;"></iframe>'
                )
                # The reciprocal hop, so "Switch to 3D" is not a one-way door.
                # Between the view and the status strip: the strip is the last
                # element on the page, so anchoring the button against its top
                # edge keeps both visible without scrolling, even though the
                # iframe alone is taller than a laptop viewport.
                switch_to_2d_btn = gr.Button(
                    "Switch to 2D", variant="secondary",
                    elem_classes=["switch-view-btn"])
                switch_to_2d_btn.click(
                    _goto_tab("mapping"),
                    inputs=None, outputs=page_tabs)

                # Declared after the iframe so the strip renders below the view.
                # It describes what is inside the frame, which makes it read as a
                # caption for the viewer rather than as part of the controls.
                foxglove_status = gr.Markdown(
                    value=_foxglove_status_html("starting"),
                    elem_classes=["foxglove-status"],
                )

                def _on_foxglove_robot(name):
                    # Reload the iframe rather than rebuild it: the viewer owns
                    # the WebSocket, and a src change restarts it cleanly.
                    return (
                        f'<iframe id="foxglove_frame" '
                        f'src="http://localhost:8767/?robot={name}" '
                        f'style="width:100%; height:820px; border:0; '
                        f'border-radius:8px; background:#0b0b12;"></iframe>',
                        _foxglove_status_html(f"switched to {name}"),
                    )

                foxglove_robot_dd.change(
                    _on_foxglove_robot, inputs=foxglove_robot_dd,
                    outputs=[foxglove_view, foxglove_status],
                )

                foxglove_status_timer = gr.Timer(2.0)
                foxglove_status_timer.tick(
                    _foxglove_probe,
                    inputs=[foxglove_robot_dd],
                    outputs=foxglove_status,
                )

    demo.launch(
        server_name=SERV_NAME,
        server_port=SERV_PORT,
    )


if __name__ == "__main__":
    main()
