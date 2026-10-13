#!/bin/bash
# ---------------------------------------------------------------------------
# restore_dashboard.sh - ONE-COMMAND restore of the QOD Lab Go2 map dashboard.
#
# Run from YOUR LAPTOP. Fully self-contained: recreates the robot_hivemind
# container if missing (from the snapshot image, which has the code, Python
# deps and the auto-boot entrypoint baked in), syncs the latest files,
# verifies deps/ffmpeg, starts the portal, opens the SSH tunnel, and opens
# the dashboard in your browser.
#
# The container runs with --restart unless-stopped and its entrypoint
# (boot.sh) auto-starts the portal on every server boot / docker restart, so
# the dashboard comes back on its own even if the machine reboots.
#
# Usage:
#   bash restore_dashboard.sh
#
# After breaking lab_portal.py:  git checkout HEAD -- lab_portal.py && bash restore_dashboard.sh
#
# NOTE: this script copies lab_portal.py but NOT the 3D viewer bundle (the
# "3D Map" tab). It is therefore safe to use ONLY for the portal itself: it
# leaves web_backend/foxglove_viewer.py and web_frontend/ alone, so it cannot
# overwrite them. If the container is RECREATED from the image, however, the
# viewer comes back at whatever the image has baked in, not at your working
# copy -- for that case run deploy_lab_portal.sh, which ships the viewer too.
# ---------------------------------------------------------------------------
set -e

SERVER="${SERVER:-salil.kulkarni@10.4.48.11}"
HOST="${SERVER#*@}"
# The portal container. Deliberately NOT "robot_hivemind", which is a compute
# container (see scripts/compute/setup_compute_container.sh). When both used
# the same name this script's "recreate only if inspect fails" logic silently
# accepted the wrong container and the portal never came up.
CONTAINER="${CONTAINER:-robot_hivemind_portal}"
IMAGE="${IMAGE:-unimelb-humble:dashboard}"   # snapshot: code + deps + boot.sh baked in
ENTRYPOINT="bash /workspace/boot.sh"
PORT=7860
# Must match the explorer container and zenoh_ros2dds_server. Without it the
# portal lands on the default domain 0, cannot discover ANY robot topic, and
# silently shows no map, no pose, no plan and no odom while the camera (which
# is WebRTC, not DDS) keeps working -- so the dashboard looks half-alive.
ROS_DOMAIN="${ROS_DOMAIN:-70}"
FILES="lab_portal.py lab_start.sh start_dashboard.sh boot.sh exhibition_control.py"
WEB_BACKEND_FILES="web_backend/qod_consumer.py web_backend/qod_detections.py"
# Host-side (not container-side) helper that sets up the restricted SSH key the
# explorer button uses. It is NOT copied into the container -- it runs on the
# host as the robot owner.
HOST_SETUP_FILES="scripts/dashboard/install_explorer_key.sh"
# Deliberately NOT $SERVER. The forced-command key has to be installed into the
# robot owner's authorized_keys, and explorer_host.sh has to exist in that
# user's repo, so this runs as the owner regardless of who is deploying.
EXPLORER_SSH="${EXPLORER_SSH:-selini.samaranayake@10.4.48.11}"

SSH="ssh -o BatchMode=yes $SERVER"
SCP="scp -o BatchMode=yes"

echo "============================================================"
echo " Restoring QOD Lab Go2 map dashboard -> $SERVER"
echo "============================================================"

# --- 1) Make sure the portal files exist locally ---------------------------
for f in $FILES; do
    [ -f "$f" ] || { echo "ERROR: $f missing. Run from the repo root, or restore it: git checkout HEAD -- $f"; exit 1; }
done
for f in $HOST_SETUP_FILES; do
    [ -f "$f" ] || { echo "WARNING: $f missing -- the Start Exploring button will not work."; }
done

# --- 2) Container must exist ------------------------------------------------
echo "[1/7] Checking container $CONTAINER..."
# Verify the container is actually the PORTAL and not something that merely
# borrowed the name. Without this, a container built from a different image
# passes `docker inspect`, the script copies lab_portal.py into it, restarts it,
# and then fails much later with a bare "never answered on port 7860" -- which
# says nothing about the real cause. Check the image, because that is what
# decides whether boot.sh + the portal deps are even present.
EXISTING_IMAGE="$($SSH "docker inspect -f '{{.Config.Image}}' $CONTAINER 2>/dev/null" || true)"
if [ -n "$EXISTING_IMAGE" ] && [ "$EXISTING_IMAGE" != "$IMAGE" ]; then
    echo "ERROR: container $CONTAINER exists but was built from '$EXISTING_IMAGE',"
    echo "       not '$IMAGE'. It is not a portal container, so copying files"
    echo "       into it cannot make it serve port $PORT."
    echo ""
    echo "       If this name is taken by a compute container, use a different one:"
    echo "         CONTAINER=robot_hivemind_portal bash restore_dashboard.sh"
    echo "       Or remove the stale container first:"
    echo "         $SSH \"docker rm -f $CONTAINER\""
    exit 1
fi

if ! $SSH "docker inspect $CONTAINER >/dev/null 2>&1"; then
    echo "  -> Container missing. Recreating from $IMAGE (host networking, auto-boot)..."
    $SSH "docker run -d --name $CONTAINER --network host --restart unless-stopped -e ROS_DOMAIN_ID=$ROS_DOMAIN $IMAGE $ENTRYPOINT"
    echo "  -> Container recreated. It will be restarted at [6/7] to pick up the copied boot.sh."
    # The 3D Map tab lives in the viewer bundle, which this script does NOT
    # copy. A fresh container therefore starts with whatever the IMAGE has
    # baked in, so say so rather than letting the tab silently regress.
    echo "  -> WARNING: this container was rebuilt from the image, so the '3D Map'"
    echo "     tab may be an OLD version. After this finishes, run:"
    echo "       bash deploy_lab_portal.sh"
    echo "     to ship the current viewer (foxglove_viewer.py + foxglove_view.html)."
else
    if ! $SSH "docker inspect -f '{{.State.Running}}' $CONTAINER" | grep -q true; then
        echo "  -> Container exists but stopped. Starting it..."
        $SSH "docker start $CONTAINER"
    fi
fi

# --- 3) Copy portal files into /workspace ----------------------------------
echo "[2/7] Copying portal files into container..."
$SCP $FILES "$SERVER:/tmp/"
for f in $FILES; do
    $SSH "docker cp /tmp/$f $CONTAINER:/workspace/$f"
done
$SSH "docker exec $CONTAINER chmod +x /workspace/lab_start.sh /workspace/start_dashboard.sh /workspace/boot.sh"
# The QOD WebRTC camera consumer + detection-subscriber modules (used by
# lab_portal.py).
$SSH "docker exec $CONTAINER mkdir -p /workspace/web_backend"
for wf in $WEB_BACKEND_FILES; do
    if [ -f "$wf" ]; then
        bn=$(basename "$wf")
        echo "  -> Copying $wf into container..."
        $SCP "$wf" "$SERVER:/tmp/$bn"
        $SSH "docker cp /tmp/$bn $CONTAINER:/workspace/web_backend/$bn"
    fi
done
# The dashboard logo and any assets are served from the container, so copy
# the whole assets/ folder (missing logo caused a FileNotFoundError crash).
if [ -d assets ]; then
    echo "  -> Copying assets/ into container..."
    if ! $SSH "docker exec $CONTAINER ls /workspace/assets >/dev/null 2>&1"; then
        $SSH "docker exec $CONTAINER mkdir -p /workspace/assets"
    fi
    for f in assets/*; do
        [ -f "$f" ] || continue
        bn=$(basename "$f")
        $SCP "$f" "$SERVER:/tmp/$bn"
        $SSH "docker cp /tmp/$bn $CONTAINER:/workspace/assets/$bn"
    done
fi

# --- 4) Ensure Python deps --------------------------------------------------
echo "[3/7] Ensuring Python deps (installs only if missing)..."
# NOTE: numpy is pinned to <2 because the ROS cv_bridge package was compiled
# against NumPy 1.x and crashes with NumPy 2.x (_ARRAY_API not found), and
# opencv is pinned to <5 to stay numpy<2 compatible.
$SSH "docker exec $CONTAINER bash -lc '
    if ! python3 -c \"import gradio,numpy,cv2\" 2>/dev/null; then
        if ! python3 -m pip --version >/dev/null 2>&1; then
            apt-get update -qq && apt-get install -y -qq python3-pip
        fi
        python3 -m pip install --no-cache-dir \"numpy<2\" \"opencv-python-headless<5\" gradio
    elif python3 -c \"import numpy,sys; sys.exit(0 if numpy.__version__.startswith(chr(49)) else 1)\" 2>/dev/null; then
        echo deps-ok
    else
        echo \"  -> numpy >= 2 detected, downgrading to <2 for cv_bridge compat...\"
        python3 -m pip install --no-cache-dir \"numpy<2\" \"opencv-python-headless<5\"
    fi
'"

# --- 5) Ensure ffmpeg (for the live H.264 camera pipeline) -----------------
echo "[4/7] Ensuring ffmpeg (installs only if missing)..."
$SSH "docker exec $CONTAINER bash -lc '
    if ! command -v ffmpeg >/dev/null 2>&1; then
        echo \"  -> installing ffmpeg...\"
        apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends ffmpeg
    else
        echo ffmpeg-ok
    fi
'"

# --- 5) Ensure QOD WebRTC consumer deps (aiortc, websockets, redis) --------------
echo "[5/7] Ensuring QOD WebRTC camera deps (aiortc, websockets, redis)..."
$SSH "docker exec $CONTAINER bash -lc '
    if ! python3 -c \"import aiortc,websockets,redis\" 2>/dev/null || ! python3 -c \"import websockets;assert int(websockets.__version__.split(chr(46))[0])>=10\" 2>/dev/null; then
        echo \"  -> installing/fixing aiortc + websockets + redis (QOD camera/detection consumers)...\"
        python3 -m pip install --no-cache-dir \"aiortc>=1.15.0\" \"websockets>=10,<17\" redis
    else
        echo webrtc-deps-ok
    fi
'"

# --- 6) Start the portal ------------------------------------------------------
echo "[6/7] Starting the portal..."
# ALWAYS restart the container, never rely on the entrypoint having already
# booted the portal. The dashboard files (boot.sh included) are copied in at
# step [2/7], which is AFTER the container's entrypoint has run, so on a fresh
# container PID 1 is still the `sleep infinity` from the boot.sh baked into the
# IMAGE and the freshly-copied watchdog never takes effect. Restarting makes
# the container re-exec the /workspace/boot.sh we just deployed, so the
# supervisor is the one actually running.
$SSH "docker restart $CONTAINER" >/dev/null
echo "  -> Container restarted (runs the boot.sh we just copied in)."

# --- 6b) Explorer button's restricted SSH key --------------------------------
# The container was just recreated/restarted, so /root/.ssh is empty and the
# Start Exploring button would be dead without this. Non-fatal on purpose: the
# dashboard is the thing being restored here, and a missing key costs one
# button, not the portal. Loud, though -- a silently broken button is worse.
echo "[6b/7] Installing the explorer button's restricted SSH key (as $EXPLORER_SSH)..."

# 6b-1: deploy the forced-command script itself FIRST.
# authorized_keys pins command=".../explorer_host.sh", so if that file is absent
# every Start Exploring press dies with
#   bash: line 1: .../explorer_host.sh: No such file or directory
# It goes to $HOME/.config, NOT into hardware_code: that repo is a pull target
# on this server, so an untracked file in its tree is destroyed by the next
# `git pull`/`git clean` by anyone working there. Must happen before
# install_explorer_key.sh, which hard-fails if the script is not executable.
if [ -f scripts/dashboard/explorer_host.sh ]; then
    HOST_WRAPPER='$HOME/.config/lab-portal-explorer/explorer_host.sh'
    if WRAP_OUT=$(ssh -o BatchMode=yes "$EXPLORER_SSH" \
                   "mkdir -p \$(dirname $HOST_WRAPPER) && chmod 700 \$(dirname $HOST_WRAPPER) && cat > $HOST_WRAPPER && chmod +x $HOST_WRAPPER && echo deployed" \
                   < scripts/dashboard/explorer_host.sh 2>&1) && \
       echo "$WRAP_OUT" | grep -q deployed; then
        echo "    [host-wrapper] deployed explorer_host.sh -> $HOST_WRAPPER"
    else
        echo "$WRAP_OUT" | sed 's/^/    [host-wrapper] /'
        echo "    [host-wrapper] WARNING: could not deploy explorer_host.sh; the Start Exploring button will fail."
    fi
else
    echo "    [host-wrapper] WARNING: scripts/dashboard/explorer_host.sh missing from this repo."
fi

if [ -f scripts/dashboard/install_explorer_key.sh ]; then
    # Deliberately NOT $SCP/$SSH: those already embed $SERVER, so appending
    # $EXPLORER_SSH yields two destinations and ssh runs the second as a remote
    # COMMAND (which "succeeds" while doing nothing). Plain ssh/scp here.
    # Piped through cat rather than scp'd to /tmp because /tmp on this host is
    # shared between accounts and a file owned by the deploy user is not
    # overwritable by the robot owner.
    if KEY_OUT=$(ssh -o BatchMode=yes "$EXPLORER_SSH" \
                   "cat > \$HOME/install_explorer_key.sh && bash \$HOME/install_explorer_key.sh" \
                   < scripts/dashboard/install_explorer_key.sh 2>&1); then
        echo "$KEY_OUT" | sed 's/^/    /'
        echo "  -> Explorer key installed."
    else
        echo "$KEY_OUT" | sed 's/^/    /'
        echo "  -> WARNING: could not install the explorer key."
        echo "     The dashboard is UP, but Start Exploring will report a missing key."
        echo "     Re-run as the robot owner:"
        echo "       ssh $EXPLORER_SSH 'bash ~/install_explorer_key.sh'"
    fi
else
    echo "  -> SKIPPED: scripts/dashboard/install_explorer_key.sh not found."
    echo "     Start Exploring will report a missing key until it is installed."
fi

echo "  -> Waiting for the portal to answer on port $PORT..."
# --fail matters: without it a Gradio 500 still exits 0 and we would report a
# broken portal as UP.
PORTAL_UP=0
for i in $(seq 1 30); do
    if $SSH "curl -fsS --noproxy '*' -o /dev/null -m 5 http://$HOST:$PORT/ 2>/dev/null"; then
        echo "  -> Portal is UP."
        PORTAL_UP=1
        break
    fi
    sleep 2
done
if [ "$PORTAL_UP" != "1" ]; then
    echo ""
    echo "ERROR: the portal never answered on $HOST:$PORT (gave up after 60s)."
    echo "Last 25 log lines from the container:"
    $SSH "docker exec $CONTAINER bash -lc 'tail -25 /workspace/lab_portal.log'" || true
    exit 1
fi

# --- 7) Tunnel + open browser --------------------------------------------------
PATROL_PORT="${PATROL_PORT:-8766}"
# The "Foxglove view" tab's 3D viewer runs as a side-car next to the portal and
# is restarted with it, so -- like 7860 -- it only needs its port forwarded.
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8767}"

# A local port being LISTENing is NOT proof of a working tunnel: an `ssh -N -L`
# orphaned by a VPN drop keeps the port bound while forwarding nowhere, and the
# old `if ! lsof ...` check treated that as "tunnel already up" and skipped it.
# The dashboard then looked dead even though the script printed DONE.
tunnel_is_live() {
    pids=$(lsof -tiTCP:"$1" -sTCP:LISTEN 2>/dev/null || true)
    [ -n "$pids" ] || return 1
    ps -p $pids -o comm= 2>/dev/null | grep -q 'ssh'
}

# Replace whatever squats on the port, then forward it and wait for a real ssh
# listener instead of a fixed sleep.
#   $3 verify: "http"   -> also prove real bytes come through the tunnel
#               "listen" -> a bound ssh listener is enough
#   $4 reclaim: "ours"   -> kill any squatter, this port is ours alone
#                "ssh"   -> only replace a stale ssh tunnel, never touch a
#                            real process (the patrol planner may be running
#                            locally on purpose)
open_tunnel() {
    _port="$1"
    _name="$2"
    _verify_http="$3"
    _reclaim="${4:-ssh}"
    if tunnel_is_live "$_port"; then
        if [ "$_verify_http" != "http" ]; then
            echo "  -> Tunnel for $_name (:$_port) is already live, reusing it."
            return 0
        fi
        # A bound ssh listener can still be forwarding nowhere (orphaned by a
        # VPN drop), so for the dashboard prove it with real bytes before
        # trusting it, and rebuild it if it does not answer.
        if curl -fsS --noproxy '*' -o /dev/null -m 5 "http://localhost:$_port/" 2>/dev/null; then
            echo "  -> Tunnel for $_name (:$_port) is already live and serving, reusing it."
            return 0
        fi
        echo "  -> Existing tunnel on $_port is bound but NOT serving. Rebuilding it..."
        _stale=$(lsof -tiTCP:"$_port" -sTCP:LISTEN 2>/dev/null || true)
        kill $_stale 2>/dev/null || true
        sleep 1
        kill -9 $_stale 2>/dev/null || true
    fi
    _squatters=$(lsof -tiTCP:"$_port" -sTCP:LISTEN 2>/dev/null || true)
    if [ -n "$_squatters" ]; then
        if [ "$_reclaim" = "ours" ]; then
            echo "  -> Port $_port held by a dead listener (pids: $(echo $_squatters | tr '\n' ' ')). Reclaiming it..."
            kill $_squatters 2>/dev/null || true
            sleep 1
            kill -9 $_squatters 2>/dev/null || true
        else
            # Only ssh squatters are ours to remove; anything else could be a
            # process the operator started by hand, so leave it alone.
            _stale=""
            for _p in $_squatters; do
                if ps -p "$_p" -o comm= 2>/dev/null | grep -q 'ssh'; then
                    _stale="$_stale $_p"
                fi
            done
            if [ -n "$_stale" ]; then
                echo "  -> Stale ssh tunnel on $_port (pids:$(echo $_stale)). Reclaiming it..."
                kill $_stale 2>/dev/null || true
                sleep 1
                kill -9 $_stale 2>/dev/null || true
            fi
            if [ -n "$(lsof -tiTCP:"$_port" -sTCP:LISTEN 2>/dev/null || true)" ]; then
                echo "  -> Port $_port is held by a real (non-ssh) process; leaving it alone."
                echo "     Not tunnelling $_name - stop that process first if you want the tunnel."
                return 1
            fi
        fi
    fi
    nohup $SSH -N -L "$_port:localhost:$_port" >/dev/null 2>&1 < /dev/null &
    disown || true
    for i in $(seq 1 15); do
        if tunnel_is_live "$_port"; then
            if [ "$_verify_http" = "http" ]; then
                # End-to-end proof: real bytes through the tunnel.
                for j in $(seq 1 10); do
                    if curl -fsS --noproxy '*' -o /dev/null -m 5 "http://localhost:$_port/" 2>/dev/null; then
                        echo "  -> Tunnel for $_name verified: http://localhost:$_port"
                        return 0
                    fi
                    sleep 1
                done
                echo "  -> ERROR: tunnel for $_name is open but http://localhost:$_port did not respond."
                return 1
            fi
            echo "  -> Tunnel for $_name open: http://localhost:$_port"
            return 0
        fi
        sleep 1
    done
    echo "  -> ERROR: could not establish the tunnel for $_name (:$_port)."
    return 1
}

echo "[7/7] Opening SSH tunnels..."
TUNNEL_OK=0
if open_tunnel "$PORT" "dashboard" http ours; then
    TUNNEL_OK=1
fi
# The patrol planner is embedded as an iframe on the "Patrolling" tab and is
# started separately (by hand, in its own terminal), so all this needs to do is
# forward the port. "ssh" reclaim mode: a real process already on 8766 is left
# untouched, so a planner you started locally is never killed by this script.
if open_tunnel "$PATROL_PORT" "patrol planner" listen ssh; then
    echo "  -> Start the patrol planner separately for the 'Patrolling' tab to render."
fi
if open_tunnel "$FOXGLOVE_PORT" "3D viewer" listen ssh; then
    echo "  -> Port $FOXGLOVE_PORT forwarded; the 'Foxglove view' tab will render."
fi

echo ""
if [ "$TUNNEL_OK" = "1" ]; then
    echo "DONE. Dashboard restored, tunneled and serving the latest code."
    echo "  -> http://localhost:$PORT       (tunneled, this laptop)"
    echo "  -> http://$HOST:$PORT     (anyone on the network)"
    if command -v open >/dev/null 2>&1; then
        open "http://localhost:$PORT"
        echo "  -> Opened http://localhost:$PORT in your browser."
    fi
    exit 0
fi

echo "ERROR: the dashboard is running on the server but the local tunnel failed."
echo "  -> It is still reachable over the network at http://$HOST:$PORT"
echo "  -> Or re-run this script; it reclaims the stale local port automatically."
exit 1
