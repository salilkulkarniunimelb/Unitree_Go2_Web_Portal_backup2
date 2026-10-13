#!/bin/bash
# ---------------------------------------------------------------------------
# deploy_lab_portal.sh - One-command deploys of the QOD Lab Go2 map portal.
#
# Run this FROM YOUR LAPTOP to push an updated dashboard to the server. It:
#   1. Copies lab_portal.py (+ controller scripts) into the robot_hivemind_portal
#      container
#   2. Ensures gradio/numpy/opencv are present (no-op once installed)
#   3. Restarts the portal on port 7860
#
# The robot_hivemind_portal container is created by restore_dashboard.sh and
# auto-starts the portal on boot (boot.sh entrypoint), so the dashboard comes
# back on its own after a server reboot.
#
# Usage:
#   bash deploy_lab_portal.sh                 # uses defaults below
#   SERVER=user@host bash deploy_lab_portal.sh
#
# Prereqs: SSH key auth to the server already configured (we did this earlier).
# ---------------------------------------------------------------------------
set -e

SERVER="${SERVER:-salil.kulkarni@10.4.48.11}"
HOST="${SERVER#*@}"
# The portal container. Deliberately NOT "robot_hivemind": that name is used
# for a compute container (hardware_code bind-mounted, ROS workspace sourced)
# and the two must not collide -- sharing a name means whichever was created
# last silently wins, and the portal then copies its files into a container
# whose entrypoint is a bare shell and never serves 7860.
CONTAINER="${CONTAINER:-robot_hivemind_portal}"
PORT=7860
FILES="lab_portal.py lab_start.sh start_dashboard.sh boot.sh exhibition_control.py"
CONTROL_SSH="${CONTROL_SSH:-selini.samaranayake@10.4.48.11}"

# The "Foxglove view" tab's 3D viewer: a side-car module plus the Three.js
# page and the vendored library it loads. Copied separately because they live in
# subdirectories that docker cp needs as directory targets.
VIEWER_PATHS="web_backend/foxglove_viewer.py web_frontend/foxglove_view.html web_frontend/vendor/three"

echo "============================================================"
echo " Deploying QOD Lab Go2 map portal -> $SERVER"
echo "============================================================"

if [ ! -f lab_portal.py ]; then
    echo "ERROR: lab_portal.py not found in current dir. Run from the repo root."
    exit 1
fi

echo "[1/6] Checking container $CONTAINER exists..."
if ! ssh -o BatchMode=yes "$SERVER" "docker inspect $CONTAINER >/dev/null 2>&1"; then
    echo "ERROR: container '$CONTAINER' does not exist on the server."
    echo "Create it once with:  bash restore_dashboard.sh"
    exit 1
fi

echo "[2/6] Copying dashboard files into container..."
scp -o BatchMode=yes $FILES "$SERVER:/tmp/"
ssh -o BatchMode=yes "$SERVER" "for f in $FILES; do docker cp /tmp/\$f $CONTAINER:/workspace/\$f; done"
ssh -o BatchMode=yes "$SERVER" "docker exec $CONTAINER chmod +x /workspace/lab_start.sh /workspace/start_dashboard.sh /workspace/boot.sh"

echo "[3/6] Ensuring Python deps present (installs only if missing)..."
# websockets is what the 3D viewer's WebSocket feed is built on; it is already
# in the image, but the check keeps a rebuilt image from silently losing the tab.
ssh -o BatchMode=yes "$SERVER" "docker exec $CONTAINER bash -lc 'python3 -c \"import gradio,numpy,cv2,websockets\" 2>/dev/null || python3 -m pip install --no-cache-dir \"numpy<2\" \"opencv-python-headless<5\" gradio websockets'"

echo "[4/6] Copying 3D viewer files (Foxglove view tab)..."
# The viewer is a Python module under web_backend/ plus a static page and a
# vendored copy of three.js under web_frontend/. Shipped as one tar so the
# directory structure survives the trip without needing docker cp to create
# intermediate parents on the far side.
VIEWER_TAR=/tmp/foxglove_viewer_deploy.tar
# macOS bsdtar stores xattrs and AppleDouble junk by default, which both warns
# on extraction inside the container and litters it with ._* files.
COPYFILE_DISABLE=1 tar --no-xattrs -cf "$VIEWER_TAR" \
    --exclude '._*' --exclude '.DS_Store' $VIEWER_PATHS
scp -o BatchMode=yes "$VIEWER_TAR" "$SERVER:/tmp/foxglove_viewer_deploy.tar"
ssh -o BatchMode=yes "$SERVER" "\
    docker exec $CONTAINER mkdir -p /workspace/web_backend /workspace/web_frontend && \
    docker cp /tmp/foxglove_viewer_deploy.tar $CONTAINER:/tmp/ && \
    docker exec $CONTAINER bash -lc 'cd /workspace && tar -xf /tmp/foxglove_viewer_deploy.tar' && \
    docker exec $CONTAINER bash -lc 'find /workspace/web_backend /workspace/web_frontend -name \"._*\" -delete' && \
    docker exec $CONTAINER python3 -c 'import py_compile,sys; py_compile.compile(\"/workspace/web_backend/foxglove_viewer.py\", doraise=True)' && \
    docker exec $CONTAINER rm -f /tmp/foxglove_viewer_deploy.tar"
rm -f "$VIEWER_TAR"

echo "[5/6] Deploying the restricted exhibition-control boundary..."
HOST_WRAPPER='$HOME/.config/lab-portal-explorer/explorer_host.sh'
ssh -o BatchMode=yes "$CONTROL_SSH" \
    "mkdir -p \$(dirname $HOST_WRAPPER) && chmod 700 \$(dirname $HOST_WRAPPER) && cat > $HOST_WRAPPER && chmod +x $HOST_WRAPPER" \
    < scripts/dashboard/explorer_host.sh
ssh -o BatchMode=yes "$CONTROL_SSH" \
    "cat > \$HOME/install_explorer_key.sh && bash \$HOME/install_explorer_key.sh" \
    < scripts/dashboard/install_explorer_key.sh

echo "[6/6] Restarting portal in container..."
# Restart the container rather than just the portal process: boot.sh is copied
# in at [2/6], after the entrypoint already ran, so the running PID 1 is still
# whatever was baked into the image. Restarting makes the container re-exec the
# boot.sh we just deployed, so its watchdog is the one actually supervising the
# portal (otherwise the portal dies and nothing restarts it).
ssh -o BatchMode=yes "$SERVER" "docker restart $CONTAINER" >/dev/null

echo ""
echo "DONE. Portal updated and restarted on the server."
echo "Anyone on the network can open  http://$HOST:$PORT"
exit 0
