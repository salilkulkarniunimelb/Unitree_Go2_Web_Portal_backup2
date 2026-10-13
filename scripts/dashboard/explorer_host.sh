#!/bin/bash
# Forced-command bridge from the portal to hardware_code's exhibition CLI.
# The SSH key is pinned to this script with command=... and restrict.

set -f

HARDWARE_ROOT="$HOME/unimelb_project/hardware_code"
CLI="$HARDWARE_ROOT/scripts/exhibition/exhibitionctl.py"
MAPS_DIR="$HARDWARE_ROOT/maps"

fail() {
    printf '{"state":"error","error":"%s"}\n' "$1" >&2
    exit 2
}

[ -f "$CLI" ] || fail "hardware exhibition controller is missing"

REQUEST="${SSH_ORIGINAL_COMMAND:-$*}"
# shellcheck disable=SC2086 -- deliberate split after disabling glob expansion.
set -- $REQUEST
ACTION="${1:-}"

valid_robot() { [ "$1" = "luna" ] || [ "$1" = "astro" ]; }
valid_team() { valid_robot "$1" || [ "$1" = "both" ]; }
valid_number() {
    printf '%s' "$1" | grep -Eq '^-?(([0-9]+([.][0-9]*)?)|([.][0-9]+))([eE][+-]?[0-9]+)?$'
}
valid_map() {
    case "$1" in
        *..*|*/*|.*|*[^A-Za-z0-9._-]*|*_partial_map.yaml) return 1 ;;
        *_completed_map.yaml) [ -f "$MAPS_DIR/$1" ] ;;
        *) return 1 ;;
    esac
}

case "$ACTION" in
    # Compatibility alias used by the restricted-key installer's canary.
    status|mission-status)
        [ "$#" -eq 1 ] || fail "mission-status takes no arguments"
        exec python3 "$CLI" mission-status
        ;;
    robot-status|start-mapping|stop-mapping|stop-mapping-partial|confirm-localisation)
        [ "$#" -eq 2 ] && valid_robot "$2" || fail "$ACTION requires luna or astro"
        if [ "$ACTION" = "stop-mapping-partial" ]; then
            exec python3 "$CLI" stop-mapping "$2" --save-partial
        fi
        exec python3 "$CLI" "$ACTION" "$2"
        ;;
    list-maps)
        [ "$#" -eq 1 ] || fail "list-maps takes no arguments"
        exec python3 "$CLI" list-maps
        ;;
    prepare-localisation)
        [ "$#" -eq 3 ] || fail "prepare-localisation requires map and robot team"
        valid_map "$2" || fail "map must be an existing completed-map YAML"
        valid_team "$3" || fail "robot team must be luna, astro, or both"
        exec python3 "$CLI" prepare-localisation "$MAPS_DIR/$2" --robots "$3"
        ;;
    set-initial-pose)
        [ "$#" -eq 5 ] && valid_robot "$2" || fail "set-initial-pose requires robot x y yaw"
        valid_number "$3" && valid_number "$4" && valid_number "$5" \
            || fail "pose values must be finite decimal numbers"
        exec python3 "$CLI" set-initial-pose "$2" "$3" "$4" "$5"
        ;;
    start-patrol|stop-patrol)
        [ "$#" -eq 1 ] || fail "$ACTION takes no arguments"
        exec python3 "$CLI" "$ACTION"
        ;;
    *)
        fail "unsupported exhibition request"
        ;;
esac
