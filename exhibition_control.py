"""Restricted client for the hardware_code exhibition controller.

The portal container deliberately has neither the hardware repository nor the
Docker socket. Every operation crosses the existing forced-command SSH key and
is executed by the server-host wrapper. Mission ownership remains in
hardware_code's ``exhibitionctl.py``.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading


SSH_TARGET = os.getenv("EXHIBITION_SSH_TARGET", "selini.samaranayake@127.0.0.1")
SSH_KEY = os.getenv("EXHIBITION_SSH_KEY", "/root/.ssh/explorer_key")
SSH_KNOWN_HOSTS = os.getenv(
    "EXHIBITION_SSH_KNOWN_HOSTS", "/root/.ssh/known_hosts"
)
SSH_OPTS = (
    "-i", SSH_KEY,
    "-o", "IdentitiesOnly=yes",
    "-o", f"UserKnownHostsFile={SSH_KNOWN_HOSTS}",
    "-o", "StrictHostKeyChecking=yes",
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=10",
    "-o", "ServerAliveInterval=15",
    "-o", "ServerAliveCountMax=4",
)

ROBOTS = ("luna", "astro")
TEAMS = ("luna", "astro", "both")
ROBOT_LABELS = {
    "Luna": "luna", "Astro": "astro", "luna": "luna", "astro": "astro"
}
MAP_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*_completed_map\.yaml$")
READ_TIMEOUT = 35.0
TRANSITION_TIMEOUT = 180.0
_transition_lock = threading.Lock()


class ExhibitionControlError(RuntimeError):
    """A validated exhibition request failed on the server."""


def _robot(value: str) -> str:
    try:
        return ROBOT_LABELS[value]
    except (KeyError, TypeError) as exc:
        raise ValueError("robot must be Luna or Astro") from exc


def _team(value: str) -> str:
    normalised = str(value or "").strip().lower()
    if normalised not in TEAMS:
        raise ValueError("robot team must be luna, astro, or both")
    return normalised


def _map_name(value: str) -> str:
    name = Path(str(value or "")).name
    if name != value or not MAP_NAME_RE.fullmatch(name) or ".." in name:
        raise ValueError("select a valid completed-map YAML")
    return name


def _number(value: float) -> str:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("pose values must be finite numbers")
    return format(number, ".12g")


def _ssh_argv(request: str) -> list[str]:
    return ["ssh", "-T", *SSH_OPTS, SSH_TARGET, request]


def _call(parts: list[str], timeout: float = READ_TIMEOUT) -> dict | list:
    if not os.path.isfile(SSH_KEY):
        raise ExhibitionControlError(
            f"restricted dashboard key is missing at {SSH_KEY}; redeploy the portal key"
        )
    try:
        result = subprocess.run(
            _ssh_argv(" ".join(parts)),
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        raise ExhibitionControlError(
            f"{parts[0]} did not finish within {timeout:.0f}s"
        ) from exc
    except OSError as exc:
        raise ExhibitionControlError(f"could not run restricted SSH client: {exc}") from exc

    output = (result.stdout or "").strip()
    error = (result.stderr or "").strip()
    candidate = output if result.returncode == 0 else (error or output)
    try:
        payload = json.loads(candidate)
    except json.JSONDecodeError:
        detail = candidate[-2000:] or f"exit {result.returncode} with no output"
        raise ExhibitionControlError(f"{parts[0]} returned an invalid response: {detail}")
    if result.returncode:
        detail = payload.get("error") if isinstance(payload, dict) else str(payload)
        raise ExhibitionControlError(str(detail or payload))
    return payload


def _transition(parts: list[str]) -> dict:
    if not _transition_lock.acquire(blocking=False):
        raise ExhibitionControlError(
            "another mission transition is running; wait for it to finish or use cleanup"
        )
    try:
        payload = _call(parts, TRANSITION_TIMEOUT)
        if not isinstance(payload, dict):
            raise ExhibitionControlError(f"{parts[0]} did not return a workflow state")
        return payload
    finally:
        _transition_lock.release()


def mission_status() -> dict:
    payload = _call(["mission-status"])
    if not isinstance(payload, dict):
        raise ExhibitionControlError("mission-status did not return an object")
    return payload


def robot_status(robot: str) -> dict:
    payload = _call(["robot-status", _robot(robot)])
    if not isinstance(payload, dict):
        raise ExhibitionControlError("robot-status did not return an object")
    return payload


def list_maps() -> list[dict]:
    payload = _call(["list-maps"])
    if not isinstance(payload, list):
        raise ExhibitionControlError("list-maps did not return a list")
    return payload


def start_mapping(robot: str) -> dict:
    return _transition(["start-mapping", _robot(robot)])


def stop_mapping(robot: str, save_partial: bool = False) -> dict:
    action = "stop-mapping-partial" if save_partial else "stop-mapping"
    payload = _call([action, _robot(robot)], TRANSITION_TIMEOUT)
    if not isinstance(payload, dict):
        raise ExhibitionControlError("stop-mapping did not return a workflow state")
    return payload


def prepare_localisation(map_name: str, team: str) -> dict:
    return _transition(["prepare-localisation", _map_name(map_name), _team(team)])


def set_initial_pose(robot: str, x: float, y: float, yaw: float) -> dict:
    return _transition([
        "set-initial-pose", _robot(robot), _number(x), _number(y), _number(yaw)
    ])


def confirm_localisation(robot: str) -> dict:
    return _transition(["confirm-localisation", _robot(robot)])


def start_patrol() -> dict:
    state = mission_status()
    selected = state.get("patrol_robots")
    localisation = state.get("localisation") or {}
    ready = (
        state.get("workflow_state") == "ready_to_patrol"
        and isinstance(selected, list)
        and bool(selected)
        and all(
            name in ROBOTS
            and isinstance(localisation.get(name), dict)
            and localisation[name].get("confirmed") is True
            for name in selected
        )
    )
    if not ready:
        raise ExhibitionControlError(
            "patrol gate closed: confirm localisation for every selected robot "
            "after the latest pose submission"
        )
    return _transition(["start-patrol"])


def stop_patrol() -> dict:
    payload = _call(["stop-patrol"], TRANSITION_TIMEOUT)
    if not isinstance(payload, dict):
        raise ExhibitionControlError("stop-patrol did not return a workflow state")
    return payload
