# Robotic Dogs Exhibition Dashboard

This repository contains the presentation dashboard for Luna and Astro. The
deployed application is `lab_portal.py`; it displays ROS 2 telemetry and sends
mission requests to the authoritative exhibition controller in the separate
`hardware_code` repository.

The old `main.py` application and its LLM/MCP modules are retained as a legacy
portal, but they are not imported or launched by the exhibition dashboard.
They are therefore not part of the live robot-control path.

## Control architecture

```text
Browser buttons
    -> lab_portal.py
    -> exhibition_control.py (validated requests only)
    -> restricted SSH key and forced-command wrapper
    -> hardware_code/scripts/exhibition/exhibitionctl.py
    -> exhibition state machine and robot processes
```

The dashboard does not duplicate mission state or launch robot processes
directly. `hardware_code` remains the single source of truth for mapping,
completed-map handover, localisation, confirmation, patrol dispatch, and
cleanup.

The restricted SSH key cannot open a shell or forward ports. Its host-side
wrapper accepts only the following validated operations:

- mission and robot status
- list completed maps
- start/stop mapping, including an explicitly labelled partial save
- prepare localisation for Luna, Astro, or both
- set and confirm each selected robot's initial pose
- start patrol after all selected robots are confirmed
- stop patrol and clean up localisation

## Operator workflow

1. In **Mapping**, select Luna or Astro and choose **Start Mapping**.
2. Choose **Stop Mapping** for a normal completed-map handover. Use **Stop &
   Save Partial Map** only when you deliberately want a non-handover map.
3. Refresh the completed-map list, choose the map and robot team, then select
   **Prepare Localisation**.
4. Select each robot, enable **Set Initial Pose**, and drag from its position
   toward its heading on the map.
5. Verify localisation visually, then confirm each selected robot separately.
6. In **Patrolling**, draw the patrol region in the embedded planner and press
   **Start Patrol**. The button remains disabled until the controller reports
   every selected robot as confirmed.
7. Press **End Patrol & Clean Up** to cancel motion and stop the owned
   localisation/patrol processes.

## Deploy

From the repository root on the deployment laptop:

```bash
bash deploy_lab_portal.sh
```

The defaults target the existing lab server, portal container, and robot-owner
account. They can be overridden when necessary:

```bash
SERVER=user@server CONTROL_SSH=robot-owner@server \
  CONTAINER=robot_hivemind_portal bash deploy_lab_portal.sh
```

The deploy script copies the portal files, installs the forced-command wrapper
and restricted key, verifies that the key cannot execute arbitrary commands,
and restarts the container.

Use `bash restore_dashboard.sh` when the portal container itself is missing or
needs full dependency repair. It also reinstalls the restricted control key.

## Launch and status

The container's `boot.sh` automatically starts and supervises the portal. The
manual commands are:

```bash
docker exec robot_hivemind_portal bash /workspace/lab_start.sh start
docker exec robot_hivemind_portal bash /workspace/lab_start.sh status
docker exec robot_hivemind_portal bash /workspace/lab_start.sh restart
docker exec robot_hivemind_portal bash /workspace/lab_start.sh stop
```

To restart the entire container and rerun the supervisor:

```bash
docker restart robot_hivemind_portal
```

The dashboard serves on port `7860`; the embedded patrol planner and 3D viewer
use ports `8766` and `8767` respectively.

## Verification

Run repository-local checks without commanding a robot:

```bash
python3 -m py_compile lab_portal.py exhibition_control.py
bash -n scripts/dashboard/explorer_host.sh deploy_lab_portal.sh \
  restore_dashboard.sh scripts/dashboard/install_explorer_key.sh
python3 -m unittest discover -s tests -v
git diff --check
```

After deployment:

```bash
docker exec robot_hivemind_portal bash /workspace/lab_start.sh status
docker exec robot_hivemind_portal tail -n 50 /workspace/lab_portal.log
```

Before allowing motion, use the dashboard status panels or run the read-only
controller inspections in `hardware_code`:

```bash
cd ~/unimelb_project/hardware_code
./scripts/exhibition/exhibitionctl.py mission-status
./scripts/exhibition/exhibitionctl.py robot-status luna
./scripts/exhibition/exhibitionctl.py robot-status astro
./scripts/exhibition/exhibitionctl.py list-maps
```

Perform the first end-to-end button test with the dogs supported or in a clear,
controlled area. Confirm that mapping starts/stops, completed maps populate,
pose submission clears confirmation, both confirmations open the patrol gate,
and cleanup returns the workflow to `idle`.

## Relevant files

- `lab_portal.py` — deployed Gradio UI, telemetry, and button callbacks
- `exhibition_control.py` — validated dashboard-side SSH client
- `scripts/dashboard/explorer_host.sh` — host-side forced-command allowlist
- `scripts/dashboard/install_explorer_key.sh` — restricted-key installer/canary
- `deploy_lab_portal.sh` — normal deployment
- `restore_dashboard.sh` — full container restoration
- `tests/test_exhibition_control.py` — control-boundary regression tests
