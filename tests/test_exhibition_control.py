import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

import exhibition_control as control


class ExhibitionClientTests(unittest.TestCase):
    def completed(self, payload, returncode=0):
        return subprocess.CompletedProcess(
            ["ssh"], returncode, stdout=json.dumps(payload), stderr=""
        )

    @mock.patch("exhibition_control.os.path.isfile", return_value=True)
    @mock.patch("exhibition_control.subprocess.run")
    def test_actions_use_fixed_requests(self, run, _isfile):
        run.return_value = self.completed({"workflow_state": "exploring"})
        control.start_mapping("Luna")
        self.assertEqual(run.call_args.args[0][-1], "start-mapping luna")

        control.robot_status("Astro")
        self.assertEqual(run.call_args.args[0][-1], "robot-status astro")

        control.stop_mapping("Luna")
        self.assertEqual(run.call_args.args[0][-1], "stop-mapping luna")

        control.stop_mapping("Astro", save_partial=True)
        self.assertEqual(run.call_args.args[0][-1], "stop-mapping-partial astro")

        run.return_value = self.completed({"workflow_state": "awaiting_localisation"})
        control.prepare_localisation(
            "luna_20261013_120000_completed_map.yaml", "both"
        )
        self.assertEqual(
            run.call_args.args[0][-1],
            "prepare-localisation luna_20261013_120000_completed_map.yaml both",
        )

        control.set_initial_pose("Luna", 1.25, -2, 3.14159)
        self.assertEqual(
            run.call_args.args[0][-1],
            "set-initial-pose luna 1.25 -2 3.14159",
        )

        control.confirm_localisation("Astro")
        self.assertEqual(run.call_args.args[0][-1], "confirm-localisation astro")

        control.stop_patrol()
        self.assertEqual(run.call_args.args[0][-1], "stop-patrol")

        argv = run.call_args.args[0]
        self.assertIsInstance(argv, list)
        self.assertNotIn("shell", run.call_args.kwargs)

    @mock.patch("exhibition_control.os.path.isfile", return_value=True)
    @mock.patch("exhibition_control.subprocess.run")
    def test_read_actions_require_expected_json_shape(self, run, _isfile):
        run.return_value = self.completed({"workflow_state": "idle"})
        self.assertEqual(control.mission_status()["workflow_state"], "idle")
        self.assertEqual(run.call_args.args[0][-1], "mission-status")

        run.return_value = self.completed([{"yaml": "/maps/test.yaml"}])
        self.assertEqual(len(control.list_maps()), 1)
        self.assertEqual(run.call_args.args[0][-1], "list-maps")

        run.return_value = self.completed([])
        with self.assertRaises(control.ExhibitionControlError):
            control.mission_status()

    def test_map_and_pose_validation(self):
        for value in (
            "../map_completed_map.yaml", "/tmp/map_completed_map.yaml",
            "luna_partial_map.yaml", "map.yaml",
        ):
            with self.assertRaises(ValueError):
                control._map_name(value)
        with self.assertRaises(ValueError):
            control._number(float("nan"))
        with self.assertRaises(ValueError):
            control._number(float("inf"))

    @mock.patch("exhibition_control._transition")
    @mock.patch("exhibition_control.mission_status")
    def test_patrol_rechecks_every_confirmation(self, status, transition):
        status.return_value = {
            "workflow_state": "ready_to_patrol",
            "patrol_robots": ["luna", "astro"],
            "localisation": {
                "luna": {"confirmed": True},
                "astro": {"confirmed": False},
            },
        }
        with self.assertRaises(control.ExhibitionControlError):
            control.start_patrol()
        transition.assert_not_called()

        status.return_value["localisation"]["astro"]["confirmed"] = True
        transition.return_value = {"workflow_state": "patrolling"}
        self.assertEqual(control.start_patrol()["workflow_state"], "patrolling")
        transition.assert_called_once_with(["start-patrol"])


class ForcedCommandTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.home = Path(self.temp.name)
        self.root = self.home / "unimelb_project/hardware_code"
        cli = self.root / "scripts/exhibition/exhibitionctl.py"
        cli.parent.mkdir(parents=True)
        (self.root / "maps").mkdir(parents=True)
        cli.write_text(
            "import json,sys\nprint(json.dumps({'argv': sys.argv[1:]}))\n"
        )
        self.wrapper = (
            Path(__file__).parents[1] / "scripts/dashboard/explorer_host.sh"
        )

    def invoke(self, request):
        return subprocess.run(
            ["bash", str(self.wrapper)],
            env={**os.environ, "HOME": str(self.home),
                 "SSH_ORIGINAL_COMMAND": request},
            capture_output=True, text=True,
        )

    def test_maps_are_resolved_under_hardware_repository(self):
        name = "luna_20261013_120000_completed_map.yaml"
        (self.root / "maps" / name).write_text("image: map.pgm\n")
        result = self.invoke(f"prepare-localisation {name} astro")
        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["argv"], [
            "prepare-localisation", str(self.root / "maps" / name),
            "--robots", "astro",
        ])

    def test_rejects_shell_text_partial_maps_and_bad_numbers(self):
        requests = (
            "start-mapping luna;touch /tmp/nope",
            "prepare-localisation ../secret_completed_map.yaml both",
            "prepare-localisation luna_partial_map.yaml both",
            "set-initial-pose luna nan 2 0",
            "unknown-action",
        )
        for request in requests:
            with self.subTest(request=request):
                result = self.invoke(request)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(json.loads(result.stderr)["state"], "error")


if __name__ == "__main__":
    unittest.main()
