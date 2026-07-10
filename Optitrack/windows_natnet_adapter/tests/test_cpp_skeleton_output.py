import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


class CppSkeletonOutputTest(unittest.TestCase):
    def _compile_and_run(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            self.skipTest("No C++ compiler available")

        project_root = Path(__file__).resolve().parents[1]

        with tempfile.TemporaryDirectory() as temp_dir:
            binary = Path(temp_dir) / "windows_natnet_adapter_skeleton"
            compile_result = subprocess.run(
                [
                    compiler,
                    "-std=c++17",
                    "-I",
                    str(project_root / "include"),
                    str(project_root / "src" / "main.cpp"),
                    str(project_root / "src" / "config.cpp"),
                    str(project_root / "src" / "jsonl_writer.cpp"),
                    "-o",
                    str(binary),
                ],
                cwd=project_root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.assertEqual(compile_result.returncode, 0, compile_result.stderr)

            run_result = subprocess.run(
                [str(binary), "config/example_config.json"],
                cwd=project_root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

        return run_result

    def test_stdout_contains_only_json_lines(self):
        run_result = self._compile_and_run()
        self.assertEqual(run_result.returncode, 0, run_result.stderr)

        stdout_lines = [line for line in run_result.stdout.splitlines() if line.strip()]
        self.assertGreaterEqual(len(stdout_lines), 1)
        for line in stdout_lines:
            frame = json.loads(line)
            self.assertEqual(frame["source"], "optitrack")

    def test_output_reflects_configured_names_and_ball_semantics(self):
        run_result = self._compile_and_run()
        self.assertEqual(run_result.returncode, 0, run_result.stderr)

        frame = json.loads(run_result.stdout.splitlines()[0])

        self.assertIn("table", frame["rigid_bodies"])
        self.assertEqual(frame["rigid_bodies"]["table"]["configured_name"], "PPT")
        self.assertIn("robot_base", frame["rigid_bodies"])
        self.assertEqual(frame["rigid_bodies"]["robot_base"]["configured_name"], "P1_base")
        self.assertIn("ego_camera", frame["rigid_bodies"])
        self.assertEqual(frame["rigid_bodies"]["ego_camera"]["configured_name"], "P1_head_camera")
        self.assertTrue(frame["rigid_bodies"]["ego_camera"]["optional"])
        self.assertEqual(frame["ball"]["semantics"], "unlabeled_marker")
        self.assertFalse(frame["ball"]["validated_as_center"])
