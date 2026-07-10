import unittest
from pathlib import Path


class WindowsHandoffDocsTest(unittest.TestCase):
    def setUp(self):
        self.project_root = Path(__file__).resolve().parents[1]

    def test_windows_setup_doc_covers_required_handoff_items(self):
        setup_doc = self.project_root / "WINDOWS_SETUP.md"
        text = setup_doc.read_text(encoding="utf-8")

        required_terms = [
            "Visual Studio 2022",
            "CMake 3.20",
            "Python 3.10",
            "NATNET_SDK_DIR",
            "Motive",
            "NatNet SDK",
            "1510",
            "1511",
            "SampleClient",
            "MinimalClient",
            "config/lab_config.json",
            "tools/replay_jsonl.py",
        ]
        for term in required_terms:
            with self.subTest(term=term):
                self.assertIn(term, text)

    def test_field_log_template_records_reproducibility_fields(self):
        log_template = self.project_root / "logs" / "README.md"
        text = log_template.read_text(encoding="utf-8")

        required_fields = [
            "Motive version",
            "NatNet SDK version",
            "Server IP",
            "Local IP",
            "Connection type",
            "Command port",
            "Data port",
            "Table rigid body name",
            "Robot base rigid body name",
            "Ego camera rigid body name",
            "Ball source",
            "Ball physical setup",
        ]
        for field in required_fields:
            with self.subTest(field=field):
                self.assertIn(field, text)

    def test_windows_scripts_include_expected_commands(self):
        build_script = (self.project_root / "scripts" / "build_windows.ps1").read_text(encoding="utf-8")
        smoke_script = (self.project_root / "scripts" / "run_smoke_test.ps1").read_text(encoding="utf-8")

        self.assertIn("NATNET_SDK_DIR", build_script)
        self.assertIn("OPTITRACK_ADAPTER_WITH_NATNET=ON", build_script)
        self.assertIn("Visual Studio 17 2022", build_script)
        self.assertIn("windows_natnet_adapter.exe", smoke_script)
        self.assertIn("tools\\replay_jsonl.py", smoke_script)
        self.assertIn("logs\\smoke_session.jsonl", smoke_script)


if __name__ == "__main__":
    unittest.main()
