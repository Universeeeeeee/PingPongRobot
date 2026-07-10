import json
import unittest
from pathlib import Path


class ExampleConfigTest(unittest.TestCase):
    def test_example_config_defines_required_tracks_and_outputs(self):
        config_path = Path(__file__).resolve().parents[1] / "config" / "example_config.json"
        config = json.loads(config_path.read_text(encoding="utf-8"))

        self.assertEqual(config["connection_type"], "multicast")
        self.assertEqual(config["command_port"], 1510)
        self.assertEqual(config["data_port"], 1511)
        self.assertEqual(config["rigid_body_names"]["table"], "PPT")
        self.assertEqual(config["rigid_body_names"]["robot_base"], "P1_base")
        self.assertIn("ego_camera", config["optional_rigid_bodies"])
        self.assertEqual(config["ball"]["source"], "unlabeled_marker")
        self.assertFalse(config["ball"]["validated_as_center"])
        self.assertFalse(config["output"]["enable_udp"])


if __name__ == "__main__":
    unittest.main()
