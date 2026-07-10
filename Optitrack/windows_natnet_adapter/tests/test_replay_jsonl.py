import json
import tempfile
import unittest
from pathlib import Path

from tools.replay_jsonl import load_frames, summarize


class ReplayJsonlTest(unittest.TestCase):
    def test_summarize_counts_missing_frames(self):
        frames = [
            {
                "frame_id": 10,
                "ball": {"tracked": False},
                "rigid_bodies": {
                    "table": {"tracked": True},
                    "robot_base": {"tracked": False},
                },
            },
            {
                "frame_id": 11,
                "ball": None,
                "rigid_bodies": {
                    "table": {"tracked": True},
                    "robot_base": {"tracked": True},
                },
            },
            {
                "frame_id": 13,
                "ball": {"tracked": True},
                "rigid_bodies": {
                    "table": {"tracked": False},
                    "robot_base": {"tracked": True},
                },
            },
        ]

        summary = summarize(frames)

        self.assertEqual(summary["frames"], 3)
        self.assertEqual(summary["first_frame_id"], 10)
        self.assertEqual(summary["last_frame_id"], 13)
        self.assertEqual(summary["missing_frame_count"], 1)
        self.assertEqual(summary["ball_frame_count"], 2)
        self.assertEqual(summary["rigid_body_roles"], ["robot_base", "table"])
        self.assertEqual(
            summary["rigid_body_tracked_counts"],
            {
                "robot_base": 2,
                "table": 2,
            },
        )

    def test_load_frames_reads_jsonl(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "session.jsonl"
            path.write_text(
                json.dumps({"frame_id": 1}) + "\n" + json.dumps({"frame_id": 2}) + "\n",
                encoding="utf-8",
            )

            self.assertEqual(load_frames(path), [{"frame_id": 1}, {"frame_id": 2}])


if __name__ == "__main__":
    unittest.main()
