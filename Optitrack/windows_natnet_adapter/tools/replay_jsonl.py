import json
import sys
from pathlib import Path


def load_frames(path: Path) -> list[dict]:
    frames: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                frames.append(json.loads(stripped))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSONL: {exc}") from exc
    return frames


def summarize(frames: list[dict]) -> dict:
    frame_ids = [int(frame["frame_id"]) for frame in frames if "frame_id" in frame]
    missing = 0
    if len(frame_ids) >= 2:
        expected = frame_ids[-1] - frame_ids[0] + 1
        missing = max(0, expected - len(set(frame_ids)))
    rigid_body_roles = sorted(
        {
            role
            for frame in frames
            for role in frame.get("rigid_bodies", {}).keys()
        }
    )
    rigid_body_tracked_counts = {
        role: sum(
            1
            for frame in frames
            if frame.get("rigid_bodies", {}).get(role, {}).get("tracked") is True
        )
        for role in rigid_body_roles
    }
    return {
        "frames": len(frames),
        "first_frame_id": frame_ids[0] if frame_ids else None,
        "last_frame_id": frame_ids[-1] if frame_ids else None,
        "missing_frame_count": missing,
        "ball_frame_count": sum(1 for frame in frames if frame.get("ball") is not None),
        "rigid_body_roles": rigid_body_roles,
        "rigid_body_tracked_counts": rigid_body_tracked_counts,
    }


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: replay_jsonl.py <session.jsonl>", file=sys.stderr)
        return 2
    frames = load_frames(Path(argv[1]))
    print(json.dumps(summarize(frames), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
