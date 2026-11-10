"""Record measured forward motion and prepare a reversed trace without hardware."""

import json
from datetime import datetime, timezone
from pathlib import Path

from .arm import READY_ANGLES, _pose


def save_forward_trace(samples, complete, failure=None):
    """Keep each run separately, including failed captures for diagnosis."""
    directory = Path("hover_trajectories")
    directory.mkdir(exist_ok=True)
    name = datetime.now(timezone.utc).strftime("forward-%Y%m%dT%H%M%S-%fZ.json")
    path = directory / name
    payload = {"schema": 1, "direction": "ready_to_hover", "complete": complete,
               "speed": 5, "samples": samples, "failure": failure}
    # Exclusive creation prevents overwriting another session's evidence.
    with path.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
    return path.resolve()


def reverse_trace(path):
    """Return recorded angles and time intervals in reverse; never move hardware.

    This preserves samples, not a mathematical guarantee of physical retracing.
    No interpolation, angle wrapping, sorting, or closest-path joining is used.
    """
    from .resume_hover import HOVER
    with Path(path).open(encoding="utf-8") as stream:
        data = json.load(stream)
    if (not isinstance(data, dict) or data.get("schema") != 1
            or data.get("direction") != "ready_to_hover"
            or data.get("complete") is not True):
        raise ValueError("A complete measured ready-to-hover recording is required")
    samples = data.get("samples")
    if not isinstance(samples, list) or len(samples) < 3:
        raise ValueError("Recording needs at least three measured samples")
    previous = -1.0
    for sample in samples:
        if not isinstance(sample, dict):
            raise ValueError("Recording samples must be JSON objects")
        stamp = sample.get("elapsed_s")
        if type(stamp) not in (int, float) or not 0 <= stamp < float("inf") or stamp <= previous:
            raise ValueError("Recording timestamps must be finite and strictly increasing")
        if previous >= 0 and stamp - previous > 0.5:
            raise ValueError("Recording has a telemetry gap exceeding 0.5 seconds")
        _pose(sample.get("angles_deg"))
        if (type(sample.get("moving")) is not int or sample["moving"] not in (0, 1)
                or type(sample.get("error")) is not int or sample["error"] != 0):
            raise ValueError("Recording contains invalid motion status")
        previous = stamp
    if max(abs(a - b) for a, b in zip(samples[0]["angles_deg"], READY_ANGLES)) > 3:
        raise ValueError("Recording does not start at saved ready")
    if max(abs(a - b) for a, b in zip(samples[-1]["angles_deg"], HOVER)) > 1.5:
        raise ValueError("Recording does not end at corrected hover")
    end = samples[-1]["elapsed_s"]
    return [{"elapsed_s": end - sample["elapsed_s"],
             "angles_deg": list(sample["angles_deg"])} for sample in reversed(samples)]
