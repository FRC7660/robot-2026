#!/usr/bin/env python3
"""
Flip a PathPlanner .path file from one alliance side to the other.

Uses the same logic as PathPlanner's FlippingUtil (rotational symmetry):
  x'        = FIELD_X - x
  y'        = FIELD_Y - y
  rotation' = rotation - 180  (normalized to [-180, 180])

Field dimensions from PathPlanner 2026.1.2 FlippingUtil source:
  fieldSizeX = 16.54 m
  fieldSizeY = 8.07 m
  symmetryType = kRotational

Usage:
  python3 flip_path.py <input.path> [output.path]

  If output.path is omitted, the input file is overwritten in place.
"""

import json
import sys

FIELD_X = 16.54
FIELD_Y = 8.07


def flip_x(x):
    return FIELD_X - x


def flip_y(y):
    return FIELD_Y - y


def flip_rotation_deg(deg):
    """Rotate by -180 deg, normalized to (-180, 180]."""
    result = (deg - 180.0) % 360.0
    if result > 180.0:
        result -= 360.0
    return result


def flip_point(pt):
    """Flip a dict with 'x' and 'y' keys."""
    if pt is None:
        return None
    return {"x": flip_x(pt["x"]), "y": flip_y(pt["y"])}


def flip_path(data):
    # Flip waypoints
    for wp in data.get("waypoints", []):
        wp["anchor"] = flip_point(wp["anchor"])
        wp["prevControl"] = flip_point(wp.get("prevControl"))
        wp["nextControl"] = flip_point(wp.get("nextControl"))

    # Flip rotation targets
    for rt in data.get("rotationTargets", []):
        rt["rotationDegrees"] = flip_rotation_deg(rt["rotationDegrees"])

    # Flip ideal starting state rotation
    if "idealStartingState" in data and data["idealStartingState"] is not None:
        data["idealStartingState"]["rotation"] = flip_rotation_deg(
            data["idealStartingState"]["rotation"]
        )

    # Flip goal end state rotation
    if "goalEndState" in data and data["goalEndState"] is not None:
        data["goalEndState"]["rotation"] = flip_rotation_deg(
            data["goalEndState"]["rotation"]
        )

    return data


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else input_path

    with open(input_path, "r") as f:
        data = json.load(f)

    flipped = flip_path(data)

    with open(output_path, "w") as f:
        json.dump(flipped, f, indent=2)
        f.write("\n")

    print(f"Flipped '{input_path}' → '{output_path}'")
    print(f"  Field: {FIELD_X} x {FIELD_Y} m  (rotational symmetry)")


if __name__ == "__main__":
    main()
