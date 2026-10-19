#!/usr/bin/env python3
"""Convert Turtle Tracer .turt files into methods in AutoPaths.java.

Every .turt file in the turt_files folder becomes one public method (named after the
file) that returns a FollowPathCommand for the whole path.  Everything in
OurPaths.java above the "Do not change anything above this line" banner is
kept as is; everything below it is regenerated on each run.

Usage:  python tools/turt2java.py
"""

import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TURT_DIR = ROOT / "turt_files"
JAVA_FILE = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/AutoPaths.java"

BANNER_TEXT = "Do not change anything above this line"
BANNER_RULE = "// ===="

JAVA_KEYWORDS = {
    "abstract", "assert", "boolean", "break", "byte", "case", "catch", "char", "class", "const",
    "continue", "default", "do", "double", "else", "enum", "extends", "final", "finally", "float",
    "for", "goto", "if", "implements", "import", "instanceof", "int", "interface", "long", "native",
    "new", "package", "private", "protected", "public", "return", "short", "static", "strictfp",
    "super", "switch", "synchronized", "this", "throw", "throws", "transient", "try", "void",
    "volatile", "while", "true", "false", "null",
}


class TurtError(Exception):
    pass


def identifier(text):
    """Turn free text ("Path 1", "Auto1") into a lowerCamelCase Java identifier."""
    words = re.findall(r"[A-Za-z0-9]+", text)
    if not words:
        return ""
    name = words[0][0].lower() + words[0][1:] + "".join(w[0].upper() + w[1:] for w in words[1:])
    if name[0].isdigit() or name in JAVA_KEYWORDS:
        name = "_" + name
    return name


def num(value):
    """Format a number the way the visualizer does: 4 decimals, no trailing zeros."""
    text = f"{round(float(value), 4):.4f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def normalize_deg(deg):
    """Wrap an angle into (-180, 180]."""
    deg = math.fmod(deg, 360)
    if deg > 180:
        deg -= 360
    elif deg <= -180:
        deg += 360
    return deg


def bezier(points, t):
    pts = list(points)
    while len(pts) > 1:
        pts = [((1 - t) * a[0] + t * b[0], (1 - t) * a[1] + t * b[1]) for a, b in zip(pts, pts[1:])]
    return pts[0]


def end_tangent_deg(points):
    """Direction of travel at the end of a segment (sampled like the visualizer's export)."""
    ax, ay = bezier(points, 0.99)
    bx, by = points[-1]
    return math.degrees(math.atan2(by - ay, bx - ax))


def first_key(mapping, *keys):
    for key in keys:
        if key in mapping:
            return mapping[key]
    raise TurtError(f"heading is missing one of {keys}: {mapping}")


def line_heading(line):
    """Heading of a line, from either layout Turtle Tracer writes.

    Either a "heading" object ({type, startDeg, endDeg, reverse}) on the line, or the
    fields on the endPoint ("heading": "linear", plus startDeg / endDeg / degrees).
    """
    heading = line.get("heading")
    if isinstance(heading, dict):
        return heading
    end = line.get("endPoint", {})
    if isinstance(end.get("heading"), str):
        merged = {k: v for k, v in end.items() if k not in ("x", "y")}
        merged["type"] = merged.pop("heading")
        return merged
    return {}


def ordered_lines(data):
    """Lines in the order given by the sequence list (waits and other entries are skipped)."""
    lines = data.get("lines", [])
    sequence = [s for s in data.get("sequence", []) if s.get("kind") == "path"]
    if not sequence:
        return lines
    by_id = {line.get("id"): line for line in lines}
    try:
        return [by_id[s["lineId"]] for s in sequence]
    except KeyError as missing:
        raise TurtError(f"sequence refers to unknown line {missing}")


def build_method(turt_path):
    data = json.loads(turt_path.read_text(encoding="utf-8"))
    method = identifier(turt_path.stem)
    if not method:
        raise TurtError("file name cannot be turned into a method name")

    start = data["startPoint"]
    start_deg = first_key(start, "headingDeg", "startDeg", "degrees")
    poses = [("start", start["x"], start["y"], start_deg)]
    segments = []
    used = {"start", "follower", "poseFactory", method + "Path"}

    def unique(name):
        candidate, n = name, 2
        while candidate in used:
            candidate, n = f"{name}_{n}", n + 1
        used.add(candidate)
        return candidate

    prev_name, prev_xy, prev_deg = "start", (start["x"], start["y"]), start_deg

    for index, line in enumerate(ordered_lines(data), start=1):
        if "endPoint" not in line:
            raise TurtError(f"line {index} (kind '{line.get('kind')}') has no endPoint")
        name = unique(identifier(line.get("name") or "") or f"point{index}")
        end = line["endPoint"]
        end_xy = (end["x"], end["y"])
        controls = [(c["x"], c["y"]) for c in line.get("controlPoints", [])]
        shape = [prev_xy] + controls + [end_xy]

        heading = line_heading(line)
        kind = heading.get("type")
        if kind == "linear":
            end_deg = heading["endDeg"]
            if math.isclose(normalize_deg(heading["startDeg"] - prev_deg), 0, abs_tol=1e-6):
                interpolation = f".linear({prev_name}, {name})"
            else:
                # Start heading differs from the previous pose, so give the angles directly (radians).
                interpolation = (f".linear(Math.toRadians({num(heading['startDeg'])}), "
                                 f"Math.toRadians({num(end_deg)}))")
        elif kind == "constant":
            end_deg = first_key(heading, "degrees", "deg", "constantDeg", "headingDeg")
            interpolation = f".constant({name})"
        elif kind in ("tangential", "tangent"):
            reverse = bool(heading.get("reverse"))
            end_deg = normalize_deg(end_tangent_deg(shape) + (180 if reverse else 0))
            interpolation = ".reverseTangent()" if reverse else ".tangent()"
        else:
            raise TurtError(f"line {index} has unsupported heading type '{kind}'")

        control_names = []
        for c_index, (cx, cy) in enumerate(controls, start=1):
            control_name = unique(f"{name}Control{c_index}")
            control_names.append(control_name)
            poses.append((control_name, cx, cy, 0))
        # Declare the end pose ahead of its control points, as the visualizer's export does.
        poses.insert(len(poses) - len(controls), (name, end["x"], end["y"], end_deg))

        if controls:
            geometry = f"curve({', '.join([prev_name] + control_names + [name])})"
        else:
            geometry = f"line({prev_name}, {name})"
        segments.append(geometry + interpolation)
        prev_name, prev_xy, prev_deg = name, end_xy, end_deg

    if not segments:
        raise TurtError("file contains no paths")

    out = [f"    // Generated from {turt_path.name}",
           f"    public FollowPathCommand {method}() {{"]
    for name, x, y, deg in poses:
        out.append(f"        final Pose {name} = poseFactory.of({num(x)}, {num(y)}, {num(deg)});")
    out.append("")
    out.append(f"        Path {method}Path = Paths.path(")
    out.append(",\n".join(" " * 16 + s for s in segments))
    out.append("        );")
    out.append("")
    out.append("        follower.setPose(start);")
    out.append(f"        return new FollowPathCommand(follower, {method}Path);")
    out.append("    }")
    return method, "\n".join(out)


def read_header():
    lines = JAVA_FILE.read_text(encoding="utf-8").splitlines()
    for i, line in enumerate(lines):
        if BANNER_TEXT in line:
            for j in range(i + 1, len(lines)):
                if lines[j].strip().startswith(BANNER_RULE):
                    return lines[:j + 1]
            break
    raise TurtError(f'could not find the "{BANNER_TEXT}" banner in {JAVA_FILE.name}')


def main():
    turt_files = sorted(TURT_DIR.glob("*.turt"), key=lambda p: p.name.lower())
    if not turt_files:
        print(f"No .turt files found in {TURT_DIR}")
        return 1

    methods, names, failed = [], {}, False
    for turt_path in turt_files:
        try:
            method, code = build_method(turt_path)
            if method in names:
                raise TurtError(f"method name '{method}' already used by {names[method]}")
            names[method] = turt_path.name
            methods.append(code)
            print(f"  {turt_path.name} -> {method}()")
        except (TurtError, KeyError, TypeError, ValueError) as error:
            failed = True
            print(f"  {turt_path.name}: ERROR {error!r}" if isinstance(error, KeyError)
                  else f"  {turt_path.name}: ERROR {error}")

    if failed:
        print(f"{JAVA_FILE.name} was NOT updated. Fix the errors above and run again.")
        return 1

    try:
        header = read_header()
    except TurtError as error:
        print(f"ERROR {error}")
        return 1

    text = "\n".join(header) + "\n\n" + "\n\n".join(methods) + "\n}\n"
    # Keep whatever line endings the file already has.
    newline = "\r\n" if b"\r\n" in JAVA_FILE.read_bytes() else "\n"
    JAVA_FILE.write_text(text, encoding="utf-8", newline=newline)
    print(f"Wrote {len(methods)} method(s) to {JAVA_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
