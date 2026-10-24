#!/usr/bin/env python3
"""Convert Turtle Tracer .turt files into one path class per file-name group.

Files in /turt_files are grouped by the text before the last underscore of the file name
(NearCenter_start.turt, NearCenter_2.turt -> group "NearCenter").  Each group becomes
TeamCode/.../teamcode/paths/Path_<Group>.java.  Every group needs exactly one file ending in
_start; its starting position becomes getStartPose().  Within a group the files are sorted
(numbers in natural order, so _2 comes before _10) and each becomes a method path_<suffix>().

A new Path_<Group>.java is created from the template Path_GroupName.java.  After that,
everything above the "Do not change anything above this line" banner is left alone (so
autoCommand() can be edited by hand) and everything below it is regenerated on every run.

Usage:  python tools/turt2java.py
"""

import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TURT_DIR = ROOT / "turt_files"
PATHS_DIR = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/paths"
TEMPLATE_NAME = "GroupName"          # Path_GroupName.java is the template for new groups
START_SUFFIX = "start"

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


def build_method(turt_path, method, is_start):
    data = json.loads(turt_path.read_text(encoding="utf-8"))
    start = data["startPoint"]
    start_deg = first_key(start, "headingDeg", "startDeg", "degrees")
    poses = [("start", start["x"], start["y"], start_deg)]
    segments = []
    used = {"start", "robot", "poseFactory", method + "Path"}

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
        if name == "start" and is_start:
            out.append("        final Pose start = getStartPose();")
        else:
            out.append(f"        final Pose {name} = poseFactory.of({num(x)}, {num(y)}, {num(deg)});")
    out.append("")
    out.append(f"        Path {method}Path = Paths.path(")
    out.append(",\n".join(" " * 16 + s for s in segments))
    out.append("        );")
    out.append("")
    out.append(f"        return new FollowPathCommand(robot.drive, {method}Path);")
    out.append("    }")
    start_pose = (num(poses[0][1]), num(poses[0][2]), num(poses[0][3]))
    return "\n".join(out), start_pose


def natural_key(text):
    return [int(t) if t.isdigit() else t.lower() for t in re.findall(r"\d+|\D+", text)]


def clean(text):
    return re.sub(r"[^A-Za-z0-9]+", "_", text).strip("_")


def header_of(text, source):
    """Lines up to and including the rule below the banner."""
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if BANNER_TEXT in line:
            for j in range(i + 1, len(lines)):
                if lines[j].strip().startswith(BANNER_RULE):
                    return lines[:j + 1]
            break
    raise TurtError(f'could not find the "{BANNER_TEXT}" banner in {source}')


def read_text(path):
    raw = path.read_bytes()
    return raw.decode("utf-8"), ("\r\n" if b"\r\n" in raw else "\n")


def find_groups():
    groups = {}
    for path in sorted(TURT_DIR.glob("*.turt"), key=lambda p: natural_key(p.name)):
        if "_" not in path.stem:
            raise TurtError(f"{path.name}: file names must be <Group>_<suffix>.turt")
        prefix, suffix = path.stem.rsplit("_", 1)
        groups.setdefault(clean(prefix), []).append((clean(suffix), path))
    return groups


def build_group(group, members, template_text):
    """Returns (file text, target path) for Path_<group>.java, or raises TurtError."""
    cls = f"Path_{group}"
    starts = [m for m in members if m[0].lower() == START_SUFFIX]
    if len(starts) != 1:
        raise TurtError(f"needs exactly one *_{START_SUFFIX}.turt file (found {len(starts)})")
    # The start file first, then the rest in natural order.
    ordered = starts + sorted((m for m in members if m not in starts), key=lambda m: natural_key(m[0]))

    methods, names = [], []
    for suffix, path in ordered:
        is_start = path is starts[0][1]
        method = f"path_{suffix}"
        try:
            code, pose = build_method(path, method, is_start)
        except TurtError as error:
            raise TurtError(f"{path.name}: {error}")
        except (KeyError, TypeError, ValueError) as error:
            raise TurtError(f"{path.name}: {error!r}")
        if is_start:
            marker = f"    // Generated from {path.name}\n"
            code = code.replace(marker,
                                marker +
                                "    public Pose getStartPose() {\n"
                                f"        return poseFactory.of({', '.join(pose)}); // from start path in this name group\n"
                                "    }\n\n", 1)
        methods.append(code)
        names.append(method)

    target = PATHS_DIR / f"{cls}.java"
    if target.exists():
        header = header_of(read_text(target)[0], target.name)
    else:
        header = header_of(template_text, f"Path_{TEMPLATE_NAME}.java")
        header = [l.replace(f"Path_{TEMPLATE_NAME}", cls) for l in header]
        # Fill in the command list for a brand-new file.
        calls = ", ".join(n + "()" for n in names)
        header = [re.sub(r"auto\.addCommands\(.*\);", lambda _: f"auto.addCommands({calls});", l) for l in header]
    return "\n".join(header) + "\n\n" + "\n\n".join(methods) + "\n}\n", target


def main():
    template = PATHS_DIR / f"Path_{TEMPLATE_NAME}.java"
    try:
        if not template.exists():
            raise TurtError(f"template {template.name} not found in {PATHS_DIR}")
        template_text, template_newline = read_text(template)
        groups = find_groups()
        if not groups:
            raise TurtError(f"no .turt files found in {TURT_DIR}")
    except TurtError as error:
        print(f"ERROR {error}")
        return 1

    outputs, failed = [], False
    for group, members in groups.items():
        try:
            text, target = build_group(group, members, template_text)
            outputs.append((text, target))
            print(f"  {group}: {', '.join(p.name for _, p in members)} -> {target.name}")
        except TurtError as error:
            failed = True
            print(f"  {group}: ERROR {error}")

    if failed:
        print("No path files were updated. Fix the errors above and run again.")
        return 1

    for text, target in outputs:
        newline = read_text(target)[1] if target.exists() else template_newline
        new_bytes = text.replace("\n", newline).encode("utf-8")
        if target.exists() and target.read_bytes() == new_bytes:
            print(f"{target.name} is already up to date.")
        else:
            target.write_bytes(new_bytes)
            print(f"Wrote {target.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
