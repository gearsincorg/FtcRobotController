#!/usr/bin/env python3
"""Write the shooter calibration (ShooterSubsystem.SHOT_TABLE) from Shooter Calibration data.

The "Shooter Calibration" opmode (util/ShooterCalibrationOpMode) appends one row per saved point
to /sdcard/FIRST/shooter_cal.csv on the Control Hub.  This tool:

  1. pulls that file over ADB into tools/shooter_cal.csv (commit it: it is the raw data),
     unless --csv names a local file instead;
  2. keeps the latest save for each (goal, distance, angle) point;
  3. makes one table row per calibration distance: the mean measured distance, RPM, hood angle and
     flight time over every angle saved at that distance (an untimed distance gets its flight time
     from a straight-line fit of the timed ones, or the current table if nothing was timed);
  4. prints the table and the spread across angles, and rewrites the lines between
     "// BEGIN GENERATED SHOT_TABLE" and "// END GENERATED SHOT_TABLE" in ShooterSubsystem.java.

Usage:  python tools/shotcal.py [--csv FILE] [--dry-run]
Then review with git diff, build and deploy.

Removing data: delete a point on the robot (select it, press Options twice), or edit
tools/shooter_cal.csv on the PC (delete the bad lines) and then
        python tools/shotcal.py --push
which copies it back to the hub (the hub's old file is kept as shooter_cal.csv.bak), so the next
pull doesn't bring the deleted lines back.  To use local edits without pushing:
        python tools/shotcal.py --csv tools/shooter_cal.csv
"""

import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SHOOTER = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java"
LOCAL_CSV = ROOT / "tools/shooter_cal.csv"
HUB_CSV = "/sdcard/FIRST/shooter_cal.csv"
BEGIN = "// BEGIN GENERATED SHOT_TABLE"
END = "// END GENERATED SHOT_TABLE"


def fail(msg):
    print("shotcal: " + msg, file=sys.stderr)
    sys.exit(1)


def find_adb():
    sdk = None
    props = ROOT / "local.properties"
    if props.exists():
        for line in props.read_text(encoding="utf-8").splitlines():
            if line.strip().startswith("sdk.dir="):
                # local.properties escapes ':' and '\' on Windows (C\:\\Users\\...)
                sdk = line.split("=", 1)[1].strip().replace("\\:", ":").replace("\\\\", "\\")
    sdk = sdk or os.environ.get("ANDROID_HOME") or os.environ.get("ANDROID_SDK_ROOT")
    if sdk:
        adb = Path(sdk) / "platform-tools" / ("adb.exe" if os.name == "nt" else "adb")
        if adb.exists():
            return str(adb)
    return shutil.which("adb")


def connect():
    adb = find_adb()
    if not adb:
        fail("adb not found (set sdk.dir in local.properties or ANDROID_HOME), or use --csv FILE")
    subprocess.run([adb, "connect", "192.168.43.1:5555"], capture_output=True, timeout=15)
    return adb


def push():
    if not LOCAL_CSV.exists():
        fail(f"{LOCAL_CSV.relative_to(ROOT)} not found")
    adb = connect()
    subprocess.run([adb, "shell", "cp", HUB_CSV, HUB_CSV + ".bak"], capture_output=True, timeout=15)
    r = subprocess.run([adb, "push", str(LOCAL_CSV), HUB_CSV], capture_output=True, text=True, timeout=30)
    if r.returncode != 0:
        fail("adb push failed (on the hub's Wi-Fi?):\n" + (r.stderr or r.stdout).strip())
    print(f"Pushed {LOCAL_CSV.relative_to(ROOT)} -> {HUB_CSV} (old copy: {HUB_CSV}.bak)")
    print("Restart the Shooter Calibration opmode to see the change.")


def pull():
    adb = connect()
    r = subprocess.run([adb, "pull", HUB_CSV, str(LOCAL_CSV)], capture_output=True, text=True, timeout=30)
    if r.returncode != 0:
        fail("adb pull failed (on the hub's Wi-Fi?):\n" + (r.stderr or r.stdout).strip())
    print(f"Pulled {HUB_CSV} -> {LOCAL_CSV.relative_to(ROOT)}")
    return LOCAL_CSV


def read_points(path):
    """Latest "best" row for each (goal, set_dist, set_angle); rows with kind=comp are skipped."""
    points = {}
    with open(path, newline="", encoding="utf-8") as f:
        for n, row in enumerate(csv.DictReader(f), start=2):
            if (row.get("kind") or "").strip() == "comp":
                continue                    # compensation shots (other RPMs) don't build the table
            try:
                p = {
                    "goal": row["goal"].strip(),
                    "set_dist": float(row["set_dist"]),
                    "set_angle": float(row["set_angle"]),
                    "dist": float(row["dist"]),
                    "rpm": float(row["rpm"]),
                    "hood": float(row["hood_deg"]),
                    "tof": float(row["tof_s"]) if row["tof_s"].strip() else None,
                }
            except (KeyError, ValueError, AttributeError) as e:
                fail(f"{path}: line {n}: bad row ({e})")
            if p["rpm"] <= 0:
                continue
            points[(p["goal"], p["set_dist"], p["set_angle"])] = p    # later rows win
    return list(points.values())


def mean(xs):
    return sum(xs) / len(xs)


def read_current_table(text):
    """Rows of the SHOT_TABLE now in the source, for flight times nobody measured."""
    block = text.split(BEGIN, 1)[1].split(END, 1)[0]
    rows = []
    for m in re.finditer(r"\{([^{}]*)\}", block):
        nums = [float(x) for x in m.group(1).split(",") if x.strip()]
        if len(nums) == 4:
            rows.append(nums)
    return rows


def interpolate(table, dist):
    if not table:
        return 0.0
    if dist <= table[0][0]:
        return table[0][3]
    for lo, hi in zip(table, table[1:]):
        if dist <= hi[0]:
            t = (dist - lo[0]) / (hi[0] - lo[0])
            return lo[3] + t * (hi[3] - lo[3])
    return table[-1][3]


def build_table(points, current):
    groups = {}
    for p in points:
        groups.setdefault(p["set_dist"], []).append(p)

    rows = []
    for set_dist in sorted(groups):
        ps = groups[set_dist]
        timed = [p["tof"] for p in ps if p["tof"] is not None]
        rows.append({
            "set_dist": set_dist,
            "dist": mean([p["dist"] for p in ps]),
            "rpm": mean([p["rpm"] for p in ps]),
            "hood": mean([p["hood"] for p in ps]),
            "tof": mean(timed) if timed else None,
            "points": sorted(ps, key=lambda p: (p["goal"], p["set_angle"])),
        })

    # Fill in untimed distances: straight-line fit of tof vs distance over the timed rows.
    timed = [(r["dist"], r["tof"]) for r in rows if r["tof"] is not None]
    for r in rows:
        if r["tof"] is not None:
            r["tof_note"] = "measured"
        elif len(timed) >= 2:
            n = len(timed)
            mx, my = mean([d for d, _ in timed]), mean([t for _, t in timed])
            sxx = sum((d - mx) ** 2 for d, _ in timed)
            slope = sum((d - mx) * (t - my) for d, t in timed) / sxx if sxx else 0
            r["tof"] = max(0.0, my + slope * (r["dist"] - mx))
            r["tof_note"] = f"fit of {n} timed"
        elif len(timed) == 1:
            r["tof"] = timed[0][1] * r["dist"] / timed[0][0]
            r["tof_note"] = "scaled from 1 timed"
        else:
            r["tof"] = interpolate(current, r["dist"])
            r["tof_note"] = "old table"
    return sorted(rows, key=lambda r: r["dist"])


def report(rows):
    print()
    for r in rows:
        print(f"{r['set_dist']:>5.0f} in  ->  dist {r['dist']:6.1f}  rpm {r['rpm']:6.0f}  "
              f"hood {r['hood']:5.1f}  tof {r['tof']:.3f} ({r['tof_note']})")
        ps = r["points"]
        for p in ps:
            tof = f"{p['tof']:.3f}" if p["tof"] is not None else "  -  "
            print(f"         {p['goal']:<8} {p['set_angle']:+4.0f} deg  dist {p['dist']:6.1f}  "
                  f"rpm {p['rpm']:6.0f}  hood {p['hood']:5.1f}  tof {tof}")
        if len(ps) > 1:
            rpm_spread = max(p["rpm"] for p in ps) - min(p["rpm"] for p in ps)
            hood_spread = max(p["hood"] for p in ps) - min(p["hood"] for p in ps)
            flag = "   <-- angles disagree" if rpm_spread > 200 or hood_spread > 2 else ""
            print(f"         spread across angles: rpm {rpm_spread:.0f}, hood {hood_spread:.1f}{flag}")
    print()


def table_java(rows, indent):
    lines = [f"{indent}private static final double[][] SHOT_TABLE = {{"]
    for r in rows:
        lines.append(f"{indent}    {{ {r['dist']:6.1f}, {r['rpm']:5.0f}, {r['hood']:5.1f}, {r['tof']:.3f} }},")
    lines.append(f"{indent}}};")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--csv", type=Path, help="use this local CSV instead of pulling from the hub")
    ap.add_argument("--dry-run", action="store_true", help="print the table, don't change the source")
    ap.add_argument("--push", action="store_true",
                    help="copy tools/shooter_cal.csv (after editing it) back to the hub, and stop")
    args = ap.parse_args()

    if args.push:
        push()
        return

    path = args.csv if args.csv else pull()
    if not path.exists():
        fail(f"{path} not found")

    raw = SHOOTER.read_bytes().decode("utf-8")
    newline = "\r\n" if "\r\n" in raw else "\n"
    text = raw.replace("\r\n", "\n")
    if text.count(BEGIN) != 1 or text.count(END) != 1 or text.index(BEGIN) > text.index(END):
        fail(f"{SHOOTER.name}: needs exactly one '{BEGIN}' line followed by one '{END}' line")

    points = read_points(path)
    rows = build_table(points, read_current_table(text))
    if len(rows) < 2:
        fail(f"need at least 2 calibration distances, found {len(rows)} in {path}")
    report(rows)

    lines = text.split("\n")
    b = next(i for i, l in enumerate(lines) if BEGIN in l)
    e = next(i for i, l in enumerate(lines) if END in l)
    indent = lines[b][: len(lines[b]) - len(lines[b].lstrip())]
    new_lines = lines[: b + 1] + table_java(rows, indent) + lines[e:]

    if args.dry_run:
        print("\n".join(table_java(rows, indent)))
        print("\n(dry run: ShooterSubsystem.java not changed)")
        return
    SHOOTER.write_bytes("\n".join(new_lines).replace("\n", newline).encode("utf-8"))
    print(f"Wrote {len(rows)} rows to {SHOOTER.relative_to(ROOT)}; review with git diff.")


if __name__ == "__main__":
    main()
