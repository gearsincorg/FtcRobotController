#!/usr/bin/env python3
"""Write the shooter calibration (ShooterSubsystem.SHOT_TABLE) from Shooter Calibration data.

The "Shooter Calibration" opmode (util/ShooterCalibrationOpMode) appends one row per saved point
to /sdcard/FIRST/shooter_cal.csv on the Control Hub.  This tool:

  1. pulls that file over ADB into tools/shooter_cal.csv (commit it: it is the raw data),
     unless --csv names a local file instead;
  2. uses every saved best shot (comp rows are skipped; nothing else is filtered and repeats are all
     kept), wherever it was taken from;
  3. groups the shots by measured distance to the goal into BIN_IN-inch bins and makes one table row
     per bin: the mean distance, RPM, hood angle and flight time of its shots (an untimed bin gets its
     flight time from a straight-line fit of the timed ones, or the current table if nothing was timed);
  4. prints the table and the spread within each bin, and rewrites the lines between
     "// BEGIN GENERATED SHOT_TABLE" and "// END GENERATED SHOT_TABLE" in ShooterSubsystem.java.

Usage:  python tools/shotcal.py [--csv FILE] [--dry-run] [--fit [--degree 2|3]]

--fit replaces step 3: RPM, hood and flight time are each fitted to distance (least squares, a line by
default), and the table gets a row every 3 in from 12 to 96 in, extrapolating past the measured shots
(hood and RPM are clamped to what the hardware can do; those rows are marked in the generated table).
Then review with git diff, build and deploy.

Removing data: undo the last save on the robot (Options twice), or edit
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
BIN_IN = 3.0        # shots whose distances round to the same multiple of this make one table row

COLUMNS = ["time", "goal", "set_dist", "set_angle", "x", "y", "heading_deg", "dist", "rpm", "hood_deg",
           "tof_s", "tof_n", "kind", "rpm_meas", "volts"]


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
    """Every "best" row (rows with kind=comp are skipped); repeats are all kept."""
    points = []
    with open(path, newline="", encoding="utf-8") as f:
        # Columns by position, whatever the header says (an old file may still have the 12-column
        # header above newer 15-column rows); missing trailing columns read as None.
        for n, row in enumerate(csv.DictReader(f, fieldnames=COLUMNS), start=1):
            if row["time"] == "time":
                continue                    # the header line
            if (row.get("kind") or "").strip() == "comp":
                continue                    # compensation shots (other RPMs) don't build the table
            try:
                p = {
                    "goal": row["goal"].strip(),
                    "set_angle": opt_float(row.get("set_angle")),   # blank: shot from anywhere
                    "dist": float(row["dist"]),
                    "rpm": float(row["rpm"]),
                    "hood": float(row["hood_deg"]),
                    "tof": float(row["tof_s"]) if row["tof_s"].strip() else None,
                }
            except (KeyError, ValueError, AttributeError) as e:
                fail(f"{path}: line {n}: bad row ({e})")
            if p["rpm"] <= 0:
                continue
            points.append(p)
    return points


def opt_float(s):
    s = (s or "").strip()
    return float(s) if s else None


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
        groups.setdefault(round(p["dist"] / BIN_IN) * BIN_IN, []).append(p)

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
            "points": sorted(ps, key=lambda p: p["dist"]),
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
        print(f"{r['set_dist']:>5.0f} in  ({len(r['points'])} shots)  ->  dist {r['dist']:6.1f}  rpm {r['rpm']:6.0f}  "
              f"hood {r['hood']:5.1f}  tof {r['tof']:.3f} ({r['tof_note']})")
        ps = r["points"]
        for p in ps:
            tof = f"{p['tof']:.3f}" if p["tof"] is not None else "  -  "
            ang = f"{p['set_angle']:+4.0f} deg" if p["set_angle"] is not None else "anywhere"
            print(f"         {p['goal']:<8} {ang:>8}  dist {p['dist']:6.1f}  "
                  f"rpm {p['rpm']:6.0f}  hood {p['hood']:5.1f}  tof {tof}")
        if len(ps) > 1:
            rpm_spread = max(p["rpm"] for p in ps) - min(p["rpm"] for p in ps)
            hood_spread = max(p["hood"] for p in ps) - min(p["hood"] for p in ps)
            flag = "   <-- shots disagree" if rpm_spread > 200 or hood_spread > 2 else ""
            print(f"         spread: rpm {rpm_spread:.0f}, hood {hood_spread:.1f}{flag}")
    print()


# == --fit: fill the whole range from a least-squares fit ==================

FIT_MIN, FIT_MAX, FIT_STEP = 12.0, 96.0, 3.0     # same range and step as the calibration points


def polyfit(xs, ys, degree):
    """Least-squares polynomial coefficients c0 + c1 x + ... (no numpy)."""
    n = degree + 1
    a = [[sum(x ** (i + j) for x in xs) for j in range(n)] + [sum(y * x ** i for x, y in zip(xs, ys))]
         for i in range(n)]
    for col in range(n):                                    # Gaussian elimination, partial pivoting
        piv = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-12:
            fail(f"can't fit degree {degree}: need more distinct distances")
        a[col], a[piv] = a[piv], a[col]
        for r in range(n):
            if r != col:
                f = a[r][col] / a[col][col]
                a[r] = [v - f * w for v, w in zip(a[r], a[col])]
    return [a[i][n] / a[i][i] for i in range(n)]


def poly(c, x):
    return sum(ci * x ** i for i, ci in enumerate(c))


def java_const(text, name):
    # A number, or simple arithmetic on numbers (e.g. FLYWHEEL_GEAR_RATIO = 4.0 / 3.0).
    m = re.search(r"\b" + name + r"\s*=\s*([-0-9.+*/() ]+?)\s*;", text)
    if not m:
        fail(f"{SHOOTER.name}: can't find {name}")
    return float(eval(m.group(1), {"__builtins__": {}}))


def fit_table(points, text, degree):
    """One row every FIT_STEP in from FIT_MIN to FIT_MAX, from fits of rpm, hood and tof vs distance."""
    if len({round(p["dist"], 1) for p in points}) < degree + 2:
        fail(f"--fit degree {degree} needs at least {degree + 2} different distances")
    hood_lo = min(java_const(text, "HOOD_ANGLE_A_DEG"), java_const(text, "HOOD_ANGLE_B_DEG"))
    hood_hi = max(java_const(text, "HOOD_ANGLE_A_DEG"), java_const(text, "HOOD_ANGLE_B_DEG"))
    rpm_hi = java_const(text, "MOTOR_FREE_RPM") * java_const(text, "FLYWHEEL_GEAR_RATIO")

    xs = [p["dist"] for p in points]
    c_rpm = polyfit(xs, [p["rpm"] for p in points], degree)
    c_hood = polyfit(xs, [p["hood"] for p in points], degree)
    timed = [p for p in points if p["tof"] is not None]
    tdeg = min(degree, len({round(p["dist"], 1) for p in timed}) - 1)
    c_tof = polyfit([p["dist"] for p in timed], [p["tof"] for p in timed], tdeg) if tdeg >= 0 else None

    print(f"\nFit (degree {degree}) of {len(points)} shots, {len(timed)} timed:")
    for p in sorted(points, key=lambda p: p["dist"]):
        print(f"   dist {p['dist']:5.1f}  rpm {p['rpm']:5.0f} (fit {poly(c_rpm, p['dist']):5.0f})  "
              f"hood {p['hood']:4.1f} (fit {poly(c_hood, p['dist']):4.1f})")

    lo, hi = min(xs), max(xs)
    rows, d = [], FIT_MIN
    while d <= FIT_MAX + 1e-9:
        rpm, hood = poly(c_rpm, d), poly(c_hood, d)
        notes = []
        if not lo <= d <= hi:
            notes.append("extrapolated")
        if hood < hood_lo or hood > hood_hi:
            notes.append(f"hood {hood:.1f} clamped")
            hood = min(max(hood, hood_lo), hood_hi)
        if rpm > rpm_hi or rpm < 0:
            notes.append(f"rpm {rpm:.0f} clamped")
            rpm = min(max(rpm, 0.0), rpm_hi)
        tof = max(0.0, poly(c_tof, d)) if c_tof else interpolate(read_current_table(text), d)
        rows.append({"dist": d, "rpm": rpm, "hood": hood, "tof": tof, "note": ", ".join(notes)})
        d += FIT_STEP
    print(f"\nMeasured {lo:.1f}-{hi:.1f} in; rows outside that are extrapolated:")
    for r in rows:
        print(f"   {r['dist']:5.1f} in  rpm {r['rpm']:5.0f}  hood {r['hood']:4.1f}  tof {r['tof']:.3f}  {r['note']}")
    print()
    return rows


def table_java(rows, indent):
    lines = [f"{indent}private static final double[][] SHOT_TABLE = {{"]
    for r in rows:
        note = f"   // {r['note']}" if r.get("note") else ""
        lines.append(f"{indent}    {{ {r['dist']:6.1f}, {r['rpm']:5.0f}, {r['hood']:5.1f}, {r['tof']:.3f} }},{note}")
    lines.append(f"{indent}}};")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--csv", type=Path, help="use this local CSV instead of pulling from the hub")
    ap.add_argument("--dry-run", action="store_true", help="print the table, don't change the source")
    ap.add_argument("--push", action="store_true",
                    help="copy tools/shooter_cal.csv (after editing it) back to the hub, and stop")
    ap.add_argument("--fit", action="store_true",
                    help=f"fill {FIT_MIN:.0f}-{FIT_MAX:.0f} in every {FIT_STEP:.0f} in from a least-squares fit "
                         "(extrapolates past the measured range)")
    ap.add_argument("--degree", type=int, default=1, choices=[1, 2, 3],
                    help="--fit: 1 = straight line (default, safest to extrapolate), 2 = curve, 3 = cubic "
                         "(follows the data closely, but bends sharply past the measured range)")
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
    if args.fit:
        rows = fit_table(points, text, args.degree)
    else:
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
