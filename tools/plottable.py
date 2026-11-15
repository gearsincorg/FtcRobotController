#!/usr/bin/env python3
"""3D plot of ShooterSubsystem.SHOT_TABLE: distance x flywheel RPM x hood angle.

Draws the table as a line (measured range solid, extrapolated rows marked) and the saved shots from
tools/shooter_cal.csv as points (best shots, and compensation shots in another color), colored by
flight time where timed.  Writes tools/shot_table_3d.html (interactive: drag to rotate, scroll to
zoom, hover for values; uses Plotly from a CDN, so it needs an internet connection to view).

Usage:  python tools/plottable.py [--csv FILE] [--out FILE]
"""

import argparse
import csv
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SHOOTER = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java"
BEGIN = "// BEGIN GENERATED SHOT_TABLE"
END = "// END GENERATED SHOT_TABLE"
COLUMNS = ["time", "goal", "set_dist", "set_angle", "x", "y", "heading_deg", "dist", "rpm", "hood_deg",
           "tof_s", "tof_n", "kind", "rpm_meas", "volts"]


def read_table():
    """[(dist, rpm, hood, tof, note)] from the generated block."""
    text = SHOOTER.read_text(encoding="utf-8")
    block = text.split(BEGIN, 1)[1].split(END, 1)[0]
    rows = []
    for line in block.splitlines():
        m = re.search(r"\{([^{}]*)\}", line)
        if not m:
            continue
        nums = [float(x) for x in m.group(1).split(",") if x.strip()]
        if len(nums) == 4:
            note = line.split("//", 1)[1].strip() if "//" in line else ""
            rows.append((*nums, note))
    return rows


def read_shots(path):
    shots = []
    if not path.exists():
        return shots
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f, fieldnames=COLUMNS):
            if row["time"] == "time":
                continue
            try:
                shots.append({
                    "dist": float(row["dist"]), "rpm": float(row["rpm"]), "hood": float(row["hood_deg"]),
                    "tof": float(row["tof_s"]) if (row["tof_s"] or "").strip() else None,
                    "kind": (row["kind"] or "best").strip() or "best",
                    "time": row["time"],
                })
            except (TypeError, ValueError):
                continue
    return shots


def trace_table(rows, name, color, dash):
    return {
        "type": "scatter3d", "mode": "lines+markers", "name": name,
        "x": [r[0] for r in rows], "y": [r[1] for r in rows], "z": [r[2] for r in rows],
        "line": {"color": color, "width": 6, "dash": dash},
        "marker": {"size": 3, "color": color},
        "text": [f"{r[0]:.1f} in<br>{r[1]:.0f} rpm<br>hood {r[2]:.1f} deg<br>tof {r[3]:.3f} s"
                 + (f"<br>{r[4]}" if r[4] else "") for r in rows],
        "hoverinfo": "text",
    }


def build(table, shots, out):
    measured = [r for r in table if "extrapolated" not in r[4]]
    traces = []
    # Extrapolated parts as dashed lines through the whole table, the measured range solid on top.
    traces.append(trace_table(table, "SHOT_TABLE (extrapolated)", "#9aa4b2", "dash"))
    if measured:
        traces.append(trace_table(measured, "SHOT_TABLE (measured range)", "#2563eb", "solid"))
    for kind, name, symbol in (("best", "best shots", "circle"), ("comp", "comp shots", "diamond")):
        ss = [s for s in shots if s["kind"] == kind]
        if not ss:
            continue
        traces.append({
            "type": "scatter3d", "mode": "markers", "name": name,
            "x": [s["dist"] for s in ss], "y": [s["rpm"] for s in ss], "z": [s["hood"] for s in ss],
            "marker": {"size": 7, "symbol": symbol,
                       "color": "#e11d48" if kind == "best" else "#f59e0b",
                       "line": {"color": "#111", "width": 1}},
            "text": [f"{kind} {s['time']}<br>{s['dist']:.1f} in<br>{s['rpm']:.0f} rpm<br>hood {s['hood']:.1f} deg"
                     f"<br>tof {s['tof']:.3f} s" if s["tof"] is not None else
                     f"{kind} {s['time']}<br>{s['dist']:.1f} in<br>{s['rpm']:.0f} rpm<br>hood {s['hood']:.1f} deg"
                     for s in ss],
            "hoverinfo": "text",
        })
    layout = {
        "title": {"text": "Shooter table: distance x RPM x hood"},
        "scene": {
            "xaxis": {"title": {"text": "distance to goal (in)"}},
            "yaxis": {"title": {"text": "flywheel RPM"}},
            "zaxis": {"title": {"text": "hood (deg from vertical)"}},
            "camera": {"eye": {"x": 1.9, "y": -1.9, "z": 1.1}},
        },
        "margin": {"l": 0, "r": 0, "t": 48, "b": 0},
        "legend": {"x": 0.01, "y": 0.98},
    }
    html = f"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Shot Table 3D</title>
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>html,body{{margin:0;height:100%;background:#fff;font-family:system-ui,sans-serif}}
#plot{{width:100%;height:100vh}}</style></head>
<body><div id="plot"></div>
<script>Plotly.newPlot("plot", {json.dumps(traces)}, {json.dumps(layout)}, {{responsive: true}});</script>
</body></html>
"""
    out.write_text(html, encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--csv", type=Path, default=ROOT / "tools/shooter_cal.csv")
    ap.add_argument("--out", type=Path, default=ROOT / "tools/shot_table_3d.html")
    args = ap.parse_args()
    table = read_table()
    shots = read_shots(args.csv)
    build(table, shots, args.out)
    print(f"Wrote {args.out} ({len(table)} table rows, {len(shots)} shots)")


if __name__ == "__main__":
    main()
