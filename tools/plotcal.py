#!/usr/bin/env python3
"""Plot the shooter calibration: flywheel RPM at each saved robot location, with the field on top.

Reads tools/shooter_cal.csv (pull it first with `python tools/shotcal.py --dry-run`) and writes
tools/shooter_cal_plot.html, a self-contained page (open it in any browser).  Each dot is where the
robot was when the point was saved, in Pedro coordinates (inches, origin at a field corner, x right,
y up), shaded by RPM.  Hover a dot for its distance, angle, RPM, hood and flight time.

The field drawn over the dots is a schematic: the 144 in square, 24 in tiles and the goals from
ShooterSubsystem.java.  To lay a real field picture over it instead, pass --field IMAGE (a top-down
picture of the whole field, cropped to the field edges, with Pedro (0, 0) at the bottom left).

Usage:  python tools/plotcal.py [--csv FILE] [--out FILE] [--field IMAGE] [--field-opacity 0.4]
"""

import argparse
import base64
import csv
import html
import json
import mimetypes
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SHOOTER = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java"
FIELD = 144.0

# Sequential blue ramp, light -> dark (steps 250..700 of the reference palette).
RAMP = ["#86b6ef", "#6da7ec", "#5598e7", "#3987e5", "#2a78d6", "#256abf",
        "#1c5cab", "#184f95", "#104281", "#0d366b"]


def fail(msg):
    print("plotcal: " + msg, file=sys.stderr)
    sys.exit(1)


def read_points(path):
    """Latest row for each (goal, set_dist, set_angle), like shotcal.py."""
    points = {}
    with open(path, newline="", encoding="utf-8") as f:
        for n, row in enumerate(csv.DictReader(f), start=2):
            try:
                p = {
                    "goal": row["goal"].strip(),
                    "set_dist": float(row["set_dist"]),
                    "set_angle": float(row["set_angle"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "dist": float(row["dist"]),
                    "rpm": float(row["rpm"]),
                    "hood": float(row["hood_deg"]),
                    "tof": row["tof_s"].strip(),
                    "time": row["time"].strip(),
                }
            except (KeyError, ValueError, AttributeError) as e:
                fail(f"{path}: line {n}: bad row ({e})")
            points[(p["goal"], p["set_dist"], p["set_angle"])] = p
    return sorted(points.values(), key=lambda p: (p["goal"], p["set_dist"], p["set_angle"]))


def read_goals():
    """Goal poses from ShooterSubsystem.java: name -> (x, y)."""
    goals = {}
    if not SHOOTER.exists():
        return goals
    text = SHOOTER.read_text(encoding="utf-8")
    for m in re.finditer(r"Pose\s+(\w*GOAL\w*)\s*=\s*new\s+Pose\(([^,]+),([^)]+)\)", text):
        name, xs, ys = m.groups()
        try:
            x = eval(xs, {"__builtins__": {}})     # plain numbers like 60 or 144 - 54.5
            y = eval(ys, {"__builtins__": {}})
        except Exception:
            continue
        label = name.replace("RED_GOAL_", "").replace("_", " ").title() + " goal"
        goals[label] = (float(x), float(y))
    return goals


def build(points, goals, field_image, field_opacity, csv_name):
    S = 5.0                              # px per inch
    M = 36                               # margin for axis labels
    W = FIELD * S

    def px(x): return M + x * S
    def py(y): return M + (FIELD - y) * S

    lo = min(p["rpm"] for p in points)
    hi = max(p["rpm"] for p in points)

    def color(rpm):
        t = 0 if hi == lo else (rpm - lo) / (hi - lo)
        return RAMP[min(len(RAMP) - 1, int(t * len(RAMP)))]

    dots = []
    for i, p in enumerate(points):
        dots.append(
            f'<circle class="dot" data-i="{i}" cx="{px(p["x"]):.1f}" cy="{py(p["y"]):.1f}" r="5.5" '
            f'fill="{color(p["rpm"])}"/>')

    grid = []
    for k in range(0, 7):
        v = k * 24
        grid.append(f'<line x1="{px(v)}" y1="{py(0)}" x2="{px(v)}" y2="{py(FIELD)}"/>')
        grid.append(f'<line x1="{px(0)}" y1="{py(v)}" x2="{px(FIELD)}" y2="{py(v)}"/>')
    ticks = []
    for k in range(0, 7):
        v = k * 24
        ticks.append(f'<text x="{px(v)}" y="{py(0) + 18}" text-anchor="middle">{v}</text>')
        ticks.append(f'<text x="{px(0) - 8}" y="{py(v) + 4}" text-anchor="end">{v}</text>')

    goal_marks = []
    for label, (gx, gy) in goals.items():
        goal_marks.append(
            f'<g class="goal"><rect x="{px(gx) - 7}" y="{py(gy) - 7}" width="14" height="14" rx="2"/>'
            f'<text x="{px(gx) + 12}" y="{py(gy) + 4}">{html.escape(label)}</text></g>')

    image = ""
    if field_image:
        mime = mimetypes.guess_type(field_image.name)[0] or "image/png"
        data = base64.b64encode(field_image.read_bytes()).decode("ascii")
        image = (f'<image href="data:{mime};base64,{data}" x="{px(0)}" y="{py(FIELD)}" '
                 f'width="{W}" height="{W}" opacity="{field_opacity}" preserveAspectRatio="none"/>')

    legend_stops = "".join(
        f'<stop offset="{i / (len(RAMP) - 1):.3f}" stop-color="{c}"/>' for i, c in enumerate(RAMP))

    rows = "".join(
        f"<tr><td>{p['goal']}</td><td>{p['set_dist']:.0f}</td><td>{p['set_angle']:+.0f}</td>"
        f"<td>{p['x']:.1f}</td><td>{p['y']:.1f}</td><td>{p['dist']:.1f}</td>"
        f"<td>{p['rpm']:.0f}</td><td>{p['hood']:.1f}</td><td>{p['tof'] or '-'}</td></tr>"
        for p in points)

    vb = W + 2 * M
    data_json = json.dumps(points)
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Shooter RPM Map</title>
<style>
  :root {{
    --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e; --muted: #898781;
    --grid: #52514e; --border: rgba(11,11,11,0.10); --goal: #d03b3b; --ring: #fcfcfb;
  }}
  @media (prefers-color-scheme: dark) {{
    :root:not([data-theme="light"]) {{
      --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --muted: #898781;
      --grid: #c3c2b7; --border: rgba(255,255,255,0.10); --ring: #1a1a19;
    }}
  }}
  :root[data-theme="dark"] {{
    --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --muted: #898781;
    --grid: #c3c2b7; --border: rgba(255,255,255,0.10); --ring: #1a1a19;
  }}
  body {{ margin: 0; background: var(--page); color: var(--ink);
         font: 14px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }}
  main {{ max-width: 860px; margin: 0 auto; padding: 24px 16px; }}
  h1 {{ font-size: 20px; margin: 0 0 4px; }}
  .sub {{ color: var(--ink-2); margin: 0 0 16px; }}
  .card {{ background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 16px; }}
  .legend {{ display: flex; align-items: center; gap: 10px; color: var(--ink-2); font-size: 13px;
            margin-bottom: 12px; flex-wrap: wrap; }}
  .legend svg {{ width: 200px; height: 12px; border-radius: 3px; }}
  .legend .goalkey {{ display: inline-flex; align-items: center; gap: 6px; margin-left: 12px; }}
  .legend .goalkey i {{ width: 10px; height: 10px; border-radius: 2px; background: var(--goal); }}
  svg.map {{ width: 100%; height: auto; display: block; }}
  .map .dot {{ stroke: var(--ring); stroke-width: 2; cursor: pointer; }}
  .map .dot:hover, .map .dot.on {{ stroke: var(--ink); }}
  .map .fieldgrid, .map .edge, .map image, .map .goals, .map .ticks {{ pointer-events: none; }}
  .map .fieldgrid line {{ stroke: var(--grid); stroke-width: 1; opacity: 0.35; }}
  .map .edge {{ fill: none; stroke: var(--grid); stroke-width: 2; opacity: 0.7; }}
  .map .ticks text {{ fill: var(--muted); font-size: 11px; }}
  .map .goal rect {{ fill: var(--goal); stroke: var(--ring); stroke-width: 2; }}
  .map .goal text {{ fill: var(--ink-2); font-size: 12px; }}
  .map .axis {{ fill: var(--muted); font-size: 11px; }}
  #tip {{ position: fixed; pointer-events: none; background: var(--surface); color: var(--ink);
         border: 1px solid var(--border); border-radius: 8px; padding: 8px 10px; font-size: 12px;
         box-shadow: 0 4px 16px rgba(0,0,0,0.15); display: none; white-space: nowrap; }}
  #tip b {{ font-size: 14px; }}
  #tip span {{ color: var(--ink-2); }}
  details {{ margin-top: 16px; }}
  summary {{ cursor: pointer; color: var(--ink-2); }}
  .tablewrap {{ overflow-x: auto; }}
  table {{ border-collapse: collapse; font-size: 12px; margin-top: 8px; width: 100%; }}
  th, td {{ padding: 4px 8px; text-align: right; border-bottom: 1px solid var(--border); }}
  th:first-child, td:first-child {{ text-align: left; }}
  th {{ color: var(--ink-2); font-weight: 600; }}
</style></head>
<body><main>
  <h1>Shooter RPM by location</h1>
  <p class="sub">{len(points)} saved points from {html.escape(csv_name)}. Each dot is where the robot
  was when the point was saved (Pedro inches); darker means more RPM. Hover a dot for details.</p>
  <div class="card">
    <div class="legend">
      <span>{lo:.0f} rpm</span>
      <svg viewBox="0 0 200 12" preserveAspectRatio="none" aria-hidden="true">
        <defs><linearGradient id="g">{legend_stops}</linearGradient></defs>
        <rect width="200" height="12" fill="url(#g)"/></svg>
      <span>{hi:.0f} rpm</span>
      <span class="goalkey"><i></i>goal</span>
    </div>
    <svg class="map" viewBox="0 0 {vb} {vb}" role="img"
         aria-label="Field map of saved calibration points shaded by flywheel RPM">
      <g class="dots">{''.join(dots)}</g>
      {image}
      <g class="fieldgrid">{''.join(grid)}</g>
      <rect class="edge" x="{px(0)}" y="{py(FIELD)}" width="{W}" height="{W}"/>
      <g class="goals">{''.join(goal_marks)}</g>
      <g class="ticks">{''.join(ticks)}</g>
      <text class="axis" x="{px(FIELD)}" y="{py(0) + 32}" text-anchor="end">x (in)</text>
      <text class="axis" x="{px(0) - 8}" y="{py(FIELD) - 12}" text-anchor="end">y (in)</text>
    </svg>
  </div>
  <details><summary>Table</summary><div class="tablewrap"><table>
    <thead><tr><th>Goal</th><th>Set dist</th><th>Angle</th><th>x</th><th>y</th><th>Dist</th>
    <th>RPM</th><th>Hood</th><th>Flight s</th></tr></thead>
    <tbody>{rows}</tbody></table></div></details>
</main>
<div id="tip"></div>
<script>
  const pts = {data_json};
  const tip = document.getElementById('tip');
  document.querySelectorAll('.dot').forEach(el => {{
    const p = pts[+el.dataset.i];
    el.addEventListener('mouseenter', () => {{
      el.classList.add('on');
      tip.innerHTML = `<b>${{p.rpm.toFixed(0)}} rpm</b> &nbsp;<span>hood ${{p.hood.toFixed(1)}}&deg;</span><br>` +
        `<span>${{p.set_dist.toFixed(0)}} in @ ${{p.set_angle >= 0 ? '+' : ''}}${{p.set_angle.toFixed(0)}}&deg; from ${{p.goal.toLowerCase()}} goal</span><br>` +
        `<span>robot (${{p.x.toFixed(1)}}, ${{p.y.toFixed(1)}}), measured ${{p.dist.toFixed(1)}} in</span><br>` +
        `<span>flight ${{p.tof ? p.tof + ' s' : 'not timed'}} &middot; ${{p.time}}</span>`;
      tip.style.display = 'block';
    }});
    el.addEventListener('mousemove', e => {{
      const r = tip.getBoundingClientRect();
      let x = e.clientX + 14, y = e.clientY + 14;
      if (x + r.width > innerWidth - 8) x = e.clientX - r.width - 14;
      if (y + r.height > innerHeight - 8) y = e.clientY - r.height - 14;
      tip.style.left = x + 'px'; tip.style.top = y + 'px';
    }});
    el.addEventListener('mouseleave', () => {{ el.classList.remove('on'); tip.style.display = 'none'; }});
  }});
</script>
</body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--csv", type=Path, default=ROOT / "tools/shooter_cal.csv")
    ap.add_argument("--out", type=Path, default=ROOT / "tools/shooter_cal_plot.html")
    ap.add_argument("--field", type=Path, help="top-down field picture to lay over the dots")
    ap.add_argument("--field-opacity", type=float, default=0.4)
    args = ap.parse_args()

    if not args.csv.exists():
        fail(f"{args.csv} not found (pull it with: python tools/shotcal.py --dry-run)")
    if args.field and not args.field.exists():
        fail(f"{args.field} not found")
    points = read_points(args.csv)
    if not points:
        fail(f"no points in {args.csv}")
    page = build(points, read_goals(), args.field, args.field_opacity, args.csv.name)
    args.out.write_text(page, encoding="utf-8")
    print(f"Wrote {args.out} ({len(points)} points)")


if __name__ == "__main__":
    main()
