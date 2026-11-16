#!/usr/bin/env python3
"""Pull and plot the per-loop shooter log (util/ShooterLog.java).

Every opmode run that uses BiobuzzRobot writes one CSV row per loop to
/sdcard/FIRST/shooter_log/<OpMode>_<yyyyMMdd-HHmmss>.csv on the Control Hub (newest 10 kept).
This tool pulls one into tools/logs/ (git-ignored) and writes tools/logs/<name>.html: flywheel
target, commanded and actual RPM over time, with gate-open periods shaded, boost and ramp periods
marked, and the ball count.  Hover for every value in that loop, including why a shot was blocked.
It also prints a summary: loop time, and for each ball fired, how far the flywheel dipped and how
long it took to get back within tolerance.

Usage:
  python tools/shotlog.py              pull the newest log and plot it
  python tools/shotlog.py --list       list the logs on the hub
  python tools/shotlog.py --file NAME  pull and plot that log
  python tools/shotlog.py --csv FILE   plot a local CSV (no ADB)
"""

import argparse
import csv
import html
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shotcal import find_adb  # noqa: E402  (same ADB lookup as shotcal.py)

ROOT = Path(__file__).resolve().parent.parent
LOGS = ROOT / "tools" / "logs"
HUB_DIR = "/sdcard/FIRST/shooter_log"
RPM_TOLERANCE = 400          # ShooterSubsystem.RPM_TOLERANCE (isAtSpeed)
NUMERIC = ["t_s", "dt_ms", "target_rpm", "command_rpm", "actual_rpm", "rpm_err", "ramping", "boosting",
           "at_speed", "ready", "gate_pos", "gate_open", "balls", "shoot_held", "dist_in", "hood_deg",
           "aim_err_deg", "volts"]


def fail(msg):
    print("shotlog: " + msg, file=sys.stderr)
    sys.exit(1)


def adb():
    exe = find_adb()
    if not exe:
        fail("adb not found (set sdk.dir in local.properties or ANDROID_HOME), or use --csv FILE")
    subprocess.run([exe, "connect", "192.168.43.1:5555"], capture_output=True, timeout=15)
    return exe


def hub_files(exe):
    r = subprocess.run([exe, "shell", "ls", HUB_DIR], capture_output=True, text=True, timeout=15)
    if r.returncode != 0:
        fail("can't list " + HUB_DIR + " (on the hub's Wi-Fi? any opmode run yet?):\n"
             + (r.stderr or r.stdout).strip())
    names = [n.strip() for n in r.stdout.split() if n.strip().endswith(".csv")]
    # Names end in _yyyyMMdd-HHmmss.csv, so sort by that stamp across opmodes.
    return sorted(names, key=lambda n: n.rsplit("_", 1)[-1])


def pull(exe, name):
    LOGS.mkdir(parents=True, exist_ok=True)
    dest = LOGS / name
    r = subprocess.run([exe, "pull", HUB_DIR + "/" + name, str(dest)], capture_output=True, text=True, timeout=60)
    if r.returncode != 0:
        fail("adb pull failed:\n" + (r.stderr or r.stdout).strip())
    print(f"Pulled {name} -> {dest.relative_to(ROOT)}")
    return dest


def read_rows(path):
    """Rows as dicts; numeric columns as floats.  A short or bad row (the last one may be cut off) is skipped."""
    rows, skipped = [], 0
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                if None in row.values() or None in row:
                    raise ValueError("short row")
                for k in NUMERIC:
                    row[k] = float(row[k]) if k in row else 0.0   # older logs lack newer columns
                rows.append(row)
            except (KeyError, ValueError, TypeError):
                skipped += 1
    if skipped:
        print(f"  ({skipped} incomplete row(s) skipped)")
    return rows


def summary(rows):
    if not rows:
        fail("no rows in the log (did the opmode get past START?)")
    dur = rows[-1]["t_s"] - rows[0]["t_s"]
    dts = [r["dt_ms"] for r in rows[1:] if r["dt_ms"] > 0]
    print(f"\n{len(rows)} rows over {dur:.1f} s;  loop {sum(dts) / len(dts):.1f} ms mean, "
          f"{max(dts):.0f} ms worst" if dts else f"\n{len(rows)} rows")

    opens = sum(1 for a, b in zip(rows, rows[1:]) if b["gate_open"] and not a["gate_open"])
    print(f"Gate opened {opens} time(s)")

    # A ball fired: the count drops while the intake is SHOOTING.
    shots = [i for i in range(1, len(rows))
             if rows[i]["balls"] < rows[i - 1]["balls"] and rows[i]["intake_mode"] == "SHOOTING"]
    if not shots:
        print("No balls fired.")
        return
    print(f"\n{'ball':>4} {'t (s)':>7} {'target':>7} {'lowest':>7} {'dip':>6} {'back in tol':>12}")
    for n, i in enumerate(shots, 1):
        t0 = rows[i]["t_s"]
        # Up to 1 s after the ball, but not into the next ball's dip.
        t_end = min(t0 + 1.0, rows[shots[n]]["t_s"] if n < len(shots) else t0 + 1.0)
        window = [r for r in rows[i:] if r["t_s"] < t_end] or [rows[i]]
        low = min(window, key=lambda r: r["actual_rpm"])
        back = "-"
        left = False
        for r in rows[i:]:
            if r["t_s"] - t0 > 3.0:
                back = "> 3 s"
                break
            if abs(r["rpm_err"]) >= RPM_TOLERANCE:
                left = True
            elif left:
                back = f"{r['t_s'] - t0:.2f} s"
                break
        if not left:
            back = "stayed in"
        print(f"{n:>4} {t0:>7.2f} {rows[i]['target_rpm']:>7.0f} {low['actual_rpm']:>7.0f} "
              f"{low['actual_rpm'] - rows[i]['target_rpm']:>+6.0f} {back:>12}")


PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Shooter Log</title>
<style>
  :root { --bg:#ffffff; --fg:#1f2328; --muted:#6b7280; --grid:#e5e7eb; --target:#9ca3af;
          --command:#2563eb; --actual:#dc2626; --balls:#7c3aed; --gate:rgba(22,163,74,.16);
          --boost:#f59e0b; --ramp:#8b5cf6; --tip:#111827; --tipfg:#f9fafb; }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) { --bg:#0f1115; --fg:#e5e7eb; --muted:#9ca3af; --grid:#272b33;
      --target:#6b7280; --command:#60a5fa; --actual:#f87171; --balls:#a78bfa; --gate:rgba(74,222,128,.14);
      --boost:#fbbf24; --ramp:#a78bfa; --tip:#f3f4f6; --tipfg:#111827; }
  }
  :root[data-theme="dark"] { --bg:#0f1115; --fg:#e5e7eb; --muted:#9ca3af; --grid:#272b33;
      --target:#6b7280; --command:#60a5fa; --actual:#f87171; --balls:#a78bfa; --gate:rgba(74,222,128,.14);
      --boost:#fbbf24; --ramp:#a78bfa; --tip:#f3f4f6; --tipfg:#111827; }
  body { margin:0; padding:16px; background:var(--bg); color:var(--fg); font:14px system-ui,sans-serif; }
  h1 { font-size:18px; margin:0 0 4px; }  .sub { color:var(--muted); margin:0 0 12px; }
  .legend { display:flex; flex-wrap:wrap; gap:14px; margin:0 0 8px; color:var(--muted); }
  .legend i { display:inline-block; width:14px; height:3px; vertical-align:middle; margin-right:5px; }
  .legend b { display:inline-block; width:14px; height:10px; vertical-align:middle; margin-right:5px; }
  .wrap { overflow-x:auto; border:1px solid var(--grid); border-radius:6px; }
  svg text { fill:var(--muted); font-size:11px; }
  #tip { position:fixed; pointer-events:none; background:var(--tip); color:var(--tipfg); padding:6px 8px;
         border-radius:4px; font:12px ui-monospace,monospace; white-space:pre; display:none; z-index:2; }
  .ctl { margin:8px 0; color:var(--muted); }
</style></head><body>
<h1>Shooter log</h1>
<p class="sub">__NAME__ &middot; __ROWS__ loops over __DUR__ s. Scroll sideways; hover for the values in each loop.</p>
<div class="legend">
  <span><i style="background:var(--target)"></i>target rpm</span>
  <span><i style="background:var(--command)"></i>commanded rpm</span>
  <span><i style="background:var(--actual)"></i>actual rpm</span>
  <span><i style="background:var(--balls)"></i>balls (right axis)</span>
  <span><b style="background:var(--gate)"></b>gate open</span>
  <span><b style="background:var(--boost)"></b>boost</span>
  <span><b style="background:var(--ramp)"></b>ramp</span>
</div>
<div class="ctl">Zoom: <input id="zoom" type="range" min="20" max="400" value="80"> <span id="zl"></span> px per second</div>
<div class="wrap" id="wrap"><svg id="plot"></svg></div>
<div id="tip"></div>
<script>
const R = __DATA__;
const H = 420, M = {l:52, r:40, t:24, b:28};
const svg = document.getElementById('plot'), tip = document.getElementById('tip');
const t0 = R.length ? R[0].t_s : 0, tEnd = R.length ? R[R.length-1].t_s : 1;
let rpmMax = 1000;
for (const r of R) rpmMax = Math.max(rpmMax, r.target_rpm, r.command_rpm, r.actual_rpm);
rpmMax = Math.ceil(rpmMax * 1.08 / 500) * 500;
let pxs = 80, X;
const Y = v => M.t + (H - M.t - M.b) * (1 - v / rpmMax);
const YB = v => M.t + (H - M.t - M.b) * (1 - v / 4.5);
const NS = 'http://www.w3.org/2000/svg';
function el(n, a) { const e = document.createElementNS(NS, n); for (const k in a) e.setAttribute(k, a[k]); return e; }
function bands(key, y, h, color) {
  let start = null;
  const out = [];
  R.forEach((r, i) => {
    const on = r[key] > 0;
    if (on && start === null) start = r.t_s;
    if ((!on || i === R.length - 1) && start !== null) { out.push([start, r.t_s]); start = null; }
  });
  for (const [a, b] of out) svg.appendChild(el('rect', {x: X(a), y, width: Math.max(1, X(b) - X(a)), height: h, fill: color}));
}
function line(key, color, w, step) {
  let d = '';
  R.forEach((r, i) => {
    const x = X(r.t_s).toFixed(1), y = (step ? YB(r[key]) : Y(r[key])).toFixed(1);
    if (i === 0) d = `M${x},${y}`;
    else d += step ? `H${x}V${y}` : `L${x},${y}`;
  });
  svg.appendChild(el('path', {d, fill: 'none', stroke: color, 'stroke-width': w, 'stroke-linejoin': 'round'}));
}
function draw() {
  const W = Math.max(document.getElementById('wrap').clientWidth - 2, M.l + M.r + (tEnd - t0) * pxs);
  X = t => M.l + (t - t0) * pxs;
  svg.setAttribute('width', W); svg.setAttribute('height', H);
  svg.replaceChildren();
  const css = getComputedStyle(document.documentElement);
  const c = n => css.getPropertyValue(n).trim();
  bands('gate_open', M.t, H - M.t - M.b, c('--gate'));
  bands('boost_on', M.t - 10, 5, c('--boost'));
  bands('ramp_on', M.t - 18, 5, c('--ramp'));
  for (let v = 0; v <= rpmMax; v += 500) {
    svg.appendChild(el('line', {x1: M.l, x2: W - M.r, y1: Y(v), y2: Y(v), stroke: c('--grid')}));
    const t = el('text', {x: M.l - 6, y: Y(v) + 4, 'text-anchor': 'end'}); t.textContent = v; svg.appendChild(t);
  }
  for (let b = 0; b <= 4; b++) {
    const t = el('text', {x: W - M.r + 6, y: YB(b) + 4}); t.textContent = b; svg.appendChild(t);
  }
  const step = pxs >= 150 ? 0.5 : pxs >= 60 ? 1 : 5;
  for (let s = Math.ceil(t0 / step) * step; s <= tEnd; s += step) {
    svg.appendChild(el('line', {x1: X(s), x2: X(s), y1: H - M.b, y2: H - M.b + 4, stroke: c('--muted')}));
    const t = el('text', {x: X(s), y: H - 8, 'text-anchor': 'middle'}); t.textContent = s.toFixed(step < 1 ? 1 : 0) + 's'; svg.appendChild(t);
  }
  line('balls', c('--balls'), 1.5, true);
  line('target_rpm', c('--target'), 1.5, false);
  line('command_rpm', c('--command'), 1.5, false);
  line('actual_rpm', c('--actual'), 2, false);
  const cursor = el('line', {y1: M.t, y2: H - M.b, stroke: c('--muted'), 'stroke-dasharray': '3 3', visibility: 'hidden'});
  svg.appendChild(cursor);
  svg.onmousemove = e => {
    const box = svg.getBoundingClientRect();
    const t = t0 + (e.clientX - box.left - M.l) / pxs;
    let lo = 0, hi = R.length - 1;
    while (lo < hi) { const m = (lo + hi) >> 1; if (R[m].t_s < t) lo = m + 1; else hi = m; }
    const r = R[lo]; if (!r) return;
    cursor.setAttribute('x1', X(r.t_s)); cursor.setAttribute('x2', X(r.t_s)); cursor.setAttribute('visibility', 'visible');
    tip.textContent =
      `t ${r.t_s.toFixed(3)} s  (loop ${r.dt_ms.toFixed(0)} ms)\\n` +
      `shooter ${r.shooter_mode}   intake ${r.intake_mode}\\n` +
      `rpm target ${r.target_rpm}  cmd ${r.command_rpm}  actual ${r.actual_rpm}  err ${r.rpm_err > 0 ? '+' : ''}${r.rpm_err}\\n` +
      `${r.ramping ? 'RAMP ' : ''}${r.boosting ? 'BOOST ' : ''}at speed ${r.at_speed ? 'yes' : 'no'}  ready ${r.ready ? 'yes' : 'no'}\\n` +
      `gate ${r.gate_open ? 'OPEN' : 'closed'} (${r.gate_pos})  balls ${r.balls}  fire ${r.shoot_held ? 'held' : '-'}\\n` +
      `dist ${r.dist_in} in  hood ${r.hood_deg}  aim err ${r.aim_err_deg} deg  ${r.volts} V` +
      (r.blocked ? `\\nBLOCKED: ${r.blocked}` : '');
    tip.style.display = 'block';
    tip.style.left = Math.min(e.clientX + 14, innerWidth - tip.offsetWidth - 8) + 'px';
    tip.style.top = (e.clientY + 14) + 'px';
  };
  svg.onmouseleave = () => { tip.style.display = 'none'; cursor.setAttribute('visibility', 'hidden'); };
}
const z = document.getElementById('zoom'), zl = document.getElementById('zl');
z.oninput = () => { pxs = +z.value; zl.textContent = pxs; draw(); };
zl.textContent = pxs;
draw();
addEventListener('resize', draw);
</script></body></html>
"""


def write_html(rows, src):
    keep = ["t_s", "dt_ms", "shooter_mode", "target_rpm", "command_rpm", "actual_rpm", "rpm_err", "ramping",
            "boosting", "at_speed", "ready", "blocked", "intake_mode", "gate_pos", "gate_open", "balls",
            "shoot_held", "dist_in", "hood_deg", "aim_err_deg", "volts"]
    data = []
    for r in rows:
        d = {k: r[k] for k in keep}
        d["boost_on"] = r["boosting"]
        d["ramp_on"] = r["ramping"]
        data.append(d)
    dur = rows[-1]["t_s"] - rows[0]["t_s"] if rows else 0
    page = (PAGE.replace("__DATA__", json.dumps(data, separators=(",", ":")))
                .replace("__NAME__", html.escape(src.name))
                .replace("__ROWS__", str(len(rows)))
                .replace("__DUR__", f"{dur:.1f}"))
    out = src.with_suffix(".html")
    out.write_text(page, encoding="utf-8")
    print(f"\nPlot: {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--list", action="store_true", help="list the logs on the hub")
    ap.add_argument("--file", help="pull and plot this log (a name from --list)")
    ap.add_argument("--csv", type=Path, help="plot a local CSV instead of pulling from the hub")
    args = ap.parse_args()

    if args.csv:
        if not args.csv.exists():
            fail(f"{args.csv} not found")
        path = args.csv
    else:
        exe = adb()
        names = hub_files(exe)
        if args.list:
            for n in names:
                print(n)
            return
        if not names:
            fail("no logs on the hub yet")
        name = args.file or names[-1]
        if name not in names:
            fail(f"{name} is not on the hub (try --list)")
        path = pull(exe, name)

    rows = read_rows(path)
    summary(rows)
    write_html(rows, path)


if __name__ == "__main__":
    main()
