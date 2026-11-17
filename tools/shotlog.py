#!/usr/bin/env python3
"""Pull and plot the per-loop shooter log (util/ShooterLog.java).

Every opmode run that uses BiobuzzRobot writes one CSV row per loop to
/sdcard/FIRST/shooter_log/<OpMode>_<yyyyMMdd-HHmmss>.csv on the Control Hub (newest 10 kept).
This tool pulls one into tools/logs/ (the CSV is git-ignored) and writes tools/logs/<name>.html (committed): flywheel
target, commanded and actual RPM over time, with gate-open periods shaded, boost and ramp periods
marked, and the ball count.  Hover for every value in that loop, including why a shot was blocked.
It also prints a summary: loop time, and for each ball fired, how far the flywheel dipped and how
long it took to get back within tolerance.

Every run also rewrites tools/logs/index.html: a list of every log page in tools/logs/ (newest
first) with its opmode, date, time, duration and loop count, each linking to its page.  A CSV in
tools/logs/ with no page yet gets its page made first.

Usage:
  python tools/shotlog.py              pull every hub log not yet in tools/logs (and the newest again,
                                       in case it was still being written), plot them, summarise the
                                       newest, update the index
  python tools/shotlog.py --list       list the logs on the hub
  python tools/shotlog.py --file NAME  pull and plot that log, update the index
  python tools/shotlog.py --csv FILE   plot a local CSV (no ADB), update the index
  python tools/shotlog.py --index      only make missing pages and rewrite the index (no ADB)
  python tools/shotlog.py --rebuild    remake every page that has its CSV (after a plot change), then
                                       the index (no ADB)
"""

import argparse
import csv
import html
import json
import re
import subprocess
from datetime import datetime
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shotcal import find_adb  # noqa: E402  (same ADB lookup as shotcal.py)

ROOT = Path(__file__).resolve().parent.parent
LOGS = ROOT / "tools" / "logs"
HUB_DIR = "/sdcard/FIRST/shooter_log"
INDEX = LOGS / "index.html"
RPM_TOLERANCE = 200          # ShooterSubsystem.RPM_TOLERANCE (isAtSpeed)
NUMERIC = ["t_s", "dt_ms", "target_rpm", "command_rpm", "actual_rpm", "rpm_err", "ramping", "boosting",
           "at_speed", "ready", "gate_pos", "gate_open", "balls", "shoot_held", "dist_in", "hood_deg",
           "aim_err_deg", "volts", "power", "ff_power", "i_power"]


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


def fit_feedforward(rows):
    """Fit volts = KS + KV * rpm from the steady stretches (ShooterSubsystem.FLYWHEEL_KS / KV).

    Steady: not boosting or ramping, gate closed, the speed within 64 RPM (one reading step) of
    where it is 0.3 s later.  There P and I have settled, so the volts applied (power x battery)
    are what that speed needs, whatever the gains.  Logs from before the power column was added
    have no data for it.
    """
    pts = []
    j = 0
    for i, r in enumerate(rows):
        # Skip full power: the motor can't give more, so it says nothing about what the speed needs.
        if (r["boosting"] or r["ramping"] or r["gate_open"] or r["power"] <= 0 or r["power"] >= 0.99
                or r["volts"] <= 0):
            continue
        while j < len(rows) and rows[j]["t_s"] < r["t_s"] + 0.3:
            j += 1
        if j >= len(rows):
            break
        later = rows[j]
        if later["boosting"] or later["gate_open"] or abs(later["actual_rpm"] - r["actual_rpm"]) > 64:
            continue
        pts.append((r["actual_rpm"], r["power"] * r["volts"]))
    if len(pts) < 50:
        print("Feedforward fit: not enough steady data" + ("" if any(r["power"] for r in rows)
                                                            else " (no power column in this log)"))
        return
    n = len(pts)
    mx = sum(x for x, _ in pts) / n
    my = sum(y for _, y in pts) / n
    sxx = sum((x - mx) ** 2 for x, _ in pts)
    # Separate KS and KV only from a real spread of targets (the speed reading alone wanders a few
    # hundred RPM around one target, which gives nonsense).  Otherwise keep the current KS and
    # give the KV that makes the feedforward right at this speed.
    targets = [r["command_rpm"] for r in rows if r["command_rpm"] > 0 and not r["ramping"]]
    spread = (max(targets) - min(targets)) if targets else 0
    if spread < 800:
        ks_now = current_ks()
        print(f"Feedforward ({n} steady rows near {mx:.0f} rpm, needs {my:.2f} V there; targets only "
              f"span {spread:.0f} rpm, too little to fit KS): with FLYWHEEL_KS = {ks_now:.2f} V, "
              f"FLYWHEEL_KV = {(my - ks_now) / mx:.5f} V/rpm")
        return
    kv = sum((x - mx) * (y - my) for x, y in pts) / sxx
    ks = my - kv * mx
    lo, hi = min(x for x, _ in pts), max(x for x, _ in pts)
    print(f"Feedforward fit ({n} steady rows, {lo:.0f}-{hi:.0f} rpm): FLYWHEEL_KS = {ks:.2f} V, "
          f"FLYWHEEL_KV = {kv:.5f} V/rpm")


SHOOTER_JAVA = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java"


def current_ks():
    """FLYWHEEL_KS as it is in ShooterSubsystem.java now (0.5 if it can't be read)."""
    try:
        m = re.search(r"FLYWHEEL_KS\s*=\s*([-\d.]+)", SHOOTER_JAVA.read_text(encoding="utf-8"))
        return float(m.group(1))
    except (OSError, AttributeError, ValueError):
        return 0.5

def summary(rows):
    if not rows:
        fail("no rows in the log (did the opmode get past START?)")
    dur = rows[-1]["t_s"] - rows[0]["t_s"]
    dts = [r["dt_ms"] for r in rows[1:] if r["dt_ms"] > 0]
    print(f"\n{len(rows)} rows over {dur:.1f} s;  loop {sum(dts) / len(dts):.1f} ms mean, "
          f"{max(dts):.0f} ms worst" if dts else f"\n{len(rows)} rows")

    opens = sum(1 for a, b in zip(rows, rows[1:]) if b["gate_open"] and not a["gate_open"])
    print(f"Gate opened {opens} time(s)")

    fit_feedforward(rows)

    # Balls fired: the count drops while shoot is held.  The count is debounced, so the drop is
    # usually seen after the gate has already closed again (SPINUP), not while SHOOTING.  Each drop is
    # timed from the gate opening that let the ball(s) out, the last one before the drop.
    shots = []
    for i in range(1, len(rows)):
        drop = rows[i - 1]["balls"] - rows[i]["balls"]
        if drop > 0 and rows[i]["shoot_held"]:
            j = i
            while j > 0 and not (rows[j]["gate_open"] and not rows[j - 1]["gate_open"]):
                j -= 1
            shots.append((j, drop))
    if not shots:
        print("No balls fired.")
        return
    print(f"\n{'shot':>4} {'balls':>5} {'gate t':>7} {'target':>7} {'lowest':>7} {'dip':>6} {'back in tol':>12}")
    for n, (i, drop) in enumerate(shots):
        t0 = rows[i]["t_s"]
        # Up to 1 s after the gate opened, but not into the next shot's dip.
        t_end = min(t0 + 1.0, rows[shots[n + 1][0]]["t_s"] if n + 1 < len(shots) else t0 + 1.0)
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
        print(f"{n + 1:>4} {drop:>5.0f} {t0:>7.2f} {rows[i]['target_rpm']:>7.0f} {low['actual_rpm']:>7.0f} "
              f"{low['actual_rpm'] - rows[i]['target_rpm']:>+6.0f} {back:>12}")

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Shooter Log</title>
<style>
  :root { --bg:#ffffff; --fg:#1f2328; --muted:#6b7280; --grid:#e5e7eb; --target:#9ca3af;
          --command:#2563eb; --actual:#dc2626; --balls:#7c3aed; --gate:rgba(22,163,74,.16);
          --boost:#f59e0b; --ramp:#8b5cf6; --volts:#0891b2; --tip:#111827; --tipfg:#f9fafb; }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) { --bg:#0f1115; --fg:#e5e7eb; --muted:#9ca3af; --grid:#272b33;
      --target:#6b7280; --command:#60a5fa; --actual:#f87171; --balls:#a78bfa; --gate:rgba(74,222,128,.14);
      --boost:#fbbf24; --ramp:#a78bfa; --volts:#22d3ee; --tip:#f3f4f6; --tipfg:#111827; }
  }
  :root[data-theme="dark"] { --bg:#0f1115; --fg:#e5e7eb; --muted:#9ca3af; --grid:#272b33;
      --target:#6b7280; --command:#60a5fa; --actual:#f87171; --balls:#a78bfa; --gate:rgba(74,222,128,.14);
      --boost:#fbbf24; --ramp:#a78bfa; --volts:#22d3ee; --tip:#f3f4f6; --tipfg:#111827; }
  body { margin:0; padding:16px; background:var(--bg); color:var(--fg); font:14px system-ui,sans-serif; }
  h1 { font-size:18px; margin:0 0 4px; }  .sub { color:var(--muted); margin:0 0 12px; }
  .legend { display:flex; flex-wrap:wrap; gap:14px; margin:0 0 8px; color:var(--muted); }
  .legend i { display:inline-block; width:14px; height:3px; vertical-align:middle; margin-right:5px; }
  .legend b { display:inline-block; width:14px; height:10px; vertical-align:middle; margin-right:5px; }
  .frame { position:relative; border:1px solid var(--grid); border-radius:6px; overflow:hidden; }
  .wrap { overflow-x:auto; }
  .ax { position:absolute; top:0; pointer-events:none; }
  .ax.left { left:0; }  .ax.right { right:0; }
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
  <span><i style="background:var(--volts)"></i>battery volts (lower panel)</span>
</div>
<div class="ctl">Zoom: <input id="zoom" type="range" min="20" max="400" value="80"> <span id="zl"></span> px per second</div>
<div class="frame">
  <div class="wrap" id="wrap"><svg id="plot"></svg></div>
  <svg id="axL" class="ax left"></svg>
  <svg id="axR" class="ax right"></svg>
</div>
<div id="tip"></div>
<script>
const R = __DATA__;
// Layout: RPM chart on top (balls on its right axis), battery volts panel below, one shared time axis.
// The value axes are drawn in two small SVGs pinned over the left and right edges of the scrolling
// chart, so they stay visible while scrolling sideways.
const M = {l:52, r:34, t:24};
const H1 = 400;                   // RPM chart bottom
const VT = H1 + 22, VB = VT + 90; // volts panel top / bottom
const H = VB + 30;                // total (time labels below)
const svg = document.getElementById('plot'), tip = document.getElementById('tip');
const axL = document.getElementById('axL'), axR = document.getElementById('axR');
const t0 = R.length ? R[0].t_s : 0, tEnd = R.length ? R[R.length-1].t_s : 1;
let rpmMax = 1000;
for (const r of R) rpmMax = Math.max(rpmMax, r.target_rpm, r.command_rpm, r.actual_rpm);
rpmMax = Math.ceil(rpmMax * 1.08 / 500) * 500;
// Volts: only the measured range (0 = not read yet), padded and rounded to 0.5 V.
const vs = R.map(r => r.volts).filter(v => v > 0);
let vMin = vs.length ? Math.min(...vs) : 11, vMax = vs.length ? Math.max(...vs) : 13;
vMin = Math.floor((vMin - 0.1) * 2) / 2; vMax = Math.ceil((vMax + 0.1) * 2) / 2;
if (vMax - vMin < 1) vMax = vMin + 1;
const vStep = vMax - vMin > 3 ? 1 : 0.5;
let pxs = 80, X;
const Y  = v => M.t + (H1 - M.t) * (1 - v / rpmMax);
const YB = v => M.t + (H1 - M.t) * (1 - v / 4.5);
const YV = v => VT + (VB - VT) * (1 - (v - vMin) / (vMax - vMin));
const NS = 'http://www.w3.org/2000/svg';
function el(n, a) { const e = document.createElementNS(NS, n); for (const k in a) e.setAttribute(k, a[k]); return e; }
function txt(parent, x, y, s, anchor, color) {
  const t = el('text', {x, y, 'text-anchor': anchor || 'start'});
  if (color) t.style.fill = color;
  t.textContent = s; parent.appendChild(t);
}
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
function line(key, color, w, yOf, step, skipZero) {
  let d = '';
  R.forEach(r => {
    if (skipZero && !(r[key] > 0)) return;
    const x = X(r.t_s).toFixed(1), y = yOf(r[key]).toFixed(1);
    if (!d) d = `M${x},${y}`;
    else d += step ? `H${x}V${y}` : `L${x},${y}`;
  });
  if (d) svg.appendChild(el('path', {d, fill: 'none', stroke: color, 'stroke-width': w, 'stroke-linejoin': 'round'}));
}
function draw() {
  const W = Math.max(document.getElementById('wrap').clientWidth - 2, M.l + M.r + (tEnd - t0) * pxs);
  X = t => M.l + (t - t0) * pxs;
  for (const s of [svg, axL, axR]) { s.setAttribute('height', H); s.replaceChildren(); }
  svg.setAttribute('width', W);
  axL.setAttribute('width', M.l); axR.setAttribute('width', M.r);
  const css = getComputedStyle(document.documentElement);
  const c = n => css.getPropertyValue(n).trim();
  // Background panels on the pinned axes, so the chart scrolls underneath them.
  axL.appendChild(el('rect', {x: 0, y: 0, width: M.l, height: H, fill: c('--bg')}));
  axR.appendChild(el('rect', {x: 0, y: 0, width: M.r, height: H, fill: c('--bg')}));

  // RPM chart
  bands('gate_open', M.t, H1 - M.t, c('--gate'));
  bands('boost_on', M.t - 10, 5, c('--boost'));
  bands('ramp_on', M.t - 18, 5, c('--ramp'));
  for (let v = 0; v <= rpmMax; v += 500) {
    svg.appendChild(el('line', {x1: 0, x2: W, y1: Y(v), y2: Y(v), stroke: c('--grid')}));
    txt(axL, M.l - 6, Y(v) + 4, v, 'end');
  }
  txt(axL, 4, M.t - 8, 'rpm');
  for (let b = 0; b <= 4; b++) txt(axR, 6, YB(b) + 4, b, 'start', c('--balls'));
  txt(axR, 2, M.t - 8, 'balls', 'start', c('--balls'));

  // Volts panel (measured range only)
  bands('gate_open', VT, VB - VT, c('--gate'));
  for (let v = vMin; v <= vMax + 1e-9; v += vStep) {
    svg.appendChild(el('line', {x1: 0, x2: W, y1: YV(v), y2: YV(v), stroke: c('--grid')}));
    txt(axL, M.l - 6, YV(v) + 4, v.toFixed(1), 'end');
  }
  txt(axL, 4, VT - 8, 'volts');

  // Time axis
  const step = pxs >= 150 ? 0.5 : pxs >= 60 ? 1 : 5;
  for (let s = Math.ceil(t0 / step) * step; s <= tEnd; s += step) {
    svg.appendChild(el('line', {x1: X(s), x2: X(s), y1: VB, y2: VB + 4, stroke: c('--muted')}));
    txt(svg, X(s), VB + 18, s.toFixed(step < 1 ? 1 : 0) + 's', 'middle');
  }

  line('balls', c('--balls'), 1.5, YB, true);
  line('target_rpm', c('--target'), 1.5, Y, false);
  line('command_rpm', c('--command'), 1.5, Y, false);
  line('actual_rpm', c('--actual'), 2, Y, false);
  line('volts', c('--volts'), 1.5, YV, true, true);

  const cursor = el('line', {y1: M.t, y2: VB, stroke: c('--muted'), 'stroke-dasharray': '3 3', visibility: 'hidden'});
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
      `power ${r.power}  (ff ${r.ff_power}  i ${r.i_power})  battery ${r.volts} V` + '\\n' +
      `dist ${r.dist_in} in  hood ${r.hood_deg}  aim err ${r.aim_err_deg} deg` +
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


def write_html(rows, src, quiet=False):
    keep = ["t_s", "dt_ms", "shooter_mode", "target_rpm", "command_rpm", "actual_rpm", "rpm_err", "ramping",
            "boosting", "at_speed", "ready", "blocked", "intake_mode", "gate_pos", "gate_open", "balls",
            "shoot_held", "dist_in", "hood_deg", "aim_err_deg", "volts", "power", "ff_power", "i_power"]
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
    if not quiet:
        print(f"\nPlot: {out}")


# == Index of every log page in tools/logs =====================================

INDEX_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Shooter Logs</title>
<style>
  :root { --bg:#ffffff; --fg:#1f2328; --muted:#6b7280; --line:#e5e7eb; --link:#2563eb; --hover:#f3f4f6; }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) { --bg:#0f1115; --fg:#e5e7eb; --muted:#9ca3af; --line:#272b33;
      --link:#60a5fa; --hover:#181b21; }
  }
  :root[data-theme="dark"] { --bg:#0f1115; --fg:#e5e7eb; --muted:#9ca3af; --line:#272b33;
      --link:#60a5fa; --hover:#181b21; }
  body { margin:0; padding:16px; background:var(--bg); color:var(--fg); font:14px system-ui,sans-serif; }
  h1 { font-size:18px; margin:0 0 4px; }  .sub { color:var(--muted); margin:0 0 12px; }
  .wrap { overflow-x:auto; }
  table { border-collapse:collapse; min-width:100%; }
  th, td { text-align:left; padding:6px 10px; border-bottom:1px solid var(--line); white-space:nowrap; }
  th { color:var(--muted); font-weight:600; }
  td.num, th.num { text-align:right; font-variant-numeric:tabular-nums; }
  tr:hover td { background:var(--hover); }
  a { color:var(--link); text-decoration:none; }  a:hover { text-decoration:underline; }
</style></head><body>
<h1>Shooter logs</h1>
<p class="sub">__COUNT__ log(s), newest first. Made by tools/shotlog.py.</p>
<div class="wrap"><table>
<thead><tr><th>Date</th><th>Time</th><th>OpMode</th><th class="num">Duration</th><th class="num">Loops</th><th>Page</th></tr></thead>
<tbody>
__ROWS__
</tbody></table></div>
</body></html>
"""

PAGE_INFO = re.compile(r"(\d+) loops over ([\d.]+) s")


def page_info(page):
    """(opmode, datetime or None, duration s or None, loops or None) for a log page."""
    stem = page.stem
    opmode, stamp = (stem.rsplit("_", 1) + [""])[:2] if "_" in stem else (stem, "")
    try:
        when = datetime.strptime(stamp, "%Y%m%d-%H%M%S")
    except ValueError:
        when, opmode = None, stem
    dur = loops = None
    # The page's own subtitle says "<n> loops over <s> s"; it is near the top, so read only the start.
    with open(page, encoding="utf-8", errors="replace") as f:
        m = PAGE_INFO.search(f.read(8000))
    if m:
        loops, dur = int(m.group(1)), float(m.group(2))
    return opmode, when, dur, loops


def fmt_duration(s):
    if s is None:
        return "-"
    m, sec = divmod(int(round(s)), 60)
    return f"{m}:{sec:02d}" if m else f"{sec} s"


def build_index():
    """Make a page for any CSV in tools/logs without one, then rewrite tools/logs/index.html."""
    if not LOGS.is_dir():
        return
    for csv_path in sorted(LOGS.glob("*.csv")):
        if not csv_path.with_suffix(".html").exists():
            rows = read_rows(csv_path)
            if rows:
                write_html(rows, csv_path, quiet=True)
                print(f"Made missing page {csv_path.with_suffix('.html').name}")
    pages = [p for p in LOGS.glob("*.html") if p.name != INDEX.name]
    infos = [(p,) + page_info(p) for p in pages]
    infos.sort(key=lambda i: (i[2] or datetime.min, i[0].name), reverse=True)
    rows = []
    for page, opmode, when, dur, loops in infos:
        rows.append(
            "<tr>"
            f"<td><a href=\"{html.escape(page.name)}\">{when.strftime('%Y-%m-%d') if when else '-'}</a></td>"
            f"<td>{when.strftime('%H:%M:%S') if when else '-'}</td>"
            f"<td>{html.escape(opmode)}</td>"
            f"<td class=\"num\">{fmt_duration(dur)}</td>"
            f"<td class=\"num\">{loops if loops is not None else '-'}</td>"
            f"<td><a href=\"{html.escape(page.name)}\">{html.escape(page.name)}</a></td>"
            "</tr>")
    INDEX.write_text(INDEX_PAGE.replace("__COUNT__", str(len(rows))).replace("__ROWS__", "\n".join(rows)),
                     encoding="utf-8")
    print(f"Index: {INDEX} ({len(rows)} log(s))")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--list", action="store_true", help="list the logs on the hub")
    ap.add_argument("--file", help="pull and plot this log (a name from --list)")
    ap.add_argument("--csv", type=Path, help="plot a local CSV instead of pulling from the hub")
    ap.add_argument("--index", action="store_true",
                    help="only make missing pages and rewrite tools/logs/index.html (no ADB)")
    ap.add_argument("--rebuild", action="store_true",
                    help="remake every page from its CSV in tools/logs, then the index (no ADB)")
    args = ap.parse_args()

    if args.rebuild:
        for csv_path in sorted(LOGS.glob("*.csv")):
            rows = read_rows(csv_path)
            if rows:
                write_html(rows, csv_path, quiet=True)
                print(f"Remade {csv_path.with_suffix('.html').name}")
        build_index()
        return

    if args.index:
        build_index()
        return

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
        if args.file:
            if args.file not in names:
                fail(f"{args.file} is not on the hub (try --list)")
            path = pull(exe, args.file)
        else:
            # Every log we don't have yet (no CSV and no page), plus the newest again: it may have been
            # pulled while its opmode was still running.  Older ones only get their page; the newest
            # is summarised below.
            newest = names[-1]
            wanted = [n for n in names if n == newest
                      or not ((LOGS / n).exists() or (LOGS / n).with_suffix(".html").exists())]
            for n in wanted:
                pulled = pull(exe, n)
                if n != newest:
                    old = read_rows(pulled)
                    if old:
                        write_html(old, pulled, quiet=True)
            have = len(names) - len(wanted)
            if have:
                print(f"({have} older log(s) already in tools/logs)")
            path = LOGS / newest

    rows = read_rows(path)
    summary(rows)
    write_html(rows, path)
    build_index()


if __name__ == "__main__":
    main()
