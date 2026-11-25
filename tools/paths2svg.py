#!/usr/bin/env python3
"""Draw every autonomous path from /turt_files over a 144x144 in field.

Reads the .turt files directly (same grouping as turt2java.py: text before the
last underscore is the group, `_start` is the first path, then 2, 3, ... in
natural order), so the picture never goes stale.  Writes tools/auto_paths.svg.

    python tools/paths2svg.py
"""
import glob
import json
import math
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "tools", "auto_paths.svg")

FIELD = 144          # inches, Pedro coordinates (0,0 = bottom-left, y up)
S = 6                # pixels per inch
ROBOT_LENGTH = 15    # inches, along the heading
ROBOT_WIDTH = 13     # inches, across
PAD = 30
W = FIELD * S + 2 * PAD
LEGEND_W = 290
# One colour per path family (Fred and Fred_NT share it).  The turret version is drawn solid,
# the no-turret (_NT) version dotted.
FAMILY_COLORS = ["#e71d36", "#118ab2", "#ff9f1c", "#2ec4b6", "#9b5de5", "#06d6a0"]
NO_TURRET_TAG = "_NT"
DOTTED = "0.1 9"       # with round caps this draws dots


def family_of(group):
    """Fred_NT -> Fred (as in turt2java.py)."""
    if group.endswith(NO_TURRET_TAG) and len(group) > len(NO_TURRET_TAG):
        return group[:-len(NO_TURRET_TAG)]
    return group


def is_no_turret(group):
    return family_of(group) != group


def dash_attr(group):
    return ' stroke-dasharray="%s"' % DOTTED if is_no_turret(group) else ""


def px(x):
    return PAD + x * S


def py(y):
    return PAD + (FIELD - y) * S


def natural_key(s):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", s)]


def bezier(pts, n=60):
    out = []
    for i in range(n + 1):
        t = i / n
        p = list(pts)
        while len(p) > 1:
            p = [((1 - t) * a[0] + t * b[0], (1 - t) * a[1] + t * b[1])
                 for a, b in zip(p, p[1:])]
        out.append(p[0])
    return out


def load_groups():
    groups = {}
    for f in glob.glob(os.path.join(ROOT, "turt_files", "*.turt")):
        name = os.path.splitext(os.path.basename(f))[0]
        group, _, suffix = name.rpartition("_")
        groups.setdefault(group, {})[suffix] = json.load(open(f))
    return groups


def arrow(x, y, deg, color, size=7):
    """Triangle at field point (x, y) pointing along heading `deg`."""
    a = math.radians(deg)
    pts = []
    for ang, r in ((0, size), (140, size * 0.8), (-140, size * 0.8)):
        b = a + math.radians(ang)
        pts.append((px(x) + r * math.cos(b), py(y) - r * math.sin(b)))
    p = " ".join("%.1f,%.1f" % q for q in pts)
    return ('<polygon points="%s" fill="%s" stroke="#111" stroke-width="1.2"/>'
            % (p, color))


def main():
    groups = load_groups()
    svg = []
    height = FIELD * S + 2 * PAD
    total_w = W + LEGEND_W
    svg.append('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" '
               'width="100%%" font-family="Segoe UI, Arial, sans-serif">'
               % (total_w, height))
    svg.append('<rect width="100%" height="100%" fill="#f4f4f2"/>')

    # field floor: 6x6 grid of 24 in tiles
    svg.append('<rect x="%d" y="%d" width="%d" height="%d" fill="#9aa0a6" '
               'stroke="#222" stroke-width="3"/>' % (PAD, PAD, FIELD * S, FIELD * S))
    for i in range(6):
        for j in range(6):
            shade = "#aeb4ba" if (i + j) % 2 == 0 else "#a2a8ae"
            svg.append('<rect x="%d" y="%d" width="%d" height="%d" fill="%s"/>'
                       % (px(i * 24), py((j + 1) * 24), 24 * S, 24 * S, shade))
    for k in range(7):
        v = k * 24
        svg.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#6b7177" '
                   'stroke-width="1"/>' % (px(v), py(0), px(v), py(FIELD)))
        svg.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#6b7177" '
                   'stroke-width="1"/>' % (px(0), py(v), px(FIELD), py(v)))
        svg.append('<text x="%d" y="%d" font-size="11" fill="#333" '
                   'text-anchor="middle">%d</text>' % (px(v), py(0) + 16, v))
        svg.append('<text x="%d" y="%d" font-size="11" fill="#333" '
                   'text-anchor="end">%d</text>' % (px(0) - 6, py(v) + 4, v))

    # field shapes (goals, hives) from the first file that has them
    shapes = {}
    for g in groups.values():
        for d in g.values():
            for sh in d.get("shapes", []):
                shapes.setdefault(sh["id"], sh)
    for sh in shapes.values():
        if "Goal" in sh["name"]:
            continue  # goals are not drawn; hives only
        pts = " ".join("%.1f,%.1f" % (px(v["x"]), py(v["y"])) for v in sh["vertices"])
        svg.append('<polygon points="%s" fill="%s" fill-opacity="0.75" stroke="%s" '
                   'stroke-width="2"/>' % (pts, sh.get("fillColor", "#ccc"),
                                           sh.get("color", "#333")))
        cx = sum(v["x"] for v in sh["vertices"]) / len(sh["vertices"])
        cy = sum(v["y"] for v in sh["vertices"]) / len(sh["vertices"])
        if "Hive" in sh["name"]:
            svg.append('<text x="%.0f" y="%.0f" font-size="11" font-weight="700" fill="#333" '
                       'text-anchor="middle">Hive</text>' % (px(cx), py(max(v["y"] for v in sh["vertices"])) - 6))
        if "Goal" in sh["name"]:
            svg.append('<text x="%.0f" y="%.0f" font-size="13" font-weight="700" '
                       'fill="#fff" stroke="#000" stroke-width="0.4" '
                       'text-anchor="middle">%s</text>' % (px(cx), py(cy), sh["name"]))

    # paths
    legend = []
    gi = 0
    family_color = {f: FAMILY_COLORS[i % len(FAMILY_COLORS)]
                    for i, f in enumerate(sorted({family_of(g) for g in groups}))}
    group_order = sorted(groups, key=lambda g: (family_of(g), is_no_turret(g)))
    for group in group_order:
        files = groups[group]
        if "start" not in files:
            continue
        order = ["start"] + sorted((k for k in files if k != "start"), key=natural_key)
        dash = dash_attr(group)
        for n, suffix in enumerate(order):
            d = files[suffix]
            color = family_color[family_of(group)]
            sp = d["startPoint"]
            cur = (sp["x"], sp["y"])
            lines = {l["id"]: l for l in d["lines"]}
            seq = [lines[s["lineId"]] for s in d.get("sequence", [])
                   if s.get("kind") == "path" and s["lineId"] in lines] or d["lines"]
            label = "%s_%s" % (group, suffix)
            names = []
            for ln in seq:
                ep = ln["endPoint"]
                ctrl = [(c["x"], c["y"]) for c in ln["controlPoints"]]
                pts = bezier([cur] + ctrl + [(ep["x"], ep["y"])])
                poly = " ".join("%.1f,%.1f" % (px(x), py(y)) for x, y in pts)
                svg.append('<polyline points="%s" fill="none" stroke="#111" '
                           'stroke-width="7" stroke-linecap="round" '
                           'stroke-linejoin="round" opacity="0.35"%s/>' % (poly, dash))
                svg.append('<polyline points="%s" fill="none" stroke="%s" '
                           'stroke-width="4" stroke-linecap="round" '
                           'stroke-linejoin="round"%s/>' % (poly, color, dash))
                # direction chevron mid-path
                mid = len(pts) // 2
                dx = pts[mid + 1][0] - pts[mid - 1][0]
                dy = pts[mid + 1][1] - pts[mid - 1][1]
                svg.append(arrow(pts[mid][0], pts[mid][1],
                                 math.degrees(math.atan2(dy, dx)), color, 6))
                cur = (ep["x"], ep["y"])
                if ln.get("name"):
                    names.append(ln["name"])
                    svg.append('<text x="%.0f" y="%.0f" font-size="12" font-weight="700" '
                               'fill="#111" stroke="#fff" stroke-width="3" '
                               'paint-order="stroke" text-anchor="middle">%s</text>'
                               % (px(ep["x"]), py(ep["y"]) - 12, ln["name"]))
            # end point marker + robot heading
            ep = seq[-1]["endPoint"]
            svg.append('<circle cx="%.1f" cy="%.1f" r="6" fill="#fff" stroke="%s" '
                       'stroke-width="3"/>' % (px(ep["x"]), py(ep["y"]), color))
            # start marker
            svg.append('<circle cx="%.1f" cy="%.1f" r="5" fill="%s" stroke="#111" '
                       'stroke-width="1.5"/>' % (px(sp["x"]), py(sp["y"]), color))
            legend.append((group, label, color, names, d["startPoint"], ep, dash))
        gi += 1

    # start-pose robot footprints (ROBOT_LENGTH x ROBOT_WIDTH) with heading
    for group in sorted(groups):
        d = groups[group].get("start")
        if not d:
            continue
        sp = d["startPoint"]
        deg = sp.get("startDeg", 0)
        half_l, half_w = ROBOT_LENGTH * S / 2, ROBOT_WIDTH * S / 2
        svg.append('<g transform="translate(%.1f %.1f) rotate(%.1f)">'
                   '<rect x="%d" y="%d" width="%d" height="%d" fill="#ffffff" '
                   'fill-opacity="0.35" stroke="#111" stroke-width="1.5" '
                   'stroke-dasharray="5 3"/>'
                   '<line x1="0" y1="0" x2="%d" y2="0" stroke="#111" stroke-width="2"/>'
                   '</g>' % (px(sp["x"]), py(sp["y"]), -deg, -half_l, -half_w,
                             2 * half_l, 2 * half_w, half_l))

    # legend panel
    lx = W + 10
    y = PAD + 6
    svg.append('<text x="%d" y="%d" font-size="20" font-weight="700" fill="#111">'
               'Autonomous paths</text>' % (lx, y + 14))
    svg.append('<text x="%d" y="%d" font-size="12" fill="#555">Pedro field coordinates, '
               '144 x 144 in</text>' % (lx, y + 34))
    y += 62
    last_group = None
    for group, label, color, names, sp, ep, dash in legend:
        if group != last_group:
            y += 10
            svg.append('<text x="%d" y="%d" font-size="15" font-weight="700" '
                       'fill="#111">%s</text>' % (lx, y, group))
            y += 22
            last_group = group
        svg.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" '
                   'stroke-width="5" stroke-linecap="round"%s/>'
                   % (lx, y - 4, lx + 30, y - 4, color, dash))
        svg.append('<text x="%d" y="%d" font-size="13" fill="#111">%s</text>'
                   % (lx + 40, y, label.split("_", 1)[-1] if False else label))
        y += 15
        svg.append('<text x="%d" y="%d" font-size="11" fill="#555">(%g, %g) to (%g, %g)'
                   '%s</text>' % (lx + 40, y, sp["x"], sp["y"], ep["x"], ep["y"],
                                  "  " + ", ".join(names) if names else ""))
        y += 24
    y += 14
    svg.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#555" stroke-width="5" '
               'stroke-linecap="round"/><text x="%d" y="%d" font-size="12" fill="#333">'
               'solid = turret</text>' % (lx, y - 4, lx + 30, y - 4, lx + 40, y))
    y += 20
    svg.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#555" stroke-width="5" '
               'stroke-linecap="round" stroke-dasharray="%s"/><text x="%d" y="%d" '
               'font-size="12" fill="#333">dotted = no turret (_NT)</text>'
               % (lx, y - 4, lx + 30, y - 4, DOTTED, lx + 40, y))
    y += 28
    svg.append('<circle cx="%d" cy="%d" r="5" fill="#bbb" stroke="#111" '
               'stroke-width="1.5"/><text x="%d" y="%d" font-size="12" fill="#333">'
               'path start</text>' % (lx + 6, y - 4, lx + 20, y))
    y += 20
    svg.append('<circle cx="%d" cy="%d" r="6" fill="#fff" stroke="#888" '
               'stroke-width="3"/><text x="%d" y="%d" font-size="12" fill="#333">'
               'path end</text>' % (lx + 6, y - 4, lx + 20, y))
    y += 20
    svg.append('<rect x="%d" y="%d" width="14" height="14" fill="#fff" fill-opacity="0.5" '
               'stroke="#111" stroke-dasharray="4 2"/><text x="%d" y="%d" font-size="12" '
               'fill="#333">robot at start pose (%g x %g in)</text>'
               % (lx, y - 12, lx + 20, y, ROBOT_LENGTH, ROBOT_WIDTH))

    svg.append("</svg>")
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
