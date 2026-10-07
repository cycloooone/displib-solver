"""
viz.py - visualise a DISPLIB MILP solution.

Usage:
    python viz.py instance.json solution.json [--out viz_out] [--trains 0,3,10]

solution.json format (written by your milp.py):
{
  "status": "OPTIMAL",                    # optional
  "objective": 12.0,                      # optional
  "x": [[train, op, t], ...],             # every op with x > 0.5, with its t
  "y": [[train, a, b], ...]               # every edge with y > 0.5
}

Output (in --out):
  routes.png     per-train operation graph; used edges highlighted
  timeline.png   per-train route over time (time-space style)
  resources.png  resource occupation per resource; conflicts in red
  report.txt     text summary: walked routes, BROKEN, ghost ops, window
                 violations, resource conflicts
"""
import argparse
import json
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

INF = float("inf")


# ---------------------------------------------------------------- loading
def load_instance(path):
    with open(path) as f:
        raw = json.load(f)
    trains = []
    for tr in raw["trains"]:
        ops = []
        for op in tr:
            ops.append({
                "lb": op.get("start_lb", 0),
                "ub": op.get("start_ub", INF),
                "d": op.get("min_duration", 0),
                "res": {r["resource"]: r.get("release_time", 0)
                        for r in op.get("resources", [])},
                "sucs": op.get("successors", []),
            })
        trains.append(ops)
    objs = []
    for o in raw.get("objective", []):
        objs.append({
            "tr": o["train"], "op": o["operation"],
            "threshold": o.get("threshold", 0),
            "coeff": o.get("coeff", 0),
            "increment": o.get("increment", 0),
        })
    return trains, objs


def load_solution(path, n_trains):
    with open(path) as f:
        raw = json.load(f)
    xs = [dict() for _ in range(n_trains)]          # op -> t
    ys = [set() for _ in range(n_trains)]           # (a, b)
    for tr, op, t in raw.get("x", []):
        xs[int(tr)][int(op)] = float(t)
    for tr, a, b in raw.get("y", []):
        ys[int(tr)].add((int(a), int(b)))
    return xs, ys, raw.get("objective"), raw.get("status")


# ---------------------------------------------------------------- analysis
def walk_route(ops, used_edges):
    """Follow used edges from op 0. Returns (route, status)."""
    out = defaultdict(list)
    for a, b in used_edges:
        out[a].append(b)
    route, cur, seen = [0], 0, {0}
    while True:
        nxt = out.get(cur, [])
        if len(ops[cur]["sucs"]) == 0:
            return route, "OK"
        if len(nxt) == 0:
            return route, "BROKEN (stuck at op %d)" % cur
        if len(nxt) > 1:
            return route, "BRANCH (op %d has %d used out-edges)" % (cur, len(nxt))
        cur = nxt[0]
        if cur in seen:
            return route, "CYCLE at op %d" % cur
        seen.add(cur)
        route.append(cur)


def depth_layout(ops):
    """x = longest-path depth from op 0, y = spread within each depth."""
    n = len(ops)
    indeg = [0] * n
    for a in range(n):
        for b in ops[a]["sucs"]:
            indeg[b] += 1
    depth = [0] * n
    order = [a for a in range(n) if indeg[a] == 0]
    i = 0
    while i < len(order):
        a = order[i]
        i += 1
        for b in ops[a]["sucs"]:
            depth[b] = max(depth[b], depth[a] + 1)
            indeg[b] -= 1
            if indeg[b] == 0:
                order.append(b)
    cols = defaultdict(list)
    for a in range(n):
        cols[depth[a]].append(a)
    pos = {}
    for dpt, members in cols.items():
        k = len(members)
        for j, a in enumerate(members):
            pos[a] = (dpt, (j - (k - 1) / 2.0))
    return pos


def resource_intervals(trains, xs, routes):
    """Occupation [start, end + release) of each resource on walked routes.
    end of op = start of next op on the route. Last op: start + min_duration."""
    occ = defaultdict(list)   # res -> [(start, end_incl_release, tr, op)]
    for tr, route in enumerate(routes):
        for k, op in enumerate(route):
            if op not in xs[tr]:
                continue
            s = xs[tr][op]
            if k + 1 < len(route) and route[k + 1] in xs[tr]:
                e = xs[tr][route[k + 1]]
            else:
                e = s + trains[tr][op]["d"]
            for r, rel in trains[tr][op]["res"].items():
                occ[r].append((s, e + rel, tr, op))
    return occ


def find_conflicts(occ):
    conflicts = []
    for r, ivs in occ.items():
        ivs = sorted(ivs)
        for i in range(len(ivs)):
            for j in range(i + 1, len(ivs)):
                s1, e1, t1, o1 = ivs[i]
                s2, e2, t2, o2 = ivs[j]
                if s2 >= e1:
                    break
                if t1 != t2:
                    conflicts.append((r, ivs[i], ivs[j]))
    return conflicts


# ---------------------------------------------------------------- plots
def plot_routes(trains, xs, ys, routes, statuses, show, path):
    n = len(show)
    ncol = 1 if n == 1 else 2
    nrow = math.ceil(n / ncol)
    fig, axes = plt.subplots(nrow, ncol, figsize=(9 * ncol, 3.2 * nrow),
                             squeeze=False)
    for idx, tr in enumerate(show):
        ax = axes[idx // ncol][idx % ncol]
        ops = trains[tr]
        pos = depth_layout(ops)
        on_route = set(routes[tr])
        touched = {a for e in ys[tr] for a in e}
        # unused edges
        for a in range(len(ops)):
            for b in ops[a]["sucs"]:
                if (a, b) not in ys[tr]:
                    (x1, y1), (x2, y2) = pos[a], pos[b]
                    ax.plot([x1, x2], [y1, y2], color="#d0d0d0", lw=0.8,
                            zorder=1)
        # used edges
        for a, b in ys[tr]:
            (x1, y1), (x2, y2) = pos[a], pos[b]
            col = "#1f6fd1" if (a in on_route and b in on_route) else "#e08a00"
            ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                        arrowprops=dict(arrowstyle="-|>", color=col, lw=2),
                        zorder=2)
        # nodes
        for a in range(len(ops)):
            x, y = pos[a]
            if a in xs[tr] and a not in touched and a != 0:
                fc, ec = "#ffdddd", "#d11f1f"      # ghost op
            elif a in xs[tr]:
                fc, ec = "#d8e8ff", "#1f6fd1"
            else:
                fc, ec = "white", "#b0b0b0"
            ax.scatter([x], [y], s=150, facecolor=fc, edgecolor=ec, zorder=3)
            if len(ops) <= 120:
                ax.text(x, y, str(a), ha="center", va="center", fontsize=6,
                        zorder=4)
        ax.set_title("train %d   %s   route len %d" %
                     (tr, statuses[tr], len(routes[tr])), fontsize=9,
                     color="black" if statuses[tr] == "OK" else "#d11f1f")
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(False)
    for k in range(n, nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")
    fig.suptitle("Routes: blue = walked route, orange = used edge off the "
                 "walked route, red node = ghost op (x=1, no used edge)",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_timeline(trains, objs, xs, routes, show, path):
    fig, ax = plt.subplots(figsize=(13, 0.55 * len(show) + 1.8))
    cmap = plt.get_cmap("tab20")
    tmax = 1
    for row, tr in enumerate(show):
        route = [op for op in routes[tr] if op in xs[tr]]
        for k, op in enumerate(route):
            s = xs[tr][op]
            if k + 1 < len(route):
                e = xs[tr][route[k + 1]]
            else:
                e = s + trains[tr][op]["d"]
            w = max(e - s, 0.0)
            lb, ub = trains[tr][op]["lb"], trains[tr][op]["ub"]
            bad = s < lb - 1e-6 or s > ub + 1e-6
            ax.add_patch(Rectangle((s, row - 0.35), w if w > 0 else 0.001,
                                   0.7, facecolor=cmap(k % 20),
                                   edgecolor="#d11f1f" if bad else "black",
                                   lw=2 if bad else 0.4))
            if w == 0:
                ax.plot([s], [row], marker="|", color="black", ms=14)
            ax.text(s + w / 2, row, str(op), ha="center", va="center",
                    fontsize=6)
            tmax = max(tmax, e, s)
        for o in objs:
            if o["tr"] == tr:
                ax.plot([o["threshold"]], [row + 0.42], marker="v",
                        color="#7a1fd1", ms=6)
    ax.set_yticks(range(len(show)))
    ax.set_yticklabels(["train %d" % t for t in show])
    ax.set_xlim(0, tmax * 1.03 + 1)
    ax.set_ylim(-0.7, len(show) - 0.3)
    ax.invert_yaxis()
    ax.set_xlabel("time")
    ax.set_title("Timeline: box = op on walked route (label = op id), "
                 "red border = start outside [start_lb, start_ub], "
                 "purple ▼ = objective threshold", fontsize=9)
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_resources(occ, conflicts, path, max_res=60):
    res = sorted(occ.keys(), key=lambda r: min(s for s, *_ in occ[r]))
    if not res:
        return False
    res = res[:max_res]
    bad = {(r, iv) for r, a, b in conflicts for iv in (a, b)}
    cmap = plt.get_cmap("tab20")
    fig, ax = plt.subplots(figsize=(13, 0.32 * len(res) + 1.8))
    tmax = 1
    for row, r in enumerate(res):
        for iv in occ[r]:
            s, e, tr, op = iv
            ax.add_patch(Rectangle((s, row - 0.35), max(e - s, 0.001), 0.7,
                                   facecolor=cmap(tr % 20), alpha=0.75,
                                   edgecolor="#d11f1f" if (r, iv) in bad
                                   else "black",
                                   lw=2 if (r, iv) in bad else 0.4,
                                   hatch="///" if (r, iv) in bad else None))
            ax.text((s + e) / 2, row, "T%d" % tr, ha="center", va="center",
                    fontsize=6)
            tmax = max(tmax, e)
    ax.set_yticks(range(len(res)))
    ax.set_yticklabels([str(r) for r in res], fontsize=7)
    ax.set_xlim(0, tmax * 1.03 + 1)
    ax.set_ylim(-0.7, len(res) - 0.3)
    ax.invert_yaxis()
    ax.set_xlabel("time")
    ax.set_title("Resources: bar = occupation [op start, next op start + "
                 "release_time), colour = train, red hatch = conflict "
                 "between different trains", fontsize=9)
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instance")
    ap.add_argument("solution")
    ap.add_argument("--out", default="viz_out")
    ap.add_argument("--trains", default=None,
                    help="comma-separated train ids to plot (default all)")
    args = ap.parse_args()

    trains, objs = load_instance(args.instance)
    xs, ys, obj_val, status = load_solution(args.solution, len(trains))
    os.makedirs(args.out, exist_ok=True)

    show = (list(range(len(trains))) if args.trains is None
            else [int(s) for s in args.trains.split(",")])

    routes, statuses = [], []
    for tr in range(len(trains)):
        r, st = walk_route(trains[tr], ys[tr])
        routes.append(r)
        statuses.append(st)

    occ = resource_intervals(trains, xs, routes)
    conflicts = find_conflicts(occ)

    lines = ["status: %s   objective: %s" % (status, obj_val), ""]
    for tr in range(len(trains)):
        touched = {a for e in ys[tr] for a in e}
        ghosts = sorted(a for a in xs[tr] if a not in touched and a != 0)
        off_route = sorted(e for e in ys[tr]
                           if not (e[0] in routes[tr] and e[1] in routes[tr]))
        viol = [(op, xs[tr][op], trains[tr][op]["lb"], trains[tr][op]["ub"])
                for op in routes[tr] if op in xs[tr] and
                (xs[tr][op] < trains[tr][op]["lb"] - 1e-6 or
                 xs[tr][op] > trains[tr][op]["ub"] + 1e-6)]
        lines.append("train %d: %s" % (tr, statuses[tr]))
        lines.append("  route: " + " -> ".join(
            "%d(t=%g)" % (op, xs[tr].get(op, float("nan")))
            for op in routes[tr]))
        if ghosts:
            lines.append("  ghost ops (x=1, no used edge): %s" % ghosts)
        if off_route:
            lines.append("  used edges off walked route: %s" % off_route)
        if viol:
            lines.append("  time-window violations (op, t, lb, ub): %s" % viol)
    lines.append("")
    lines.append("objective components (recomputed from t):")
    total = 0.0
    for k, o in enumerate(objs):
        t = xs[o["tr"]].get(o["op"])
        if t is None:
            lines.append("  k=%d train %d op %d: op not used" %
                         (k, o["tr"], o["op"]))
            continue
        pen = o["coeff"] * max(0.0, t - o["threshold"]) + \
            (o["increment"] if t > o["threshold"] else 0)
        total += pen
        lines.append("  k=%d train %d op %d: t=%g thr=%g coeff=%g -> %g" %
                     (k, o["tr"], o["op"], t, o["threshold"], o["coeff"], pen))
    lines.append("  total = %g" % total)
    lines.append("")
    lines.append("resource conflicts between different trains: %d" %
                 len(conflicts))
    for r, a, b in conflicts[:50]:
        lines.append("  %s: T%d op%d [%g,%g)  vs  T%d op%d [%g,%g)" %
                     (r, a[2], a[3], a[0], a[1], b[2], b[3], b[0], b[1]))

    report = "\n".join(lines)
    with open(os.path.join(args.out, "report.txt"), "w") as f:
        f.write(report + "\n")
    print(report)

    plot_routes(trains, xs, ys, routes, statuses, show,
                os.path.join(args.out, "routes.png"))
    plot_timeline(trains, objs, xs, routes, show,
                  os.path.join(args.out, "timeline.png"))
    plot_resources(occ, conflicts, os.path.join(args.out, "resources.png"))
    print("\nwrote %s/{routes,timeline,resources}.png and report.txt"
          % args.out)


if __name__ == "__main__":
    main()