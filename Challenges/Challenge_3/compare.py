"""
compare.py - Section 13: compare the IK methods on IDENTICAL episode seeds.

    python compare.py --run          # run every experiment, then make plots
    python compare.py                # only re-make plots from existing logs

Experiments (all use seeds 0..N-1, so every method sees the same cubes):
  nominal : cube in the designed spawn region, smart choices ON
  stress  : cube ANYWHERE on the table,       smart choices ON
  naive   : cube anywhere, smart choices OFF  (--naive) -> exposes failures

Makes results/summary.md, results/summary.csv and results/plots/cmp_*.png
"""
import argparse
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

METHODS = ["mink", "pinv", "dls", "qp"]
NAMES = {"mink": "Mink (off-the-shelf)", "pinv": "Pseudo-inverse (own choice)",
         "dls": "DLS-IK", "qp": "QP-IK"}
EXPS = {"nominal": ("", []), "stress": ("_stress", ["--region", "full"]),
        "naive": ("_naive", ["--region", "full", "--naive"])}
COL = {"mink": "#1f9bd1", "pinv": "#c94fa3", "dls": "#f08a24", "qp": "#5aa832"}
os.makedirs("results/plots", exist_ok=True)


def run_all(n):
    for exp, (tag, extra) in EXPS.items():
        for m in METHODS:
            print(f"--- {exp} / {m}")
            cmd = [sys.executable, "pick_place.py", "--method", m, "--episodes", str(n),
                   "--tag", tag] + extra
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)


def wilson(k, n, z=1.96):
    """95% confidence interval of a success rate (Wilson score)."""
    if n == 0:
        return 0, 0
    p = k / n
    den = 1 + z**2 / n
    c = (p + z**2 / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z**2 / (4 * n**2)) / den
    return max(0, c - h), min(1, c + h)


def category(reason):
    if not isinstance(reason, str) or reason == "":
        return "success"
    if reason.startswith("collision"):
        return "robot_collision"
    return reason.split(" (")[0].split(" @")[0]


def load():
    rows = []
    for exp, (tag, _) in EXPS.items():
        for m in METHODS:
            f = f"results/logs_{m}{tag}.csv"
            if os.path.exists(f):
                d = pd.read_csv(f)
                d["experiment"], d["method"] = exp, m
                rows.append(d)
    d = pd.concat(rows, ignore_index=True)
    d["category"] = d["failure_reason"].apply(category)
    return d


def summarise(d):
    out = []
    for (exp, m), g in d.groupby(["experiment", "method"], sort=False):
        k, n = int(g.success.sum()), len(g)
        lo, hi = wilson(k, n)
        out.append(dict(
            experiment=exp, method=m, episodes=n, success=k,
            success_rate=round(k / n, 3), ci95=f"{lo:.2f}-{hi:.2f}",
            plan_time_ms=round(g.plan_time_s.mean() * 1000, 1),
            plan_time_std=round(g.plan_time_s.std() * 1000, 1),
            ik_iters_per_waypoint=round(g.ik_iters_mean.mean(), 2),
            ik_iters_max=int(g.ik_iters_max.max()),
            min_clear_robot_mm=round(g.min_clear_robot.min() * 1000, 1),
            min_clear_cube_tray_mm=round(pd.to_numeric(g.min_clear_cube_tray,
                                                       errors="coerce").min() * 1000, 1),
            cube_tray_contacts=int(g.cube_tray_contact.sum()),
            joint_limit_viol=int(g.joint_limit_viol.sum()),
            vel_viol=int(g.vel_viol.sum()),
            max_vel_ratio=round(g.max_vel_ratio.max(), 2),
            main_failure=g.loc[~g.success, "category"].mode().iat[0] if (~g.success).any() else "-",
        ))
    return pd.DataFrame(out)


def plots(d, s):
    # 1) success rate per experiment
    fig, ax = plt.subplots(figsize=(8, 4))
    w = 0.2
    for i, m in enumerate(METHODS):
        ss = s[s.method == m].set_index("experiment").reindex(EXPS)
        err = [[r - float(c.split("-")[0]) for r, c in zip(ss.success_rate, ss.ci95)],
               [float(c.split("-")[1]) - r for r, c in zip(ss.success_rate, ss.ci95)]]
        ax.bar(np.arange(3) + (i - 1.5) * w, ss.success_rate * 100, w, yerr=np.array(err) * 100,
               capsize=3, label=NAMES[m], color=COL[m])
    ax.set_xticks(range(3), ["nominal\n(designed region)", "stress\n(whole table)",
                             "naive\n(smart choices OFF)"])
    ax.set(ylabel="success rate [%]", ylim=(0, 110),
           title="Pick-and-place success (25 episodes each, same seeds, 95% CI)")
    ax.legend(fontsize=8, loc="lower left")
    fig.tight_layout()
    fig.savefig("results/plots/cmp_success.png", dpi=130)
    plt.close(fig)

    # 2) IK iterations & planning time (nominal)
    nom = d[d.experiment == "nominal"]
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    for ax, col, lab in [(axs[0], "ik_iters_mean", "IK iterations per waypoint"),
                         (axs[1], "plan_time_s", "planning time per episode [s]")]:
        data = [nom[nom.method == m][col] for m in METHODS]
        bp = ax.boxplot(data, patch_artist=True)
        for p, m in zip(bp["boxes"], METHODS):
            p.set_facecolor(COL[m])
        ax.set_xticks(range(1, 5), [NAMES[m].split(" (")[0] for m in METHODS])
        ax.set(title=lab + " (nominal)")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("results/plots/cmp_iters_time.png", dpi=130)
    plt.close(fig)

    # 3) failure categories (naive)
    nv = d[d.experiment == "naive"]
    cats = sorted(nv.category.unique(), key=lambda c: (c != "success", c))
    cc = {"success": "#4caf50", "ik_infeasible": "#e53935", "cube_tray_collision": "#fb8c00",
          "dropped_in_transit": "#fdd835", "robot_collision": "#8e24aa", "place_miss": "#6d4c41",
          "grasp_slip": "#00897b", "grasp_miss": "#3949ab", "planned_collision": "#d81b60"}
    fig, ax = plt.subplots(figsize=(8, 4))
    bottom = np.zeros(len(METHODS))
    for c in cats:
        v = np.array([(nv[nv.method == m].category == c).sum() for m in METHODS])
        ax.bar(range(4), v, bottom=bottom, label=c, color=cc.get(c, "grey"))
        bottom += v
    ax.set_xticks(range(4), [NAMES[m].split(" (")[0] for m in METHODS])
    ax.set(ylabel="episodes", title="Outcome breakdown - naive experiment (smart choices OFF)")
    ax.legend(fontsize=8, bbox_to_anchor=(1.01, 1), loc="upper left")
    fig.tight_layout()
    fig.savefig("results/plots/cmp_failures.png", dpi=130)
    plt.close(fig)

    # 4) constraint violations + clearances
    fig, axs = plt.subplots(1, 3, figsize=(13, 4))
    x = np.arange(4)
    for i, exp in enumerate(["nominal", "naive"]):
        ss = s[s.experiment == exp].set_index("method").reindex(METHODS)
        axs[0].bar(x + (i - 0.5) * 0.38, ss.joint_limit_viol, 0.38, label=exp,
                   color=["#90caf9", "#e57373"][i])
        axs[1].bar(x + (i - 0.5) * 0.38, ss.max_vel_ratio, 0.38, label=exp,
                   color=["#90caf9", "#e57373"][i])
        axs[2].bar(x + (i - 0.5) * 0.38, ss.min_clear_robot_mm, 0.38, label=exp,
                   color=["#90caf9", "#e57373"][i])
    axs[1].axhline(1.0, color="k", ls="--", lw=1)
    for ax, t in zip(axs, ["joint-limit violations (waypoints)",
                           "max joint speed / speed limit",
                           "min gripper clearance to table/tray [mm]"]):
        ax.set_xticks(x, [NAMES[m].split(" (")[0] for m in METHODS])
        ax.set_title(t, fontsize=10)
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("results/plots/cmp_constraints.png", dpi=130)
    plt.close(fig)

    # 5) where does each method succeed/fail on the table (naive)
    fig, axs = plt.subplots(1, 4, figsize=(15, 4), sharey=True)
    for ax, m in zip(axs, METHODS):
        g = nv[nv.method == m]
        ax.scatter(g.cube_y[g.success], g.cube_x[g.success], c="g", marker="o", label="success")
        ax.scatter(g.cube_y[~g.success], g.cube_x[~g.success], c="r", marker="x", label="fail")
        ax.set(title=NAMES[m].split(" (")[0], xlabel="cube y [m]")
        ax.invert_xaxis()
        ax.grid(alpha=0.3)
    axs[0].set_ylabel("cube x [m]")
    axs[0].legend()
    fig.suptitle("Naive experiment: initial cube positions (top view)")
    fig.tight_layout()
    fig.savefig("results/plots/cmp_positions.png", dpi=130)
    plt.close(fig)


def write_md(s):
    cols = ["method", "success", "episodes", "success_rate", "ci95", "plan_time_ms",
            "ik_iters_per_waypoint", "ik_iters_max", "min_clear_robot_mm",
            "min_clear_cube_tray_mm", "cube_tray_contacts", "joint_limit_viol",
            "vel_viol", "max_vel_ratio", "main_failure"]
    with open("results/summary.md", "w", encoding="utf-8") as f:
        f.write("# Summary metrics (generated by compare.py)\n\n")
        for exp in EXPS:
            f.write(f"## {exp}\n\n")
            t = s[s.experiment == exp][cols]
            f.write("| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n")
            for _, r in t.iterrows():
                f.write("| " + " | ".join(str(r[c]) for c in cols) + " |\n")
            f.write("\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--episodes", type=int, default=25)
    a = ap.parse_args()
    if a.run:
        run_all(a.episodes)
    d = load()
    s = summarise(d)
    s.to_csv("results/summary.csv", index=False)
    write_md(s)
    plots(d, s)
    print(s[["experiment", "method", "success_rate", "plan_time_ms", "ik_iters_per_waypoint",
             "joint_limit_viol", "vel_viol", "max_vel_ratio", "cube_tray_contacts",
             "main_failure"]].to_string(index=False))