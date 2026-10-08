"""Card Vault workcell - dimensioned plan and elevation drawn from scene_config."""

import argparse
from math import cos, sin

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

import scene_config as cfg


# ----------------------------------------------------------------------------
# FOOTPRINTS
# ----------------------------------------------------------------------------
def extents(spec, axes):
    """(centre, half-size) of a shape projected onto the given axes, e.g. 'xy'."""
    i = {"x": 0, "y": 1, "z": 2}
    c = [spec["xyz"][i[a]] for a in axes]
    if spec["type"] == "cuboid":
        h = [spec["size"][i[a]] / 2 for a in axes]
    else:
        r, L = spec["radius"], spec["length"]
        roll = spec.get("rpy", (0, 0, 0))[0]
        along = {0: "z", 1: "y"}[round(abs(sin(roll)))]          # z upright, y if rolled 90
        h = [(L / 2 if a == along else r) for a in axes]
    return c, h


def draw(ax, spec, axes, label=None):
    c, h = extents(spec, axes)
    col = spec["color"]
    face = (col[0], col[1], col[2], min(0.85, col[3]))
    ax.add_patch(Rectangle((c[0] - h[0], c[1] - h[1]), 2 * h[0], 2 * h[1],
                           facecolor=face, edgecolor=(0.1, 0.1, 0.1, 0.8), lw=0.6, zorder=2))
    if label:
        ax.annotate(label, (c[0], c[1]), fontsize=6.5, ha="center", va="center",
                    zorder=5, color="black",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.75))


# ----------------------------------------------------------------------------
# LABEL SET
# ----------------------------------------------------------------------------
LABELLED = {
    "vault_rack": "vault rack",
    "inspection_nest": "inspect nest",
    "transfer_nest": "transfer nest",
    "mailer_box": "mailer",
    "bag_tray": "bags",
    "conveyor_belt": "delivery conveyor",
    "pickup_tray": "pickup tray",
    "chute_sill": "chute",
    "chute_ramp": "chute ramp",
    "curtain_beam": "light curtain",
    "keepout_zone": "keep-out zone",
    "restock_door": "restock door",
    "control_panel": "control panel",
    "panel_estop": "e-stop",
    "estop_staff": "e-stop",
    "display_pose_marker": "display pose",
    "camera_body": "camera",
}
HIDE_IN_PLAN = {"floor_pad", "cabinet_body", "roof", "camera_mast",
                "conveyor_leg_fr", "conveyor_leg_fl", "conveyor_leg_br", "conveyor_leg_bl",
                "stand_post", "stand_base", "chute_jamb_l", "chute_jamb_r", "chute_head"}
OUTSIDE_OK = {"room", "safety", "markers", "conveyor", "panel", "hatch"}


# ----------------------------------------------------------------------------
# FIGURE
# ----------------------------------------------------------------------------
def build(out_path):
    specs = [s for s in cfg.resolved_static() if s["name"] not in HIDE_IN_PLAN] + cfg.card_objects()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))

    # --- top view -----------------------------------------------------------
    for s in specs:
        draw(ax1, s, "xy", LABELLED.get(s["name"]))
    for key, r in cfg.ROBOTS.items():
        x, y, _ = r["base_xyz"]
        ax1.add_patch(Circle((x, y), 0.055, facecolor="#d9534f", ec="k", lw=0.8, zorder=6))
        ax1.annotate(f"{key}\n{r['label'].split('(')[-1].rstrip(')')}", (x, y - 0.10),
                     fontsize=7, ha="center", va="top", zorder=7, fontweight="bold")
    ax1.set_title("Top view (x-y), origin = floor centre of the kiosk", fontsize=10)
    ax1.set_xlabel("x  (m)   +x = right")
    ax1.set_ylabel("y  (m)   +y = staff side / -y = customer side")
    ax1.set_xlim(-1.20, 1.25)
    ax1.set_ylim(-1.05, 1.10)

    # --- front elevation ----------------------------------------------------
    for s in specs:
        draw(ax2, s, "xz", LABELLED.get(s["name"]))
    for key, r in cfg.ROBOTS.items():
        x, _, z = r["base_xyz"]
        h = r["placeholder_h"]
        ax2.add_patch(Rectangle((x - 0.05, z), 0.10, h, facecolor="#d9534f",
                                alpha=0.75, ec="k", lw=0.8, zorder=6))
        ax2.annotate(key, (x, z + h + 0.03), fontsize=7, ha="center", zorder=7, fontweight="bold")
    ax2.axhline(cfg.BENCH_H, ls="--", lw=0.8, color="k", alpha=0.5)
    ax2.annotate(f"bench top z = {cfg.BENCH_H} m", (-1.0, cfg.BENCH_H + 0.02), fontsize=7)
    ax2.set_title("Front elevation (x-z), seen from the customer side", fontsize=10)
    ax2.set_xlabel("x  (m)")
    ax2.set_ylabel("z  (m)")
    ax2.set_xlim(-1.20, 1.25)
    ax2.set_ylim(0, 2.35)

    for ax in (ax1, ax2):
        ax.set_aspect("equal")
        ax.grid(True, lw=0.3, alpha=0.4)
        ax.set_axisbelow(True)

    fig.suptitle("Card Vault workcell - scene layout (generated from scene_config.py)",
                 fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    print("wrote", out_path)


# ----------------------------------------------------------------------------
# CHECKS
# ----------------------------------------------------------------------------
def check():
    bad = []
    lim = (cfg.CELL_W / 2, cfg.CELL_D / 2)
    for s in cfg.resolved_static() + cfg.card_objects():
        if s["group"] in OUTSIDE_OK:
            continue
        c, h = extents(s, "xy")
        if abs(c[0]) + h[0] > lim[0] + 1e-6 or abs(c[1]) + h[1] > lim[1] + 1e-6:
            bad.append(s["name"])
    for key, r in cfg.ROBOTS.items():
        x, y, _ = r["base_xyz"]
        if abs(x) > lim[0] or abs(y) > lim[1]:
            bad.append(key)
    print("outside the enclosure footprint:", bad if bad else "none")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default="card_vault_plan.png")
    a = ap.parse_args()
    check()
    build(a.out)
