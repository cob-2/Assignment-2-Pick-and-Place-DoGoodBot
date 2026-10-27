from math import pi, radians

import numpy as np

# ----------------------------------------------------------------------------
# FRAME CONVENTION
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# MASTER DIMENSIONS
# ----------------------------------------------------------------------------
CELL_W = 1.60
CELL_D = 1.20
FRONT_Y = -0.50
BACK_Y = FRONT_Y + CELL_D
CELL_YC = (FRONT_Y + BACK_Y) / 2
CASE_X1 = CELL_W / 2 - 0.10
CASE_X2 = CELL_W / 2 + 0.40      # outer right wall; the Kawasaki swings back to x~1.07 when it turns round
BENCH_H = 0.90
GLASS_H = 1.20
PANEL_T = 0.012

CONV_Y = -0.78
CONV_TOP = 0.95

# The glass case, as a box. Nothing outside it can be touched by an arm.
ENCLOSURE = dict(x0=-CELL_W / 2, x1=CASE_X2,
                 y0=FRONT_Y, y1=FRONT_Y + CELL_D,
                 z0=BENCH_H - 0.05, z1=BENCH_H + GLASS_H)

# Infeed spur: a short belt that starts inside the case, runs out through the
# front hatch and ends against the side of the main conveyor. The UR3e sets the
# boxed card down on its inside end; no ramp.
HATCH = dict(x=-0.35, w=0.23, h=0.26)          # opening centre x, clear width, clear height
SPUR = dict(x=HATCH["x"], y_in=-0.36, y_out=CONV_Y + 0.10, w=0.20, belt_w=0.17)
SPUR_DROP_XY = (SPUR["x"], -0.40)             # where the UR3e puts the box down

# ----------------------------------------------------------------------------
# COLOURS
# ----------------------------------------------------------------------------
C = {
    "floor":     (0.25, 0.25, 0.28, 1.0),
    "table":     (0.35, 0.37, 0.42, 1.0),
    "cabinet":   (0.20, 0.22, 0.26, 1.0),
    "glass":     (0.55, 0.75, 0.85, 0.22),
    "frame":     (0.15, 0.16, 0.18, 1.0),
    "door":      (0.30, 0.45, 0.55, 0.35),
    "rack":      (0.45, 0.33, 0.22, 1.0),
    "card":      (0.90, 0.80, 0.25, 1.0),
    "nest":      (0.15, 0.55, 0.55, 1.0),
    "camera":    (0.10, 0.10, 0.12, 1.0),
    "conveyor":  (0.30, 0.32, 0.35, 1.0),
    "belt":      (0.10, 0.10, 0.11, 1.0),
    "mailer":    (0.75, 0.60, 0.40, 1.0),
    "panel":     (0.22, 0.26, 0.32, 1.0),
    "screen":    (0.20, 0.65, 0.75, 1.0),
    "stop":      (0.85, 0.15, 0.12, 1.0),
    "go":        (0.15, 0.70, 0.35, 1.0),
    "warn":      (0.95, 0.70, 0.10, 1.0),
    "beam":      (0.95, 0.20, 0.20, 0.45),
    "keepout":   (0.95, 0.75, 0.10, 0.35),
    "target":    (0.20, 0.85, 0.80, 0.45),
    "pedestal":  (0.22, 0.24, 0.28, 1.0),
    "tape":      (0.98, 0.82, 0.08, 1.0),
}

# ----------------------------------------------------------------------------
# ARM A ROBOT BAY
# ----------------------------------------------------------------------------
ARM_A_BASE = (CASE_X1 + 0.10, 0.15, BENCH_H)
PORT_MARGIN = 0.05
ARM_PORT = dict(
    y0=FRONT_Y + 1.5 * PANEL_T + PORT_MARGIN,
    y1=BACK_Y - 1.5 * PANEL_T - PORT_MARGIN,
    h=GLASS_H - PANEL_T / 2 - PORT_MARGIN,
)

# ----------------------------------------------------------------------------
# INSPECTION STATION
# ----------------------------------------------------------------------------
INSPECT_XY = (-0.05, -0.05)
CAM = dict(
    standoff=0.160,
    mesh="siyi_a8_mini.stl",
    body=(0.0520, 0.0606, 0.0840),  # mesh bounding box, in the mesh's own frame
    lens_z=0.0690,                 # base plate -> lens centre
    lens_out=0.0260,               # base plate -> front face, along the view axis
)


# ----------------------------------------------------------------------------
# STATIC OBJECTS
# ----------------------------------------------------------------------------

STATIC = [
    dict(name="floor_pad", group="room", type="cuboid",
         size=(2.6, 2.8, 0.02), xyz=(0, -0.1, 0.01), color=C["floor"]),

    dict(name="table_top", group="table", type="cuboid",
         size=(CASE_X2 + CELL_W / 2, CELL_D, 0.05),
         xyz=((CASE_X2 - CELL_W / 2) / 2, CELL_YC, BENCH_H - 0.025), color=C["table"]),
    dict(name="cabinet_body", group="table", type="cuboid",
         size=(CASE_X2 + CELL_W / 2 - 0.08, CELL_D - 0.08, BENCH_H - 0.05),
         xyz=((CASE_X2 - CELL_W / 2) / 2, CELL_YC, (BENCH_H - 0.05) / 2),
         color=C["cabinet"]),

    # front glass in three panes around the hatch the spur belt runs through
    dict(name="glass_front", group="enclosure", type="cuboid",
         size=(HATCH["x"] - HATCH["w"] / 2 + CELL_W / 2, PANEL_T, GLASS_H),
         xyz=((HATCH["x"] - HATCH["w"] / 2 - CELL_W / 2) / 2, FRONT_Y + PANEL_T, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="glass_front_r", group="enclosure", type="cuboid",
         size=(CASE_X2 - HATCH["x"] - HATCH["w"] / 2, PANEL_T, GLASS_H),
         xyz=((CASE_X2 + HATCH["x"] + HATCH["w"] / 2) / 2, FRONT_Y + PANEL_T, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="glass_front_top", group="enclosure", type="cuboid",
         size=(HATCH["w"], PANEL_T, GLASS_H - HATCH["h"]),
         xyz=(HATCH["x"], FRONT_Y + PANEL_T, BENCH_H + (HATCH["h"] + GLASS_H) / 2),
         color=C["glass"]),
    dict(name="glass_left", group="enclosure", type="cuboid",
         size=(PANEL_T, CELL_D, GLASS_H), xyz=(-CELL_W / 2 + PANEL_T, CELL_YC, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="glass_right_front", group="enclosure", type="cuboid",
         size=(PANEL_T, ARM_PORT["y0"] - FRONT_Y, GLASS_H),
         xyz=(CASE_X1 - PANEL_T, (FRONT_Y + ARM_PORT["y0"]) / 2, BENCH_H + GLASS_H / 2), color=C["glass"]),
    dict(name="glass_right_back", group="enclosure", type="cuboid",
         size=(PANEL_T, BACK_Y - ARM_PORT["y1"], GLASS_H),
         xyz=(CASE_X1 - PANEL_T, (ARM_PORT["y1"] + BACK_Y) / 2, BENCH_H + GLASS_H / 2), color=C["glass"]),
    dict(name="glass_right_top", group="enclosure", type="cuboid",
         size=(PANEL_T, ARM_PORT["y1"] - ARM_PORT["y0"], GLASS_H - ARM_PORT["h"]),
         xyz=(CASE_X1 - PANEL_T, (ARM_PORT["y0"] + ARM_PORT["y1"]) / 2,
              BENCH_H + (ARM_PORT["h"] + GLASS_H) / 2), color=C["glass"]),
    dict(name="glass_back", group="enclosure", type="cuboid",
         size=(CASE_X2 + CELL_W / 2, PANEL_T, GLASS_H),
         xyz=((CASE_X2 - CELL_W / 2) / 2, BACK_Y - PANEL_T, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="glass_right_outer", group="enclosure", type="cuboid",
         size=(PANEL_T, CELL_D, GLASS_H),
         xyz=(CASE_X2 - PANEL_T, CELL_YC, BENCH_H + GLASS_H / 2), color=C["glass"]),
    dict(name="roof", group="enclosure", type="cuboid",
         size=(CASE_X2 + CELL_W / 2, CELL_D, PANEL_T),
         xyz=((CASE_X2 - CELL_W / 2) / 2, CELL_YC, BENCH_H + GLASS_H), color=C["frame"]),


    dict(name="camera_body", group="inspect", type="mesh", filename=CAM["mesh"],
         size=CAM["body"], rpy=(0, 0, -pi / 2),
         xyz=(INSPECT_XY[0], INSPECT_XY[1] - CAM["standoff"], BENCH_H),
         box_xyz=(INSPECT_XY[0], INSPECT_XY[1] - CAM["standoff"],
                  BENCH_H + CAM["body"][2] / 2),
         color=C["camera"]),


    dict(name="hatch_head", group="hatch", type="cuboid",
         size=(HATCH["w"] + 0.032, 0.03, 0.016),
         xyz=(HATCH["x"], FRONT_Y + PANEL_T, BENCH_H + HATCH["h"] + 0.008), color=C["frame"]),
    dict(name="hatch_jamb_l", group="hatch", type="cuboid",
         size=(0.016, 0.03, HATCH["h"]),
         xyz=(HATCH["x"] - HATCH["w"] / 2 - 0.008, FRONT_Y + PANEL_T, BENCH_H + HATCH["h"] / 2),
         color=C["frame"]),
    dict(name="hatch_jamb_r", group="hatch", type="cuboid",
         size=(0.016, 0.03, HATCH["h"]),
         xyz=(HATCH["x"] + HATCH["w"] / 2 + 0.008, FRONT_Y + PANEL_T, BENCH_H + HATCH["h"] / 2),
         color=C["frame"]),

    dict(name="spur_frame", group="conveyor", type="cuboid",
         size=(SPUR["w"], SPUR["y_in"] - SPUR["y_out"], 0.04),
         xyz=(SPUR["x"], (SPUR["y_in"] + SPUR["y_out"]) / 2, CONV_TOP - 0.02), color=C["conveyor"]),
    dict(name="spur_belt", group="conveyor", type="cuboid",
         size=(SPUR["belt_w"], SPUR["y_in"] - SPUR["y_out"] - 0.01, 0.012),
         xyz=(SPUR["x"], (SPUR["y_in"] + SPUR["y_out"]) / 2, CONV_TOP + 0.006), color=C["belt"]),
    dict(name="spur_leg_l", group="conveyor", type="cylinder",
         radius=0.02, length=CONV_TOP - 0.06,
         xyz=(SPUR["x"] - 0.07, (FRONT_Y + SPUR["y_out"]) / 2, (CONV_TOP - 0.06) / 2), color=C["frame"]),
    dict(name="spur_leg_r", group="conveyor", type="cylinder",
         radius=0.02, length=CONV_TOP - 0.06,
         xyz=(SPUR["x"] + 0.07, (FRONT_Y + SPUR["y_out"]) / 2, (CONV_TOP - 0.06) / 2), color=C["frame"]),

    dict(name="conveyor_frame", group="conveyor", type="cuboid",
         size=(1.40, 0.20, 0.10), xyz=(-0.10, CONV_Y, CONV_TOP - 0.05), color=C["conveyor"]),
    dict(name="conveyor_belt", group="conveyor", type="cuboid",
         size=(1.38, 0.17, 0.012), xyz=(-0.10, CONV_Y, CONV_TOP + 0.006), color=C["belt"]),
    dict(name="roller_infeed", group="conveyor", type="cylinder",
         radius=0.040, length=0.17, xyz=(0.56, CONV_Y, CONV_TOP - 0.045), rpy=(pi / 2, 0, 0),
         color=C["frame"]),
    dict(name="roller_outfeed", group="conveyor", type="cylinder",
         radius=0.040, length=0.17, xyz=(-0.76, CONV_Y, CONV_TOP - 0.045), rpy=(pi / 2, 0, 0),
         color=C["frame"]),
    dict(name="conveyor_leg_fr", group="conveyor", type="cylinder",
         radius=0.025, length=CONV_TOP - 0.10, xyz=(0.52, CONV_Y + 0.07, (CONV_TOP - 0.10) / 2),
         color=C["frame"]),
    dict(name="conveyor_leg_fl", group="conveyor", type="cylinder",
         radius=0.025, length=CONV_TOP - 0.10, xyz=(0.52, CONV_Y - 0.07, (CONV_TOP - 0.10) / 2),
         color=C["frame"]),
    dict(name="conveyor_leg_br", group="conveyor", type="cylinder",
         radius=0.025, length=CONV_TOP - 0.10, xyz=(-0.72, CONV_Y + 0.07, (CONV_TOP - 0.10) / 2),
         color=C["frame"]),
    dict(name="conveyor_leg_bl", group="conveyor", type="cylinder",
         radius=0.025, length=CONV_TOP - 0.10, xyz=(-0.72, CONV_Y - 0.07, (CONV_TOP - 0.10) / 2),
         color=C["frame"]),

    dict(name="stand_base", group="panel", type="cylinder",
         radius=0.14, length=0.02, xyz=(0.95, -0.62, 0.01), color=C["frame"]),
    dict(name="stand_post", group="panel", type="cylinder",
         radius=0.035, length=0.95, xyz=(0.95, -0.62, 0.475), color=C["frame"]),
    dict(name="control_panel", group="panel", type="cuboid",
         size=(0.32, 0.08, 0.24), xyz=(0.95, -0.62, 1.03), rpy=(-0.40, 0, 0), color=C["panel"]),
    dict(name="panel_screen", group="panel", type="cuboid", parent="control_panel",
         size=(0.26, 0.008, 0.13), local_xyz=(0, -0.044, 0.045), color=C["screen"]),
    dict(name="panel_btn_start", group="panel", type="cylinder", parent="control_panel",
         radius=0.016, length=0.02, local_xyz=(-0.09, -0.048, -0.070), local_rpy=(pi / 2, 0, 0),
         color=C["go"]),
    dict(name="panel_btn_stop", group="panel", type="cylinder", parent="control_panel",
         radius=0.016, length=0.02, local_xyz=(-0.03, -0.048, -0.070), local_rpy=(pi / 2, 0, 0),
         color=C["warn"]),
    dict(name="panel_estop", group="safety", type="cylinder", parent="control_panel",
         radius=0.026, length=0.03, local_xyz=(0.08, -0.052, -0.068), local_rpy=(pi / 2, 0, 0),
         color=C["stop"]),

    dict(name="stacklight_post", group="safety", type="cylinder",
         radius=0.010, length=0.10, xyz=(0.60, 0.42, BENCH_H + GLASS_H + 0.05), color=C["frame"]),
    dict(name="stacklight_green", group="safety", type="cylinder",
         radius=0.030, length=0.05, xyz=(0.60, 0.42, BENCH_H + GLASS_H + 0.13), color=C["go"]),
    dict(name="stacklight_amber", group="safety", type="cylinder",
         radius=0.030, length=0.05, xyz=(0.60, 0.42, BENCH_H + GLASS_H + 0.18), color=C["warn"]),
    dict(name="stacklight_red", group="safety", type="cylinder",
         radius=0.030, length=0.05, xyz=(0.60, 0.42, BENCH_H + GLASS_H + 0.23), color=C["stop"]),
    dict(name="display_pose_marker", group="markers", type="cuboid",
         size=(0.08025, 0.00625, 0.13575), xyz=(0.30, -0.44, 1.32), color=C["target"]),
]


# ----------------------------------------------------------------------------
# SAFETY BARRIERS
# ----------------------------------------------------------------------------
BARRIER = dict(
    x0=-1.24, x1=1.26, y0=-1.02, y1=1.24,
    post=0.045, rail=0.035, heights=(0.42, 0.88), span=0.72,
    bays=((0.34, 1.26),),
)


def _run_specs(tag, p0, p1, b):
    px, py, pz = b["post"], b["post"], b["heights"][-1] + 0.05
    r = b["rail"]
    a, c = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    length = float(np.linalg.norm(c - a))
    if length < 1e-6:
        return []
    n = max(2, int(np.ceil(length / b["span"])) + 1)
    out = []
    for i, t in enumerate(np.linspace(0.0, 1.0, n)):
        x, y = a + t * (c - a)
        out.append(dict(name=f"barrier_{tag}_post{i}", group="barrier", type="cuboid",
                        size=(px, py, pz), xyz=(x, y, pz / 2), color=C["frame"]))
    mid = (a + c) / 2
    along_x = abs(c[0] - a[0]) > abs(c[1] - a[1])
    size = (length, r, r) if along_x else (r, length, r)
    for i, h in enumerate(b["heights"]):
        out.append(dict(name=f"barrier_{tag}_rail{i}", group="barrier", type="cuboid",
                        size=size, xyz=(mid[0], mid[1], h), color=C["tape"]))
    return out


def _front_runs(b):
    edges = [b["x0"]]
    for lo, hi in b["bays"]:
        edges += [lo, hi]
    edges.append(b["x1"])
    out = []
    for i in range(0, len(edges) - 1, 2):
        a, c = edges[i], edges[i + 1]
        if c - a > b["post"]:
            out.append((f"front{i // 2}", (a, b["y0"]), (c, b["y0"])))
    return out


def _barrier_specs():
    b = BARRIER
    runs = [("back", (b["x0"], b["y1"]), (b["x1"], b["y1"])),
            ("left", (b["x0"], b["y0"]), (b["x0"], b["y1"])),
            ("right", (b["x1"], b["y0"]), (b["x1"], b["y1"]))]
    runs += _front_runs(b)
    out = []
    for tag, p0, p1 in runs:
        out += _run_specs(tag, p0, p1, b)
    return out


# ----------------------------------------------------------------------------
# VAULT CARD STANDS
# ----------------------------------------------------------------------------

SLAB = dict(w=0.08025, t=0.00625, h=0.13575)

RACK = dict(
    n_racks=3,
    per_rack=4,
    gap=0.020,
    end_margin=0.015,
    centre_x=0.455,
    rack_y=(0.00, 0.10, 0.20),
    groove=0.009,
    base_h=0.015,
    lip_h=0.030,
    lip_d=0.018,
    back_h=0.050,
    back_d=0.025,
)
RACK["pitch_x"] = SLAB["w"] + RACK["gap"]
RACK["length"] = RACK["per_rack"] * SLAB["w"] + (RACK["per_rack"] - 1) * RACK["gap"] \
    + 2 * RACK["end_margin"]
RACK["depth"] = RACK["lip_d"] + RACK["groove"] + RACK["back_d"]


def _rack_specs():
    L, g, bh = RACK["length"], RACK["groove"], RACK["base_h"]
    x = RACK["centre_x"]
    out = []
    for i, yc in enumerate(RACK["rack_y"]):
        y_front = yc - g / 2 - RACK["lip_d"]
        y_back = yc + g / 2 + RACK["back_d"]
        out += [
            dict(name=f"rack{i}_base", group="vault", type="cuboid",
                 size=(L, y_back - y_front, bh),
                 xyz=(x, (y_front + y_back) / 2, BENCH_H + bh / 2), color=C["rack"]),
            dict(name=f"rack{i}_lip", group="vault", type="cuboid",
                 size=(L, RACK["lip_d"], RACK["lip_h"] - bh),
                 xyz=(x, y_front + RACK["lip_d"] / 2, BENCH_H + (bh + RACK["lip_h"]) / 2),
                 color=C["rack"]),
            dict(name=f"rack{i}_back", group="vault", type="cuboid",
                 size=(L, RACK["back_d"], RACK["back_h"] - bh),
                 xyz=(x, y_back - RACK["back_d"] / 2, BENCH_H + (bh + RACK["back_h"]) / 2),
                 color=C["rack"]),
        ]
    return out


STATIC.extend(_barrier_specs())
STATIC.extend(_rack_specs())


# ----------------------------------------------------------------------------
# HAND-OFF HOLDER
# ----------------------------------------------------------------------------
TRANSFER = dict(x=0.11, y=0.18)


def _transfer_specs():
    g, bh = RACK["groove"], RACK["base_h"]
    L = SLAB["w"] + 2 * RACK["end_margin"]
    x, yc = TRANSFER["x"], TRANSFER["y"]
    y_front = yc - g / 2 - RACK["lip_d"]
    y_back = yc + g / 2 + RACK["back_d"]
    return [
        dict(name="transfer_nest", group="pack", type="cuboid",
             size=(L, y_back - y_front, bh), xyz=(x, (y_front + y_back) / 2, BENCH_H + bh / 2),
             color=C["nest"]),
        dict(name="transfer_lip", group="pack", type="cuboid",
             size=(L, RACK["lip_d"], RACK["lip_h"] - bh),
             xyz=(x, y_front + RACK["lip_d"] / 2, BENCH_H + (bh + RACK["lip_h"]) / 2), color=C["nest"]),
        dict(name="transfer_back", group="pack", type="cuboid",
             size=(L, RACK["back_d"], RACK["back_h"] - bh),
             xyz=(x, y_back - RACK["back_d"] / 2, BENCH_H + (bh + RACK["back_h"]) / 2), color=C["nest"]),
    ]


def transfer_grasp_xyz(depth=0.030):
    return (TRANSFER["x"], TRANSFER["y"], BENCH_H + RACK["base_h"] + SLAB["h"] - depth)


# ----------------------------------------------------------------------------
# PACKAGING STATION
# ----------------------------------------------------------------------------
PACK = dict(
    holder_xy=(-0.22, 0.08),
    outfeed_xy=(-0.20, -0.09),
    inner=(0.090, 0.018, 0.150),
    wall=0.003,
    pad=(0.13, 0.09, 0.006),
    drop=0.045,
    grip_below_rim=0.020,
)
PACK["outer"] = (PACK["inner"][0] + 2 * PACK["wall"],
                 PACK["inner"][1] + 2 * PACK["wall"],
                 PACK["inner"][2] + PACK["wall"])


def _bag_specs(prefix="holder"):
    """The five parts of one bag standing at the packing station."""
    p = PACK
    ix, iy, iz = p["inner"]
    w = p["wall"]
    ox, oy, _ = p["outer"]
    hx, hy = p["holder_xy"]
    out = [dict(name=f"{prefix}_base", group="pack", type="cuboid", size=(ox, oy, w),
                xyz=(hx, hy, BENCH_H + w / 2), color=C["mailer"])]
    for tag, side in (("py", 1), ("my", -1)):
        out.append(dict(name=f"{prefix}_wall_{tag}", group="pack", type="cuboid",
                        size=(ox, w, iz),
                        xyz=(hx, hy + side * (iy + w) / 2, BENCH_H + w + iz / 2),
                        color=C["mailer"]))
    for tag, side in (("px", 1), ("mx", -1)):
        out.append(dict(name=f"{prefix}_wall_{tag}", group="pack", type="cuboid",
                        size=(w, iy, iz),
                        xyz=(hx + side * (ix + w) / 2, hy, BENCH_H + w + iz / 2),
                        color=C["mailer"]))
    return out


def bag_poses():
    """{part suffix: (x, y, z)} for a fresh bag at the packing station."""
    return {spec["name"].split("_", 1)[1]: spec["xyz"] for spec in _bag_specs("bag")}


def bag_names(prefix="holder"):
    return [f"{prefix}_base"] + [f"{prefix}_wall_{t}" for t in ("py", "my", "px", "mx")]


def _pack_specs():
    p = PACK
    out = _bag_specs("holder")
    out += [
        dict(name="outfeed_pad", group="pack", type="cuboid", size=p["pad"],
             xyz=(p["outfeed_xy"][0], p["outfeed_xy"][1], BENCH_H + p["pad"][2] / 2),
             color=C["nest"]),
    ]
    return out


def holder_slab_xyz():
    hx, hy = PACK["holder_xy"]
    return (hx, hy, BENCH_H + PACK["wall"] + SLAB["h"] / 2)


# ----------------------------------------------------------------------------
# SLAB MOUNTS
# ----------------------------------------------------------------------------
MOUNT = dict(
    shelf=(0.100, 0.044, 0.008),
    rib=(0.100, 0.005, 0.030),
    slot_clearance=0.0015,
    at={
        "return": (TRANSFER["x"], TRANSFER["y"] - 0.10, BENCH_H),
    },
    tint={"return": C["warn"]},
    placard=(0.100, 0.006, 0.040),
)
MOUNT["slot_y"] = SLAB["t"] / 2 + MOUNT["slot_clearance"] + MOUNT["rib"][1] / 2


def _mount_specs():
    sx, sy, sz = MOUNT["shelf"]
    rx, ry, rz = MOUNT["rib"]
    px, py, pz = MOUNT["placard"]
    out = []
    for name, (x, y, z) in MOUNT["at"].items():
        colour = MOUNT["tint"].get(name, C["rack"])
        out.append(dict(name=f"mount_{name}_shelf", group="mounts", type="cuboid",
                        size=(sx, sy, sz), xyz=(x, y, z + sz / 2), color=colour))
        for tag, side in (("py", 1), ("my", -1)):
            out.append(dict(name=f"mount_{name}_rib_{tag}", group="mounts", type="cuboid",
                            size=(rx, ry, rz),
                            xyz=(x, y + side * MOUNT["slot_y"], z + sz + rz / 2),
                            color=colour))
        if name in MOUNT["tint"]:
            out.append(dict(name=f"mount_{name}_placard", group="mounts", type="cuboid",
                            size=(px, py, pz),
                            xyz=(x, y - sy / 2 - py, z + pz / 2), color=colour))
    return out


def mount_slab_xyz(name):
    x, y, z = MOUNT["at"][name]
    return (x, y, z + MOUNT["shelf"][2] + SLAB["h"] / 2)


def mount_grasp_xyz(name, depth=0.030):
    x, y, z = mount_slab_xyz(name)
    return (x, y, z + SLAB["h"] / 2 - depth)


STATIC.extend(_transfer_specs())
STATIC.extend(_pack_specs())
STATIC.extend(_mount_specs())

SHOW_MARKERS = False

MESH_DIR = "meshes"


# ----------------------------------------------------------------------------
# POSE RESOLUTION
# ----------------------------------------------------------------------------
def _rot(rpy):
    import numpy as np
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def resolved_static():
    import numpy as np
    out, by_name = [], {}
    for spec in STATIC:
        spec = dict(spec)
        parent = spec.pop("parent", None)
        if parent is not None:
            p = by_name[parent]
            Rp = _rot(p.get("rpy", (0, 0, 0)))
            local = np.array(spec.pop("local_xyz", (0, 0, 0)), dtype=float)
            spec["xyz"] = tuple(np.array(p["xyz"], dtype=float) + Rp @ local)
            Rl = _rot(spec.pop("local_rpy", (0, 0, 0)))
            R = Rp @ Rl
            spec["rpy"] = (float(np.arctan2(R[2, 1], R[2, 2])),
                           float(np.arcsin(-max(-1.0, min(1.0, R[2, 0])))),
                           float(np.arctan2(R[1, 0], R[0, 0])))
        by_name[spec["name"]] = spec
        out.append(spec)
    return out

# ----------------------------------------------------------------------------
# VAULT SLOT GRID
# ----------------------------------------------------------------------------
CARD = dict(size=(SLAB["w"], SLAB["t"], SLAB["h"]))
SLOTS = dict(
    cols=RACK["per_rack"],
    rows=RACK["n_racks"],
    fill="all",
)


def slot_xyz(col, row):
    x0 = RACK["centre_x"] - (RACK["per_rack"] - 1) * RACK["pitch_x"] / 2
    return (x0 + col * RACK["pitch_x"],
            RACK["rack_y"][row],
            BENCH_H + RACK["base_h"] + SLAB["h"] / 2)


def slot_grasp_xyz(col, row, depth=0.030):
    x, y, z = slot_xyz(col, row)
    return (x, y, z + SLAB["h"] / 2 - depth)


def card_objects():
    fill = SLOTS["fill"]
    cells = ([(c, r) for c in range(SLOTS["cols"]) for r in range(SLOTS["rows"])]
             if fill == "all" else ([] if fill == "none" else list(fill)))
    return [dict(name=f"card_c{c}r{r}", group="cards", type="cuboid",
                 size=CARD["size"], xyz=slot_xyz(c, r),
                 color=C["card"]) for c, r in cells]


# ----------------------------------------------------------------------------
# ROBOTS
# ----------------------------------------------------------------------------

UR3E_HOME = [radians(a) for a in (90, -120, 60, -30, -90, 0)]
# The UR3e sits at the front left, on the far side of the outfeed pad from the
# camera: from the front right the camera is right between it and the pad, and at
# bench height its upper arm can't pass over the camera head. A riser (m) is
# optional; 0 means none.
UR3E_XY = (-0.45, -0.20)
UR3E_RISER = 0.0
KAWASAKI_HOME = [radians(a) for a in (98, 4, 95, 0, 89, -82)]

ROBOTS = {
    "A_vault": dict(label="Arm A - vault (Kawasaki RS007N)",
                    loader="models.Kawasaki_RS007N.Kawasaki_test:RS007N", base_xyz=ARM_A_BASE,
                    base_rpy=(0, 0, pi), q0=KAWASAKI_HOME, placeholder_h=0.45),
    "B_inspect": dict(label="Arm B - inspect and pack (Denso COBOTTA)",
                      loader="cobotta_test:COBOTTA", base_xyz=(-0.12, 0.18, BENCH_H),
                      base_rpy=(0, 0, -pi / 2), q0=None, placeholder_h=0.30),
    "C_handover": dict(label="UR3e + RG2 - showcase and outfeed (also the real robot)",
                   loader="models.ur3e_rg2:UR3eRG2", base_xyz=(*UR3E_XY, BENCH_H + UR3E_RISER),
                   base_rpy=(0, 0, pi / 2), q0=UR3E_HOME, placeholder_h=0.40),
}

if UR3E_RISER > 0:
    STATIC.append(dict(name="ur3e_riser", group="arm_bay", type="cuboid",
                       size=(0.14, 0.14, UR3E_RISER),
                       xyz=(*UR3E_XY, BENCH_H + UR3E_RISER / 2), color=C["pedestal"]))

SHOW_PEDESTALS = True
PEDESTAL = dict(radius=0.055, length=0.02)

# ----------------------------------------------------------------------------
# NAMED POSES
# ----------------------------------------------------------------------------
FRAMES = dict(
    rack_front=(RACK["centre_x"], RACK["rack_y"][0], BENCH_H + RACK["base_h"] + SLAB["h"]),
    rack_middle=(RACK["centre_x"], RACK["rack_y"][1], BENCH_H + RACK["base_h"] + SLAB["h"]),
    rack_back=(RACK["centre_x"], RACK["rack_y"][2], BENCH_H + RACK["base_h"] + SLAB["h"]),
    return_mount=mount_slab_xyz("return"),
    camera=(INSPECT_XY[0],
            INSPECT_XY[1] - CAM["standoff"] + CAM["lens_out"],
            BENCH_H + CAM["lens_z"]),
    holder=holder_slab_xyz(),
    outfeed=(PACK["outfeed_xy"][0], PACK["outfeed_xy"][1], BENCH_H + PACK["pad"][2]),
    transfer_nest=(TRANSFER["x"], TRANSFER["y"], BENCH_H + RACK["base_h"] + SLAB["h"]),
    display=(0.30, -0.44, 1.32),
    hatch=(HATCH["x"], FRONT_Y + PANEL_T, BENCH_H + HATCH["h"] / 2),
    spur_drop=(SPUR_DROP_XY[0], SPUR_DROP_XY[1], CONV_TOP + 0.012),
    conveyor_infeed=(SPUR["x"], CONV_Y, CONV_TOP + 0.03),
    conveyor_outfeed=(-0.76, CONV_Y, CONV_TOP + 0.03),
    pickup_tray=(-0.92, CONV_Y, CONV_TOP + 0.03),
    control_panel=(0.95, -0.66, 1.05),
)

GLASS_STANDOFF = 0.050
