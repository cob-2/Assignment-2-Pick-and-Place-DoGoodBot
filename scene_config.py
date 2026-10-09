"""Card Vault workcell - scene definition. Edit this file to change the scene."""

from math import pi, radians

# ----------------------------------------------------------------------------
# FRAME CONVENTION
# ----------------------------------------------------------------------------
# World origin: floor, centre of the kiosk footprint. Units: metres, radians.
#   +x = right (seen from the customer side)
#   +y = towards the staff/restock side (back)
#   -y = towards the customer, the showcase glass and the delivery conveyor
#   +z = up.  Bench top is at z = BENCH_H.

# ----------------------------------------------------------------------------
# MASTER DIMENSIONS
# ----------------------------------------------------------------------------
CELL_W = 1.60          # x
CELL_D = 1.00          # y
BENCH_H = 0.90         # table top height
GLASS_H = 1.20         # enclosure height above the bench
PANEL_T = 0.012        # panel thickness

CONV_Y = -0.78         # conveyor centreline, outside the front wall
CONV_TOP = 0.95        # conveyor belt height

# ----------------------------------------------------------------------------
# COLOURS  (r, g, b, alpha)
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
}

# ----------------------------------------------------------------------------
# STATIC OBJECTS
# ----------------------------------------------------------------------------
# type: "cuboid"   -> size (x, y, z)
# type: "cylinder" -> radius, length (length runs along local z; use rpy to turn)
# xyz = centre of the object, rpy = roll/pitch/yaw applied at that centre.
# group = used by the plan view and by the --hide switch in the builder.

STATIC = [
    # --- room -----------------------------------------------------------
    dict(name="floor_pad", group="room", type="cuboid",
         size=(2.6, 2.4, 0.02), xyz=(0, -0.3, 0.01), color=C["floor"]),

    # --- table / cabinet --------------------------------------------------
    dict(name="table_top", group="table", type="cuboid",
         size=(CELL_W, CELL_D, 0.05), xyz=(0, 0, BENCH_H - 0.025), color=C["table"]),
    dict(name="cabinet_body", group="table", type="cuboid",
         size=(CELL_W - 0.08, CELL_D - 0.08, BENCH_H - 0.05), xyz=(0, 0, (BENCH_H - 0.05) / 2),
         color=C["cabinet"]),

    # --- enclosure --------------------------------------------------------
    dict(name="glass_front", group="enclosure", type="cuboid",
         size=(CELL_W, PANEL_T, GLASS_H), xyz=(0, -CELL_D / 2 + PANEL_T, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="glass_left", group="enclosure", type="cuboid",
         size=(PANEL_T, CELL_D, GLASS_H), xyz=(-CELL_W / 2 + PANEL_T, 0, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="glass_right", group="enclosure", type="cuboid",
         size=(PANEL_T, CELL_D, GLASS_H), xyz=(CELL_W / 2 - PANEL_T, 0, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="glass_back", group="enclosure", type="cuboid",
         size=(CELL_W - 0.62, PANEL_T, GLASS_H), xyz=(-0.31, CELL_D / 2 - PANEL_T, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="restock_door", group="safety", type="cuboid",
         size=(0.60, PANEL_T, GLASS_H), xyz=(0.48, CELL_D / 2 - PANEL_T, BENCH_H + GLASS_H / 2),
         color=C["door"]),
    dict(name="roof", group="enclosure", type="cuboid",
         size=(CELL_W, CELL_D, PANEL_T), xyz=(0, 0, BENCH_H + GLASS_H), color=C["frame"]),

    # --- vault rack -------------------------------------------------------
    dict(name="vault_rack", group="vault", type="cuboid",
         size=(0.50, 0.16, 0.55), xyz=(0.42, 0.18, BENCH_H + 0.275), color=C["rack"]),

    # --- inspection station ----------------------------------------------
    dict(name="inspection_nest", group="inspect", type="cuboid",
         size=(0.12, 0.12, 0.012), xyz=(-0.05, -0.05, BENCH_H + 0.006), color=C["nest"]),
    dict(name="camera_body", group="inspect", type="cylinder",
         radius=0.025, length=0.07, xyz=(-0.05, -0.05, 1.35), rpy=(0, 0, 0), color=C["camera"]),
    dict(name="camera_mast", group="inspect", type="cylinder",
         radius=0.012, length=BENCH_H + GLASS_H - 1.385, xyz=(-0.05, -0.05, (1.385 + BENCH_H + GLASS_H) / 2),
         color=C["frame"]),
    dict(name="camera_bracket", group="inspect", type="cuboid",
         size=(0.06, 0.06, 0.012), xyz=(-0.05, -0.05, 1.392), color=C["frame"]),

    # --- packing station (worked by Arm B) --------------------------------
    dict(name="bag_tray", group="pack", type="cuboid",
         size=(0.14, 0.10, 0.012), xyz=(-0.25, -0.05, BENCH_H + 0.006), color=C["nest"]),
    dict(name="mailer_box", group="pack", type="cuboid",
         size=(0.16, 0.10, 0.04), xyz=(-0.22, 0.12, BENCH_H + 0.02), color=C["mailer"]),
    dict(name="transfer_nest", group="pack", type="cuboid",
         size=(0.12, 0.12, 0.012), xyz=(0.05, 0.00, BENCH_H + 0.006), color=C["nest"]),

    # --- outfeed chute through the front wall -----------------------------
    dict(name="chute_sill", group="hatch", type="cuboid",
         size=(0.26, 0.03, 0.016), xyz=(0.45, -CELL_D / 2 + PANEL_T, BENCH_H + 0.042), color=C["frame"]),
    dict(name="chute_head", group="hatch", type="cuboid",
         size=(0.26, 0.03, 0.016), xyz=(0.45, -CELL_D / 2 + PANEL_T, BENCH_H + 0.238), color=C["frame"]),
    dict(name="chute_jamb_l", group="hatch", type="cuboid",
         size=(0.016, 0.03, 0.21), xyz=(0.327, -CELL_D / 2 + PANEL_T, BENCH_H + 0.14), color=C["frame"]),
    dict(name="chute_jamb_r", group="hatch", type="cuboid",
         size=(0.016, 0.03, 0.21), xyz=(0.573, -CELL_D / 2 + PANEL_T, BENCH_H + 0.14), color=C["frame"]),
    dict(name="chute_ramp", group="hatch", type="cuboid",
         size=(0.22, 0.30, 0.010), xyz=(0.45, -0.62, BENCH_H + 0.09), rpy=(0.30, 0, 0),
         color=C["frame"]),

    # --- delivery conveyor, outside the vault -----------------------------
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
    dict(name="pickup_tray", group="conveyor", type="cuboid",
         size=(0.24, 0.20, 0.012), xyz=(-0.92, CONV_Y, CONV_TOP + 0.01), color=C["nest"]),

    # --- customer control panel on its own stand --------------------------
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

    # --- safety equipment -------------------------------------------------
    dict(name="curtain_post_l", group="safety", type="cylinder",
         radius=0.012, length=0.34, xyz=(-0.92, CONV_Y + 0.16, CONV_TOP + 0.17), color=C["frame"]),
    dict(name="curtain_post_r", group="safety", type="cylinder",
         radius=0.012, length=0.34, xyz=(-0.92, CONV_Y - 0.16, CONV_TOP + 0.17), color=C["frame"]),
    dict(name="curtain_beam", group="safety", type="cuboid",
         size=(0.006, 0.32, 0.006), xyz=(-0.92, CONV_Y, CONV_TOP + 0.17), color=C["beam"]),
    dict(name="estop_staff", group="safety", type="cylinder",
         radius=0.026, length=0.03, xyz=(0.70, CELL_D / 2 + 0.01, BENCH_H + 0.12), rpy=(pi / 2, 0, 0),
         color=C["stop"]),
    dict(name="stacklight_post", group="safety", type="cylinder",
         radius=0.010, length=0.10, xyz=(0.72, 0.42, BENCH_H + GLASS_H + 0.05), color=C["frame"]),
    dict(name="stacklight_green", group="safety", type="cylinder",
         radius=0.030, length=0.05, xyz=(0.72, 0.42, BENCH_H + GLASS_H + 0.13), color=C["go"]),
    dict(name="stacklight_amber", group="safety", type="cylinder",
         radius=0.030, length=0.05, xyz=(0.72, 0.42, BENCH_H + GLASS_H + 0.18), color=C["warn"]),
    dict(name="stacklight_red", group="safety", type="cylinder",
         radius=0.030, length=0.05, xyz=(0.72, 0.42, BENCH_H + GLASS_H + 0.23), color=C["stop"]),
    dict(name="keepout_zone", group="safety", type="cuboid",
         size=(1.00, 0.45, 0.004), xyz=(0.35, 0.78, 0.022), color=C["keepout"]),

    # --- pose markers (set SHOW_MARKERS = False to hide) -------------------
    dict(name="display_pose_marker", group="markers", type="cuboid",
         size=(0.076, 0.004, 0.102), xyz=(0.30, -0.44, 1.32), color=C["target"]),
]

SHOW_MARKERS = False

# Folder for imported CAD meshes, relative to this file.
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
    """STATIC with every parented object converted to absolute xyz/rpy."""
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
# VAULT SLOT GRID AND CARDS
# ----------------------------------------------------------------------------
CARD = dict(size=(0.076, 0.004, 0.102))        # toploader: x wide, y thin, z tall
SLOTS = dict(
    cols=5, rows=4,                            # grid size
    pitch_x=0.090, pitch_z=0.125,              # slot spacing
    origin=(0.24, 0.10, BENCH_H + 0.09),       # centre of slot (col 0, row 0)
    fill="all",                                # "all", "none", or [(col, row), ...]
)


def slot_xyz(col, row):
    """Centre of slot (col, row) in world coordinates."""
    ox, oy, oz = SLOTS["origin"]
    return (ox + col * SLOTS["pitch_x"], oy, oz + row * SLOTS["pitch_z"])


def card_objects():
    """Build the list of card objects from the slot grid."""
    fill = SLOTS["fill"]
    cells = ([(c, r) for c in range(SLOTS["cols"]) for r in range(SLOTS["rows"])]
             if fill == "all" else ([] if fill == "none" else list(fill)))
    return [dict(name=f"card_c{c}r{r}", group="cards", type="cuboid",
                 size=CARD["size"], xyz=slot_xyz(c, r), color=C["card"]) for c, r in cells]


# ----------------------------------------------------------------------------
# ROBOTS
# ----------------------------------------------------------------------------
# loader: "ir_support:UR3e"   -> from ir_support import UR3e
#         "models.mod:Class"  -> from models.mod import Class
#         None                -> placeholder block, so the scene still runs
# Anything that fails to import is drawn as a placeholder instead.

UR3E_HOME = [radians(a) for a in (90, -120, 60, -30, -90, 0)]   # tucked inside the cell

ROBOTS = {
    "A_vault": dict(label="Arm A - vault (Kawasaki RS007N)",
                    loader="models.rs007n:RS007N", base_xyz=(0.42, 0.42, BENCH_H),
                    base_rpy=(0, 0, -pi / 2), q0=None, placeholder_h=0.45),
    "B_inspect": dict(label="Arm B - inspect and pack (Denso COBOTTA)",
                      loader="models.cobotta:COBOTTA", base_xyz=(-0.12, 0.18, BENCH_H),
                      base_rpy=(0, 0, -pi / 2), q0=None, placeholder_h=0.30),
    "D_show": dict(label="UR3e - showcase and outfeed (also the real robot)",
                   loader="ir_support:UR3e", base_xyz=(0.30, -0.22, BENCH_H),
                   base_rpy=(0, 0, pi / 2), q0=UR3E_HOME, placeholder_h=0.40),
}

SHOW_PEDESTALS = True
PEDESTAL = dict(radius=0.055, length=0.02)

# ----------------------------------------------------------------------------
# NAMED POSES AND TASK FRAMES
# ----------------------------------------------------------------------------
FRAMES = dict(
    rack_front=(0.42, 0.10, BENCH_H + 0.28),
    rack_back=(0.42, 0.26, BENCH_H + 0.28),
    inspection_nest=(-0.05, -0.05, BENCH_H + 0.02),
    camera=(-0.05, -0.05, 1.35),
    bag_tray=(-0.25, -0.05, BENCH_H + 0.02),
    mailer=(-0.22, 0.12, BENCH_H + 0.05),
    transfer_nest=(0.05, 0.00, BENCH_H + 0.02),
    display=(0.30, -0.44, 1.32),
    chute=(0.45, -0.46, BENCH_H + 0.14),
    conveyor_infeed=(0.45, CONV_Y, CONV_TOP + 0.03),
    conveyor_outfeed=(-0.76, CONV_Y, CONV_TOP + 0.03),
    pickup_tray=(-0.92, CONV_Y, CONV_TOP + 0.03),
    control_panel=(0.95, -0.66, 1.05),
)

GLASS_STANDOFF = 0.050          # card face to glass, metres
