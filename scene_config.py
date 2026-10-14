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
CELL_D = 1.20          # y  (was 1.00 - back wall moved out 0.20 to clear Arm A)
FRONT_Y = -0.50        # front glass (customer side) - unchanged
BACK_Y = FRONT_Y + CELL_D   # back glass / restock door
CELL_YC = (FRONT_Y + BACK_Y) / 2   # centre of the table and case in y
CASE_X1 = CELL_W / 2 - 0.10   # right side of the glass case (pulled in 0.10 so Arm A, outside, reaches the hand-off)
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
    "tape":      (0.98, 0.82, 0.08, 1.0),
}

# ----------------------------------------------------------------------------
# ARM A ROBOT BAY (outside the right glass, behind the customer panel)
# ----------------------------------------------------------------------------
ARM_A_BASE = (CASE_X1 + 0.10, 0.15, BENCH_H)   # Kawasaki base, on the bench extension
PORT_MARGIN = 0.05                             # glass left around the arm opening, at the roof and side walls
ARM_PORT = dict(                               # opening in the right glass: y range, height above bench
    y0=FRONT_Y + 1.5 * PANEL_T + PORT_MARGIN,              # 5 cm from the front glass
    y1=BACK_Y - 1.5 * PANEL_T - PORT_MARGIN,               # 5 cm from the back glass
    h=GLASS_H - PANEL_T / 2 - PORT_MARGIN,                 # 5 cm below the roof
)
BAY_BENCH_Y = (-0.10, 0.50)                    # bench extension under Arm A: y range
BAY = dict(x1=1.20, y0=-0.25, y1=0.65)         # floor warning tape around Arm A: right side x, front y, back y

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
         size=(2.6, 2.8, 0.02), xyz=(0, -0.1, 0.01), color=C["floor"]),

    # --- table / cabinet --------------------------------------------------
    dict(name="table_top", group="table", type="cuboid",
         size=(CELL_W, CELL_D, 0.05), xyz=(0, CELL_YC, BENCH_H - 0.025), color=C["table"]),
    dict(name="cabinet_body", group="table", type="cuboid",
         size=(CELL_W - 0.08, CELL_D - 0.08, BENCH_H - 0.05), xyz=(0, CELL_YC, (BENCH_H - 0.05) / 2),
         color=C["cabinet"]),

    # --- enclosure --------------------------------------------------------
    dict(name="glass_front", group="enclosure", type="cuboid",
         size=(CASE_X1 + CELL_W / 2, PANEL_T, GLASS_H),
         xyz=((CASE_X1 - CELL_W / 2) / 2, FRONT_Y + PANEL_T, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="glass_left", group="enclosure", type="cuboid",
         size=(PANEL_T, CELL_D, GLASS_H), xyz=(-CELL_W / 2 + PANEL_T, CELL_YC, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    # right glass: pulled in to CASE_X1, with an opening (ARM_PORT) for Arm A to reach through
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
         size=(CELL_W - 0.62, PANEL_T, GLASS_H), xyz=(-0.31, BACK_Y - PANEL_T, BENCH_H + GLASS_H / 2),
         color=C["glass"]),
    dict(name="restock_door", group="safety", type="cuboid",
         size=(CASE_X1 - 0.18, PANEL_T, GLASS_H), xyz=((0.18 + CASE_X1) / 2, BACK_Y - PANEL_T, BENCH_H + GLASS_H / 2),
         color=C["door"]),
    dict(name="roof", group="enclosure", type="cuboid",
         size=(CASE_X1 + CELL_W / 2, CELL_D, PANEL_T), xyz=((CASE_X1 - CELL_W / 2) / 2, CELL_YC, BENCH_H + GLASS_H),
         color=C["frame"]),

    # --- vault card stands: generated from RACK below (see _rack_specs) ----

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
    # transfer_nest (hand-off from Arm A to the COBOTTA): generated below as a one-slab holder

    # --- outfeed chute through the front wall -----------------------------
    dict(name="chute_sill", group="hatch", type="cuboid",
         size=(0.26, 0.03, 0.016), xyz=(0.45, FRONT_Y + PANEL_T, BENCH_H + 0.042), color=C["frame"]),
    dict(name="chute_head", group="hatch", type="cuboid",
         size=(0.26, 0.03, 0.016), xyz=(0.45, FRONT_Y + PANEL_T, BENCH_H + 0.238), color=C["frame"]),
    dict(name="chute_jamb_l", group="hatch", type="cuboid",
         size=(0.016, 0.03, 0.21), xyz=(0.327, FRONT_Y + PANEL_T, BENCH_H + 0.14), color=C["frame"]),
    dict(name="chute_jamb_r", group="hatch", type="cuboid",
         size=(0.016, 0.03, 0.21), xyz=(0.573, FRONT_Y + PANEL_T, BENCH_H + 0.14), color=C["frame"]),
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
         radius=0.026, length=0.03, xyz=(0.60, BACK_Y + 0.01, BENCH_H + 0.12), rpy=(pi / 2, 0, 0),
         color=C["stop"]),
    dict(name="stacklight_post", group="safety", type="cylinder",
         radius=0.010, length=0.10, xyz=(0.60, 0.42, BENCH_H + GLASS_H + 0.05), color=C["frame"]),
    dict(name="stacklight_green", group="safety", type="cylinder",
         radius=0.030, length=0.05, xyz=(0.60, 0.42, BENCH_H + GLASS_H + 0.13), color=C["go"]),
    dict(name="stacklight_amber", group="safety", type="cylinder",
         radius=0.030, length=0.05, xyz=(0.60, 0.42, BENCH_H + GLASS_H + 0.18), color=C["warn"]),
    dict(name="stacklight_red", group="safety", type="cylinder",
         radius=0.030, length=0.05, xyz=(0.60, 0.42, BENCH_H + GLASS_H + 0.23), color=C["stop"]),
    dict(name="keepout_zone", group="safety", type="cuboid",
         size=(1.00, 0.45, 0.004), xyz=(0.35, BACK_Y + 0.28, 0.022), color=C["keepout"]),

    # --- pose markers (set SHOW_MARKERS = False to hide) -------------------
    dict(name="display_pose_marker", group="markers", type="cuboid",
         size=(0.08025, 0.00625, 0.13575), xyz=(0.30, -0.44, 1.32), color=C["target"]),
]


# ----------------------------------------------------------------------------
# VAULT CARD STANDS
# ----------------------------------------------------------------------------
# Three wooden slot stands, one behind the other, each holding a row of slabs
# upright in a groove (like a trading-card display stand). Seen from the side:
#
#          back block                     slab
#            ____                          |
#   front   |    |                         |
#   lip __  |    |       <- slab sits in the groove between lip and block
#      |  |_|    |
#      |_________|  <- base
#
# Slabs face the customer (-y). Arm A picks them straight up out of the groove.

SLAB = dict(w=0.08025, t=0.00625, h=0.13575)  # graded slab: width (x), thickness (y), height (z)

RACK = dict(
    n_racks=3,              # stands in the case, front to back
    per_rack=4,             # slabs per stand
    gap=0.020,              # space between neighbouring slabs
    end_margin=0.015,       # wood past the first/last slab at each end
    centre_x=0.455,         # stand centre along x (slid right to make room for the hand-off holder)
    rack_y=(0.00, 0.10, 0.20),   # groove centreline of each stand (front -> back)
    groove=0.009,           # groove width (slab is 0.00625, so ~1.4 mm play each side)
    base_h=0.015,           # base plate height = groove floor
    lip_h=0.030,            # front lip top, above the bench
    lip_d=0.018,            # front lip depth (y)
    back_h=0.050,           # back block top, above the bench
    back_d=0.025,           # back block depth (y)
)
RACK["pitch_x"] = SLAB["w"] + RACK["gap"]                                   # 0.10025
RACK["length"] = RACK["per_rack"] * SLAB["w"] + (RACK["per_rack"] - 1) * RACK["gap"] \
    + 2 * RACK["end_margin"]                                               # 0.411
RACK["depth"] = RACK["lip_d"] + RACK["groove"] + RACK["back_d"]             # 0.052


def _rack_specs():
    """Three cuboids per stand: base plate, front lip, back block."""
    L, g, bh = RACK["length"], RACK["groove"], RACK["base_h"]
    x = RACK["centre_x"]
    out = []
    for i, yc in enumerate(RACK["rack_y"]):
        y_front = yc - g / 2 - RACK["lip_d"]          # front face of the stand
        y_back = yc + g / 2 + RACK["back_d"]          # back face of the stand
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


STATIC.extend(_rack_specs())


# ----------------------------------------------------------------------------
# HAND-OFF HOLDER (transfer_nest): one-slab stand where Arm A leaves a slab
# upright for the COBOTTA to pick from the top. Same profile as the vault stands.
# ----------------------------------------------------------------------------
TRANSFER = dict(x=0.17, y=0.18)                 # slot centre; within the COBOTTA's reach


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
    """Point between the gripper fingers for a slab standing in the hand-off holder."""
    return (TRANSFER["x"], TRANSFER["y"], BENCH_H + RACK["base_h"] + SLAB["h"] - depth)


# ----------------------------------------------------------------------------
# SLAB MOUNTS: slotted stands that hold a slab upright where it is set down
# ----------------------------------------------------------------------------
# Shelf plus two ribs. The slab stands on the shelf, the ribs stop it toppling,
# and the gripper takes it from the top edge well above them. Seen end on:
#
#     ||   ||     <- ribs, 30 mm tall
#     || | ||        | = slab in the slot
#   __||_|_||__
#  |___________|  <- shelf
#
# Add or move a mount by editing MOUNT["at"]: the key is the mount name and the
# value is the point the shelf sits on (usually the top face of a nest).
MOUNT = dict(
    shelf=(0.100, 0.044, 0.008),        # x, y, z of the shelf pad
    rib=(0.100, 0.005, 0.030),          # x, y, z of one rib
    slot_clearance=0.0015,              # rib face to slab face, each side
    at=dict(inspection=(-0.05, -0.05, BENCH_H + 0.012),   # on the inspection nest
            packing=(-0.25, -0.05, BENCH_H + 0.012)),     # on the bag tray
)
MOUNT["slot_y"] = SLAB["t"] / 2 + MOUNT["slot_clearance"] + MOUNT["rib"][1] / 2


def _mount_specs():
    """Three cuboids per mount: shelf pad, then a rib either side of the slot."""
    sx, sy, sz = MOUNT["shelf"]
    rx, ry, rz = MOUNT["rib"]
    out = []
    for name, (x, y, z) in MOUNT["at"].items():
        out.append(dict(name=f"mount_{name}_shelf", group="mounts", type="cuboid",
                        size=(sx, sy, sz), xyz=(x, y, z + sz / 2), color=C["rack"]))
        for tag, side in (("py", 1), ("my", -1)):
            out.append(dict(name=f"mount_{name}_rib_{tag}", group="mounts", type="cuboid",
                            size=(rx, ry, rz),
                            xyz=(x, y + side * MOUNT["slot_y"], z + sz + rz / 2),
                            color=C["rack"]))
    return out


def mount_slab_xyz(name):
    """Centre of a slab standing in that mount."""
    x, y, z = MOUNT["at"][name]
    return (x, y, z + MOUNT["shelf"][2] + SLAB["h"] / 2)


def mount_grasp_xyz(name, depth=0.030):
    """Point between the gripper fingers for a slab in that mount."""
    x, y, z = mount_slab_xyz(name)
    return (x, y, z + SLAB["h"] / 2 - depth)


# ----------------------------------------------------------------------------
# ROBOT BAY: bench extension under Arm A, warning tape on the floor around it
# ----------------------------------------------------------------------------
def _bay_specs():
    x0, x1 = CELL_W / 2, CELL_W / 2 + 0.20           # bench extension, flush with the table edge
    ey0, ey1 = BAY_BENCH_Y
    bx, fy0, fy1 = BAY["x1"], BAY["y0"], BAY["y1"]
    out = [
        dict(name="bay_bench_top", group="arm_bay", type="cuboid",
             size=(x1 - x0, ey1 - ey0, 0.05), xyz=((x0 + x1) / 2, (ey0 + ey1) / 2, BENCH_H - 0.025),
             color=C["table"]),
        dict(name="bay_bench_body", group="arm_bay", type="cuboid",
             size=(x1 - (x0 - 0.04), ey1 - ey0 - 0.04, BENCH_H - 0.05),
             xyz=((x0 - 0.04 + x1) / 2, (ey0 + ey1) / 2, (BENCH_H - 0.05) / 2), color=C["cabinet"]),
    ]
    # hazard tape on the floor: front, right and back of the bay (the bench closes the fourth side)
    fw, fz = 0.05, 0.02 + 0.001
    out += [
        dict(name="floor_tape_front", group="safety", type="cuboid",
             size=(bx - (x0 - 0.04) + fw / 2, fw, 0.002), xyz=((x0 - 0.04 + bx + fw / 2) / 2, fy0, fz), color=C["tape"]),
        dict(name="floor_tape_right", group="safety", type="cuboid",
             size=(fw, fy1 - fy0 - fw, 0.002), xyz=(bx, (fy0 + fy1) / 2, fz), color=C["tape"]),
        dict(name="floor_tape_back", group="safety", type="cuboid",
             size=(bx - (x0 - 0.04) + fw / 2, fw, 0.002), xyz=((x0 - 0.04 + bx + fw / 2) / 2, fy1, fz), color=C["tape"]),
    ]
    return out


STATIC.extend(_transfer_specs())
STATIC.extend(_mount_specs())
STATIC.extend(_bay_specs())

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
CARD = dict(size=(SLAB["w"], SLAB["t"], SLAB["h"]))   # graded slab: x wide, y thin, z tall
SLOTS = dict(
    cols=RACK["per_rack"],                     # slab position along a stand (0 = left, -x)
    rows=RACK["n_racks"],                      # which stand (0 = front, nearest the customer)
    fill="all",                                # "all", "none", or [(col, row), ...]
)


def slot_xyz(col, row):
    """Centre of the slab in slot (col, row): col = position on the stand, row = stand."""
    x0 = RACK["centre_x"] - (RACK["per_rack"] - 1) * RACK["pitch_x"] / 2
    return (x0 + col * RACK["pitch_x"],
            RACK["rack_y"][row],
            BENCH_H + RACK["base_h"] + SLAB["h"] / 2)


def slot_grasp_xyz(col, row, depth=0.030):
    """Point between the gripper fingers to grab slab (col, row): 'depth' down from its top edge."""
    x, y, z = slot_xyz(col, row)
    return (x, y, z + SLAB["h"] / 2 - depth)


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
KAWASAKI_HOME = [radians(a) for a in (98, 4, 95, 0, 89, -82)]     # reaching in, 30 cm above the middle stand, under the roof

ROBOTS = {
    "A_vault": dict(label="Arm A - vault (Kawasaki RS007N)",
                    loader="models.Kawasaki_RS007N.Kawasaki_test:RS007N", base_xyz=ARM_A_BASE,
                    base_rpy=(0, 0, pi), q0=KAWASAKI_HOME, placeholder_h=0.45),
    "B_inspect": dict(label="Arm B - inspect and pack (Denso COBOTTA)",
                      loader="cobotta_test:COBOTTA", base_xyz=(-0.12, 0.18, BENCH_H),
                      base_rpy=(0, 0, -pi / 2), q0=None, placeholder_h=0.30),
    "D_show": dict(label="UR3e + RG2 - showcase and outfeed (also the real robot)",
                   loader="models.ur3e_rg2:UR3eRG2", base_xyz=(0.30, -0.22, BENCH_H),
                   base_rpy=(0, 0, pi / 2), q0=UR3E_HOME, placeholder_h=0.40),
}

SHOW_PEDESTALS = True
PEDESTAL = dict(radius=0.055, length=0.02)

# ----------------------------------------------------------------------------
# NAMED POSES AND TASK FRAMES
# ----------------------------------------------------------------------------
FRAMES = dict(
    rack_front=(RACK["centre_x"], RACK["rack_y"][0], BENCH_H + RACK["base_h"] + SLAB["h"]),
    rack_middle=(RACK["centre_x"], RACK["rack_y"][1], BENCH_H + RACK["base_h"] + SLAB["h"]),
    rack_back=(RACK["centre_x"], RACK["rack_y"][2], BENCH_H + RACK["base_h"] + SLAB["h"]),
    inspection_nest=(-0.05, -0.05, BENCH_H + 0.02),
    inspection_mount=mount_slab_xyz("inspection"),
    packing_mount=mount_slab_xyz("packing"),
    camera=(-0.05, -0.05, 1.35),
    bag_tray=(-0.25, -0.05, BENCH_H + 0.02),
    mailer=(-0.22, 0.12, BENCH_H + 0.05),
    transfer_nest=(TRANSFER["x"], TRANSFER["y"], BENCH_H + RACK["base_h"] + SLAB["h"]),
    display=(0.30, -0.44, 1.32),
    chute=(0.45, -0.46, BENCH_H + 0.14),
    conveyor_infeed=(0.45, CONV_Y, CONV_TOP + 0.03),
    conveyor_outfeed=(-0.76, CONV_Y, CONV_TOP + 0.03),
    pickup_tray=(-0.92, CONV_Y, CONV_TOP + 0.03),
    control_panel=(0.95, -0.66, 1.05),
)

GLASS_STANDOFF = 0.050          # card face to glass, metres